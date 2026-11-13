"""Behavior checks for operator-reviewed LiDAR-camera calibration."""
import importlib.util
from array import array
from collections import deque
import json
import sys
import tempfile
import time
from pathlib import Path
from threading import Lock
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Trigger


SOURCE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOURCE))
spec = importlib.util.spec_from_file_location("hub_calibration", SOURCE / "lidar_camera_registration.py")
calibration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(calibration)
import interactive_calibration as hub_module


def description():
    links = "".join(f'<link name="zed_{name}_camera_link"/>' for name in calibration.CAMERAS)
    joints = "".join(
        f'<joint name="zed_{name}_mount" type="floating"><parent link="base_link"/>'
        f'<child link="zed_{name}_camera_link"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>'
        for name in calibration.CAMERAS)
    return f'<robot name="vehicle_1811"><link name="base_link"/>{links}{joints}</robot>'


def candidate(name, x, ready=True):
    pose = np.eye(4)
    pose[0, 3] = x
    mount = np.eye(4)
    mount[0, 3] = x
    return {"frames": ("os_lidar", f"zed_{name}_frame"), "pose": pose,
            "mount": mount, "ready": ready, "revision": int(x * 100)}


def make_node():
    node = SimpleNamespace(
        config=calibration.DEFAULT_CONFIG._replace(interactive_calibration=True,
                                                   initial_pose_confirmation_enabled=False,
                                                   cloud_diagnostics_enabled=False),
        data_lock=Lock(), state_lock=Lock(), pending_candidates={}, retry_global=set(), camera_failures={},
        camera_poses={}, camera_mounts={}, camera_anchors={}, camera_clouds={}, latest_lidar=None,
        cloud_receipts={name: deque(maxlen=120) for name in ('lidar', *calibration.CAMERAS)},
        update_in_flight=False, pending_update=None, update_started=0.0, next_calibration=1.0,
        robot_description=description(), description_client=Mock(), get_logger=Mock(),
        tf_buffer=Mock(), output_urdf=Path('/tmp/not-used.urdf'),
        create_publisher=Mock(), create_service=Mock(), create_timer=Mock(),
    )
    node.description_client.service_is_ready.return_value = True
    algorithms = SimpleNamespace(**{name: getattr(calibration, name) for name in (
        'prepare_fpfh', 'log_fpfh_diagnostics', 'estimate_initial_pose', 'register_camera',
        'camera_mount_matrix', 'confirm_initial_pose', 'update_camera_joints', 'save_calibrated_urdf')})
    with patch.object(hub_module, 'TransformBroadcaster'):
        hub = hub_module.InteractiveCalibration(node, calibration.CAMERAS, algorithms)
    hub.status_pub = Mock()
    hub.preview_pubs = {name: Mock() for name in calibration.CAMERAS}
    for publisher in hub.preview_pubs.values():
        publisher.get_subscription_count.return_value = 1
    return hub


def control(node, action, name=None):
    return node.control(action, name, Trigger.Response())


class HubCalibrationTest(unittest.TestCase):
    def test_cloud_rates_use_recent_receipts_and_expire(self):
        hub = make_node()
        hub.node.cloud_receipts['lidar'].extend((98.0, 99.0, 99.5))
        hub.node.cloud_receipts['front'].extend((99.0, 99.1, 99.2))
        hub.node.cloud_receipts['rear'].extend((90.0, 91.0))
        with patch.object(hub_module.time, 'monotonic', return_value=100.0):
            hub.status()
        status = json.loads(hub.status_pub.publish.call_args.args[0].data)
        self.assertEqual(status['lidar_hz'], 1.3)
        self.assertEqual(status['cameras']['front']['cloud_hz'], 10.0)
        self.assertIsNone(status['cameras']['rear']['cloud_hz'])
        self.assertIsNone(status['cameras']['right']['cloud_hz'])

    def test_interactive_mode_is_paused_until_started(self):
        hub = make_node()
        self.assertIsNone(hub.stopped_cameras())
        self.assertTrue(control(hub, 'start').success)
        self.assertEqual(hub.stopped_cameras(), set())

    def test_stop_retains_accepted_and_queued_poses_and_discards_other_proposals(self):
        hub = make_node()
        control(hub, 'start')
        hub.accepted.add('front')
        queued = candidate('left', 0.20)
        hub.accept_queue['left'] = queued
        hub.candidates['left'] = candidate('left', 0.25)
        hub.candidates['rear'] = candidate('rear', 0.30)
        self.assertTrue(control(hub, 'review', 'rear').success)
        hub.node.pending_candidates['rear'] = {'poses': [np.eye(4)]}
        self.assertTrue(control(hub, 'stop').success)
        self.assertIsNone(hub.stopped_cameras())
        self.assertEqual(hub.accepted, {'front'})
        self.assertIs(hub.accept_queue['left'], queued)
        self.assertNotIn('left', hub.candidates)
        self.assertNotIn('rear', hub.candidates)
        self.assertNotIn('rear', hub.reviews)
        self.assertNotIn('rear', hub.node.pending_candidates)
        self.assertFalse(control(hub, 'accept', 'rear').success)
        hub.status()
        status = json.loads(hub.status_pub.publish.call_args.args[0].data)
        self.assertFalse(status['active'])
        self.assertEqual(status['cameras']['front']['state'], 'accepted')
        self.assertEqual(status['cameras']['left']['state'], 'applying')
        self.assertEqual(status['cameras']['rear']['state'], 'idle')

        future = Mock()
        future.result.return_value.results = [SimpleNamespace(successful=True, reason='')]
        hub.node.description_client.call_async.return_value = future
        hub.apply_queued()
        with patch.object(hub.algorithms, 'save_calibrated_urdf'):
            future.add_done_callback.call_args.args[0](future)
        self.assertIn('left', hub.accepted)
        control(hub, 'start')
        self.assertEqual(hub.stopped_cameras(), {'front', 'left'})

    def test_stop_recalibration_keeps_the_previous_accepted_transform(self):
        hub = make_node()
        control(hub, 'start')
        old = candidate('front', 0.10)
        hub.node.camera_poses['front'] = (*old['frames'], old['pose'])
        hub.accepted.add('front')
        control(hub, 'recompute', 'front')
        hub.candidates['front'] = candidate('front', 0.20)
        control(hub, 'stop')
        self.assertIn('front', hub.accepted)
        np.testing.assert_allclose(hub.node.camera_poses['front'][2], old['pose'])
        self.assertNotIn('front', hub.candidates)

    def test_stop_keeps_previous_pose_if_queued_replacement_fails(self):
        hub = make_node()
        control(hub, 'start')
        old = candidate('front', 0.10)
        hub.node.camera_poses['front'] = (*old['frames'], old['pose'])
        hub.accepted.add('front')
        control(hub, 'recompute', 'front')
        hub.accept_queue['front'] = candidate('front', 0.20)
        hub.candidates['front'] = candidate('front', 0.25)
        control(hub, 'stop')
        hub.status()
        status = json.loads(hub.status_pub.publish.call_args.args[0].data)
        self.assertEqual(status['cameras']['front']['state'], 'applying')

        future = Mock()
        future.result.return_value.results = [SimpleNamespace(successful=False, reason='rejected')]
        hub.node.description_client.call_async.return_value = future
        hub.apply_queued()
        future.add_done_callback.call_args.args[0](future)
        self.assertIn('front', hub.accepted)
        self.assertNotIn('front', hub.accept_queue)
        self.assertNotIn('front', hub.candidates)
        self.assertIn('rejected', hub.reasons['front'])
        np.testing.assert_allclose(hub.node.camera_poses['front'][2], old['pose'])
        control(hub, 'start')
        self.assertIn('front', hub.stopped_cameras())

    def test_stop_and_restart_discards_an_older_in_flight_calculation(self):
        hub = make_node()
        control(hub, 'start')
        _, session = hub.calibration_snapshot()
        header = Header(frame_id='os_lidar')
        camera_header = Header(frame_id='zed_front_frame')
        cameras = {'front': (SimpleNamespace(header=camera_header), Mock())}
        reasons = {name: 'no cloud' for name in calibration.CAMERAS}

        def stop_during_search(*_args):
            control(hub, 'stop')
            control(hub, 'start')
            return np.eye(4), None

        with patch.object(hub.algorithms, 'prepare_fpfh', return_value=(Mock(), Mock())), \
                patch.object(hub.algorithms, 'estimate_initial_pose', side_effect=stop_during_search), \
                patch.object(hub.algorithms, 'register_camera') as icp:
            hub.calibrate(Mock(), header, cameras, reasons, session)
        icp.assert_not_called()
        self.assertNotIn('front', hub.candidates)
        self.assertTrue(hub.active)
        self.assertNotEqual(session, hub.session)

    def test_review_freezes_the_actual_proposal_and_accept_requires_its_preview(self):
        node = make_node()
        self.assertFalse(control(node, 'review', 'front').success)
        self.assertTrue(control(node, 'start').success)
        first, newer = candidate('front', 0.10), candidate('front', 0.20)
        node.candidates['front'] = first
        self.assertTrue(control(node, 'review', 'front').success)
        node.candidates['front'] = newer  # Background computation keeps going.
        self.assertFalse(control(node, 'accept', 'front').success)

        header = Header(frame_id='zed_front_frame')
        header.stamp.sec = 10
        msg = point_cloud2.create_cloud_xyz32(header, [[1.0, 2.0, 3.0]])
        original = bytes(msg.data)
        node.publish_cloud('front', msg)
        sent = node.tf.sendTransform.call_args.args[0]
        self.assertAlmostEqual(sent.transform.translation.x, 0.10)
        preview = node.preview_pubs['front'].publish.call_args.args[0]
        self.assertEqual(preview.header.frame_id, 'hub_zed_front_cloud')
        self.assertEqual(msg.header.frame_id, 'zed_front_frame')
        self.assertEqual(bytes(msg.data), original)
        self.assertTrue(control(node, 'accept', 'front').success)
        self.assertIs(node.accept_queue['front'], first)
        self.assertNotIn('front', node.node.camera_poses)
        self.assertEqual(node.generation['front'], 1)

    def test_camera_previews_are_throttled_independently_per_camera(self):
        hub = make_node()
        hub.candidates['front'] = candidate('front', 0.10)
        hub.candidates['left'] = candidate('left', 0.20)
        front = point_cloud2.create_cloud_xyz32(Header(frame_id='zed_front_frame'), [[1.0, 2.0, 3.0]])
        left = point_cloud2.create_cloud_xyz32(Header(frame_id='zed_left_frame'), [[4.0, 5.0, 6.0]])
        original = bytes(front.data)
        with patch.object(hub_module.time, 'monotonic', return_value=10.0) as now:
            hub.publish_cloud('front', front)
            now.return_value = 10.1
            hub.publish_cloud('front', front)
            hub.publish_cloud('left', left)
            self.assertEqual(hub.preview_pubs['front'].publish.call_count, 1)
            self.assertEqual(hub.preview_pubs['left'].publish.call_count, 1)
            now.return_value = 10.5
            hub.publish_cloud('front', front)
        self.assertEqual(hub.preview_pubs['front'].publish.call_count, 2)
        self.assertEqual(hub.tf.sendTransform.call_count, 3)
        preview = hub.preview_pubs['front'].publish.call_args.args[0]
        self.assertEqual(bytes(preview.data), original)
        self.assertEqual(bytes(front.data), original)
        self.assertEqual(preview.header.frame_id, 'hub_zed_front_cloud')
        self.assertEqual(front.header.frame_id, 'zed_front_frame')

    def test_preview_sampling_preserves_fields_records_padding_and_source(self):
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='rgb', offset=4, datatype=PointField.UINT32, count=1)]
        # Each 8-byte point is distinguishable; each row also has padding.
        width, height, point_step, row_step = 5, 3, 8, 44
        data = bytearray(row_step * height)
        for row in range(height):
            for column in range(width):
                offset = row * row_step + column * point_step
                data[offset:offset + point_step] = bytes([row, column, 2, 3, 4, 5, 6, 7])
        msg = PointCloud2(header=Header(frame_id='camera'), width=width, height=height,
                          fields=fields, is_bigendian=True, point_step=point_step,
                          row_step=row_step, data=array('B', data), is_dense=False)
        preview = hub_module.sampled_preview_cloud(msg)
        self.assertEqual((preview.width, preview.height, preview.point_step, preview.row_step), (3, 2, 8, 24))
        self.assertEqual(preview.fields, fields)
        self.assertTrue(preview.is_bigendian)
        self.assertFalse(preview.is_dense)
        self.assertEqual(preview.header, msg.header)
        expected = b''.join(bytes(data[row * row_step + column * point_step:
                                       row * row_step + (column + 1) * point_step])
                            for row in (0, 2) for column in (0, 2, 4))
        self.assertEqual(bytes(preview.data), expected)
        self.assertEqual(bytes(msg.data), bytes(data))
        self.assertEqual((msg.width, msg.height, msg.row_step), (5, 3, 44))

    def test_unorganized_preview_sampling_and_review_preserve_full_cloud(self):
        hub = make_node()
        hub.candidates['front'] = candidate('front', 0.10)
        header = Header(frame_id='zed_front_frame')
        header.stamp.sec = 10
        msg = point_cloud2.create_cloud_xyz32(header, [[float(i), 2.0, 3.0] for i in range(9)])
        original = bytes(msg.data)
        hub.publish_cloud('front', msg)
        preview = hub.preview_pubs['front'].publish.call_args.args[0]
        self.assertEqual((preview.width, preview.height), (3, 1))
        self.assertEqual(bytes(preview.data), b''.join(original[i * msg.point_step:(i + 1) * msg.point_step]
                                                       for i in (0, 4, 8)))
        self.assertEqual(preview.header.stamp, msg.header.stamp)
        self.assertEqual(bytes(msg.data), original)
        self.assertTrue(control(hub, 'start').success)
        self.assertTrue(control(hub, 'review', 'front').success)
        hub.publish_cloud('front', msg)
        review = hub.preview_pubs['front'].publish.call_args.args[0]
        self.assertEqual((review.width, review.height), (msg.width, msg.height))
        self.assertEqual(bytes(review.data), original)
        self.assertTrue(control(hub, 'accept', 'front').success)

    def test_preview_sampling_rejects_invalid_layouts(self):
        msg = point_cloud2.create_cloud_xyz32(Header(frame_id='camera'), [[1., 2., 3.], [4., 5., 6.]])
        msg.row_step -= 1
        with self.assertRaises(ValueError):
            hub_module.sampled_preview_cloud(msg)
        msg.row_step += 1
        msg.data = array('B', [0])
        with self.assertRaises(ValueError):
            hub_module.sampled_preview_cloud(msg)

    def test_changed_pose_and_review_bypass_preview_throttle(self):
        hub = make_node()
        control(hub, 'start')
        hub.candidates['front'] = candidate('front', 0.10)
        msg = point_cloud2.create_cloud_xyz32(Header(frame_id='zed_front_frame'), [[1.0, 2.0, 3.0]])
        with patch.object(hub_module.time, 'monotonic', return_value=10.0) as now:
            hub.publish_cloud('front', msg)
            now.return_value = 10.1
            hub.candidates['front'] = candidate('front', 0.20)
            hub.publish_cloud('front', msg)
            self.assertTrue(control(hub, 'review', 'front').success)
            hub.publish_cloud('front', msg)
            self.assertEqual(hub.preview_pubs['front'].publish.call_count, 3)
            self.assertTrue(control(hub, 'accept', 'front').success)
            self.assertAlmostEqual(hub.accept_queue['front']['pose'][0, 3], 0.20)

    def test_removed_or_changed_frame_clears_the_old_preview_immediately(self):
        hub = make_node()
        hub.candidates['front'] = candidate('front', 0.10)
        msg = point_cloud2.create_cloud_xyz32(Header(frame_id='zed_front_frame'), [[1.0, 2.0, 3.0]])
        with patch.object(hub_module.time, 'monotonic', return_value=10.0):
            hub.publish_cloud('front', msg)
            msg.header.frame_id = 'another_camera_frame'
            hub.publish_cloud('front', msg)
        self.assertEqual(hub.preview_pubs['front'].publish.call_count, 2)
        cleared = hub.preview_pubs['front'].publish.call_args.args[0]
        self.assertEqual(cleared.header.frame_id, 'base_link')
        self.assertEqual(cleared.width, 0)

    def test_preview_throttling_stays_bounded_for_30_minutes_of_scans(self):
        hub = make_node()

        class CountingPublisher:
            def __init__(self):
                self.count = 0

            def get_subscription_count(self):
                return 1

            def publish(self, _message):
                self.count += 1

        hub.preview_pubs = {name: CountingPublisher() for name in calibration.CAMERAS}
        hub.tf = SimpleNamespace(sendTransform=lambda _transform: None)
        messages = {}
        for name in calibration.CAMERAS:
            hub.candidates[name] = candidate(name, 0.10)
            messages[name] = point_cloud2.create_cloud_xyz32(
                Header(frame_id=f'zed_{name}_frame'), [[1.0, 2.0, 3.0]])
        clock = [0.0]
        with patch.object(hub_module.time, 'monotonic', new=lambda: clock[0]):
            for tick in range(30 * 60 * 5):
                clock[0] = tick / 5.0
                for name in calibration.CAMERAS:
                    hub.publish_cloud(name, messages[name])
        for publisher in hub.preview_pubs.values():
            self.assertGreaterEqual(publisher.count, 30 * 60)
            self.assertLessEqual(publisher.count, 1 + 30 * 60 * 2)
        self.assertEqual(len(hub._last_preview), len(calibration.CAMERAS))

    def test_acceptances_are_serialized_and_applied_only_after_service_success(self):
        node = make_node()
        control(node, 'start')
        front, left = candidate('front', 0.10), candidate('left', 0.20)
        front['quality'] = {'icp_fitness': 0.8, 'icp_rmse_m': 0.02}
        node.accept_queue.update(front=front, left=left)
        futures = [Mock(), Mock()]
        for future in futures:
            future.result.return_value.results = [SimpleNamespace(successful=True, reason='')]
        node.node.description_client.call_async.side_effect = futures
        node.apply_queued()
        node.apply_queued()
        self.assertEqual(node.node.description_client.call_async.call_count, 1)
        self.assertNotIn('front', node.node.camera_poses)
        with patch.object(node.algorithms, 'save_calibrated_urdf'):
            futures[0].add_done_callback.call_args.args[0](futures[0])
        self.assertIn('front', node.accepted)
        np.testing.assert_allclose(node.node.camera_poses['front'][2], front['pose'])
        self.assertEqual(node.accepted_quality['front'], front['quality'])
        self.assertNotIn('left', node.accepted)
        node.apply_queued()
        self.assertEqual(node.node.description_client.call_async.call_count, 2)
        with patch.object(node.algorithms, 'save_calibrated_urdf'):
            futures[1].add_done_callback.call_args.args[0](futures[1])
        self.assertEqual(node.accepted, {'front', 'left'})
        self.assertEqual(set(node.node.camera_mounts), {'front', 'left'})

    def test_rejected_vehicle_update_does_not_mark_camera_accepted(self):
        node = make_node()
        control(node, 'start')
        node.accept_queue['front'] = candidate('front', 0.10)
        future = Mock()
        future.result.return_value.results = [SimpleNamespace(successful=False, reason='rejected')]
        node.node.description_client.call_async.return_value = future
        node.apply_queued()
        future.add_done_callback.call_args.args[0](future)
        self.assertNotIn('front', node.accepted)
        self.assertNotIn('front', node.node.camera_poses)
        self.assertNotIn('front', node.accept_queue)
        self.assertIn('rejected', node.reasons['front'])

    def test_save_failure_reports_that_the_live_calibration_is_active(self):
        node = make_node()
        control(node, 'start')
        node.accept_queue['front'] = candidate('front', 0.10)
        future = Mock()
        future.result.return_value.results = [SimpleNamespace(successful=True, reason='')]
        node.node.description_client.call_async.return_value = future
        node.apply_queued()
        with patch.object(node.algorithms, 'save_calibrated_urdf', side_effect=OSError('disk full')):
            future.add_done_callback.call_args.args[0](future)
        self.assertIn('front', node.accepted)
        self.assertIn('Live calibration active; could not save: disk full', node.reasons['front'])

    def test_recompute_unfreezes_one_camera_and_keeps_its_last_accepted_transform(self):
        node = make_node()
        control(node, 'start')
        old = candidate('front', 0.10)
        node.node.camera_poses['front'] = (*old['frames'], old['pose'])
        node.accepted.update(('front', 'left'))
        node.node.pending_candidates['front'] = {'poses': [old['pose']], 'frames': old['frames']}
        self.assertTrue(control(node, 'recompute', 'front').success)
        self.assertEqual(node.accepted, {'left'})
        self.assertIn('front', node.node.camera_poses)
        self.assertNotIn('front', node.node.pending_candidates)
        self.assertIn('front', node.node.retry_global)
        self.assertEqual(node.generation['front'], 1)
        self.assertEqual(node.node.next_calibration, 0.0)

    def test_proposals_continue_during_review_and_stop_after_acceptance(self):
        node = make_node()
        control(node, 'start')
        header = Header(frame_id='os_lidar')
        header.stamp.sec = 10
        camera_header = Header(frame_id='zed_front_frame')
        camera_header.stamp.sec = 10
        cameras = {'front': (SimpleNamespace(header=camera_header), Mock())}
        reasons = {name: 'no cloud' for name in calibration.CAMERAS}
        poses = [candidate('front', 0.10)['pose'], candidate('front', 0.20)['pose']]
        with patch.object(node.algorithms, 'prepare_fpfh', return_value=(Mock(), Mock())), \
                patch.object(node.algorithms, 'estimate_initial_pose', return_value=(np.eye(4), None)), \
                patch.object(node.algorithms, 'register_camera', side_effect=[(Mock(), pose, None) for pose in poses]) as icp, \
                patch.object(node.algorithms, 'camera_mount_matrix', side_effect=lambda _tf, _a, _b, _c, pose: pose.copy()):
            node.calibrate(Mock(), header, cameras, reasons)
            first = node.candidates['front']
            self.assertTrue(control(node, 'review', 'front').success)
            node.calibrate(Mock(), header, cameras, reasons)
            self.assertIs(node.reviews['front'], first)
            self.assertAlmostEqual(node.candidates['front']['pose'][0, 3], 0.20)
            node.review_published.add('front')
            node.review_published_at['front'] = time.monotonic()
            self.assertTrue(control(node, 'accept', 'front').success)
            node.calibrate(Mock(), header, cameras, reasons)
            self.assertEqual(icp.call_count, 2)

    def test_first_valid_proposal_can_be_reviewed_while_confirmations_continue(self):
        node = make_node()
        node.node.config = node.node.config._replace(initial_pose_confirmation_enabled=True,
                                                     initial_pose_confirmations=3)
        control(node, 'start')
        header = Header(frame_id='os_lidar')
        header.stamp.sec = 10
        camera_header = Header(frame_id='zed_front_frame')
        camera_header.stamp.sec = 10
        cameras = {'front': (SimpleNamespace(header=camera_header), Mock())}
        reasons = {name: 'no cloud' for name in calibration.CAMERAS}
        with patch.object(node.algorithms, 'prepare_fpfh', return_value=(Mock(), Mock())), \
                patch.object(node.algorithms, 'estimate_initial_pose', return_value=(np.eye(4), None)), \
                patch.object(node.algorithms, 'register_camera', return_value=(Mock(), np.eye(4), None)), \
                patch.object(node.algorithms, 'camera_mount_matrix', return_value=np.eye(4)):
            node.calibrate(Mock(), header, cameras, reasons)
        self.assertEqual(len(node.node.pending_candidates['front']['poses']), 1)
        self.assertTrue(node.candidates['front']['ready'])
        self.assertTrue(control(node, 'review', 'front').success)

    def test_stale_review_preview_cannot_be_accepted(self):
        node = make_node()
        control(node, 'start')
        node.candidates['front'] = candidate('front', 0.10)
        control(node, 'review', 'front')
        node.review_published.add('front')
        node.review_published_at['front'] = time.monotonic() - node.node.config.max_cloud_age_s - 1
        response = control(node, 'accept', 'front')
        self.assertFalse(response.success)
        self.assertIn('fresh', response.message)
        self.assertNotIn('front', node.accept_queue)

    def test_recompute_discards_a_calculation_already_in_flight(self):
        node = make_node()
        control(node, 'start')
        header = Header(frame_id='os_lidar')
        header.stamp.sec = 10
        camera_header = Header(frame_id='zed_front_frame')
        camera_header.stamp.sec = 10
        cameras = {'front': (SimpleNamespace(header=camera_header), Mock())}
        reasons = {name: 'no cloud' for name in calibration.CAMERAS}

        def retry_during_search(*_args):
            self.assertTrue(control(node, 'recompute', 'front').success)
            return np.eye(4), None

        with patch.object(node.algorithms, 'prepare_fpfh', return_value=(Mock(), Mock())), \
                patch.object(node.algorithms, 'estimate_initial_pose', side_effect=retry_during_search), \
                patch.object(node.algorithms, 'register_camera', return_value=(Mock(), np.eye(4), None)), \
                patch.object(node.algorithms, 'camera_mount_matrix', return_value=np.eye(4)):
            node.calibrate(Mock(), header, cameras, reasons)
        self.assertNotIn('front', node.candidates)

    def test_status_is_machine_readable_and_reports_camera_state(self):
        node = make_node()
        control(node, 'start')
        node.candidates['front'] = candidate('front', 0.10)
        node.accepted.add('left')
        node.status()
        status = json.loads(node.status_pub.publish.call_args.args[0].data)
        self.assertTrue(status['active'])
        self.assertEqual(status['cameras']['front']['state'], 'ready')
        self.assertEqual(status['cameras']['left']['state'], 'accepted')
        self.assertFalse(status['lidar_online'])

    def test_status_reports_the_frozen_review_and_previous_accepted_matrices(self):
        hub = make_node()
        control(hub, 'start')
        hub.candidates['front'] = candidate('front', 0.1)
        self.assertTrue(control(hub, 'review', 'front').success)
        hub.candidates['front'] = candidate('front', 0.2)
        hub.node.camera_poses['left'] = ('os_lidar', 'zed_left_frame', np.eye(4))
        hub.node.camera_mounts['left'] = np.eye(4)
        hub.status()
        status = json.loads(hub.status_pub.publish.call_args.args[0].data)
        front = status['cameras']['front']['transform']
        self.assertEqual(front['source'], 'review')
        self.assertEqual(front['cloud_to_lidar'][0][3], 0.1)
        self.assertEqual(front['cloud_frame'], 'zed_front_frame')
        self.assertEqual(status['cameras']['left']['transform']['source'], 'previous')
        hub.accepted.add('left')
        hub.status()
        status = json.loads(hub.status_pub.publish.call_args.args[0].data)
        self.assertEqual(status['cameras']['left']['transform']['source'], 'accepted')

    def test_status_never_emits_nonfinite_or_malformed_matrices(self):
        hub = make_node()
        hub.candidates['front'] = candidate('front', 0.1)
        hub.candidates['front']['pose'][0, 0] = np.nan
        hub.candidates['front']['mount'] = np.zeros((3, 3))
        hub.status()
        raw = hub.status_pub.publish.call_args.args[0].data
        self.assertNotIn('NaN', raw)
        transform = json.loads(raw)['cameras']['front']['transform']
        self.assertIsNone(transform['cloud_to_lidar'])
        self.assertIsNone(transform['mount_to_base'])

    def test_proposal_reports_quality_from_its_registration(self):
        hub = make_node()
        control(hub, 'start')
        header = Header(frame_id='os_lidar')
        camera_header = Header(frame_id='zed_front_frame')
        cameras = {'front': (SimpleNamespace(header=camera_header), Mock())}
        reasons = {name: 'no cloud' for name in calibration.CAMERAS}

        def estimate(_camera, _lidar, _config, metrics, _report):
            metrics.update(fitness=0.4, matches=50)
            return np.eye(4), None

        def register(_camera, _lidar, _initial, _config, metrics):
            metrics.update(fitness=0.8, rmse=0.02, matches=100)
            return Mock(), np.eye(4), None

        with patch.object(hub.algorithms, 'prepare_fpfh', return_value=(Mock(), Mock())), \
                patch.object(hub.algorithms, 'estimate_initial_pose', side_effect=estimate), \
                patch.object(hub.algorithms, 'register_camera', side_effect=register), \
                patch.object(hub.algorithms, 'camera_mount_matrix', return_value=np.eye(4)):
            hub.calibrate(Mock(), header, cameras, reasons)
        hub.status()
        quality = json.loads(hub.status_pub.publish.call_args.args[0].data)['cameras']['front']['transform']['quality']
        self.assertEqual(quality, {'ransac_fitness': 0.4, 'ransac_matches': 50,
                                   'icp_fitness': 0.8, 'icp_rmse_m': 0.02, 'icp_matches': 100})


class CalibrationRestartTest(unittest.TestCase):
    def test_accept_save_restart_restore_and_recompute_one_camera(self):
        with tempfile.TemporaryDirectory() as directory:
            first = make_node()
            first.node.output_urdf = Path(directory) / 'saved.urdf'
            first.accept_queue['front'] = candidate('front', 0.10)
            future = Mock()
            future.result.return_value.results = [SimpleNamespace(successful=True)]
            first.node.description_client.call_async.return_value = future
            first.apply_queued()
            future.add_done_callback.call_args.args[0](future)
            self.assertTrue(first.node.output_urdf.is_file())

            restarted = make_node()
            node = restarted.node
            node.saved_camera_mounts = calibration.calibrated_camera_mounts(first.node.output_urdf.read_text())
            node.restored_cameras, node.restore_retry_at, node.hub = set(), 0.0, restarted
            node.restore_update_done = lambda *args: calibration.LidarCameraRegistration.restore_update_done(node, *args)
            calibration.LidarCameraRegistration.restore_saved_calibration(node)
            restored = node.pending_update
            restored.result.return_value.results = [SimpleNamespace(successful=True)]
            restored.add_done_callback.call_args.args[0](restored)
            restarted.status()
            status = json.loads(restarted.status_pub.publish.call_args.args[0].data)
            self.assertEqual(status['cameras']['front']['state'], 'accepted')
            self.assertEqual(status['cameras']['left']['state'], 'idle')
            control(restarted, 'start')
            self.assertEqual(restarted.stopped_cameras(), {'front'})
            self.assertTrue(control(restarted, 'recompute', 'front').success)
            self.assertEqual(restarted.stopped_cameras(), set())
            np.testing.assert_allclose(node.camera_mounts['front'], candidate('front', 0.10)['mount'])

    def test_restore_in_progress_blocks_new_proposals_and_overlapping_updates(self):
        restarted = make_node()
        restarted.node.saved_camera_mounts = {'front': np.eye(4)}
        control(restarted, 'start')
        response = control(restarted, 'recompute', 'front')
        self.assertFalse(response.success)
        self.assertIn('restoring', response.message)
        restarted.accept_queue['left'] = candidate('left', 0.20)
        restarted.apply_queued()
        restarted.node.description_client.call_async.assert_not_called()


if __name__ == '__main__':
    unittest.main()
