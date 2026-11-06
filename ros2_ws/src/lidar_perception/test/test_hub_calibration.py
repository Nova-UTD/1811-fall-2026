"""Behavior checks for operator-reviewed LiDAR-camera calibration."""
import importlib.util
import json
import sys
import time
from pathlib import Path
from threading import Lock
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from sensor_msgs_py import point_cloud2
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
    def test_interactive_mode_is_paused_until_started(self):
        hub = make_node()
        self.assertIsNone(hub.stopped_cameras())
        self.assertTrue(control(hub, 'start').success)
        self.assertEqual(hub.stopped_cameras(), set())

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

    def test_acceptances_are_serialized_and_applied_only_after_service_success(self):
        node = make_node()
        control(node, 'start')
        front, left = candidate('front', 0.10), candidate('left', 0.20)
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


if __name__ == '__main__':
    unittest.main()
