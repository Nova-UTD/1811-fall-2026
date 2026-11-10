# Exercise the Python hub, ROS tools, and embedded RViz with simulated inputs.
import importlib.util
import json
import ctypes
import ctypes.util
import os
import select
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from array import array
from pathlib import Path
from unittest.mock import Mock, patch

try:
    from PyQt5 import QtCore, QtGui, QtNetwork, QtTest, QtWidgets, sip
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.executors import MultiThreadedExecutor
    from sensor_msgs.msg import Image, PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header, String
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage
    from vehicle_msgs.msg import VehicleCommand, VehicleState
    from visualization_msgs.msg import Marker
except ImportError as error:
    print(f'SKIP: Python RViz test dependencies unavailable: {error}')
    sys.exit(77)

if os.environ.get('NOVA_RVIZ_INTEGRATION') != '1' or not os.environ.get('DISPLAY'):
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'

source = Path(__file__).resolve().parents[1] / 'src' / 'rviz_viewport.py'
sys.path.insert(0, str(source.parent))
spec = importlib.util.spec_from_file_location('rviz_viewport', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
camera_spec = importlib.util.spec_from_file_location('camera_views', source.with_name('camera_views.py'))
camera_module = importlib.util.module_from_spec(camera_spec)
camera_spec.loader.exec_module(camera_module)
tools_spec = importlib.util.spec_from_file_location('developer_tools', source.with_name('developer_tools.py'))
tools_module = importlib.util.module_from_spec(tools_spec)
tools_spec.loader.exec_module(tools_module)
screens_spec = importlib.util.spec_from_file_location('screens', source.with_name('screens.py'))
screens_module = importlib.util.module_from_spec(screens_spec)
screens_spec.loader.exec_module(screens_module)
status_spec = importlib.util.spec_from_file_location('vehicle_status', source.with_name('vehicle_status.py'))
status_module = importlib.util.module_from_spec(status_spec)
status_spec.loader.exec_module(status_module)
calibration_spec = importlib.util.spec_from_file_location('calibration_details', source.with_name('calibration_details.py'))
calibration_module = importlib.util.module_from_spec(calibration_spec)
calibration_spec.loader.exec_module(calibration_module)
bag_spec = importlib.util.spec_from_file_location('bag_recorder', source.with_name('bag_recorder.py'))
bag_module = importlib.util.module_from_spec(bag_spec)
bag_spec.loader.exec_module(bag_module)
network_spec = importlib.util.spec_from_file_location('network_status', source.with_name('network_status.py'))
network_module = importlib.util.module_from_spec(network_spec)
network_spec.loader.exec_module(network_module)
sketch_spec = importlib.util.spec_from_file_location('sketch_pad', source.with_name('sketch_pad.py'))
sketch_module = importlib.util.module_from_spec(sketch_spec)
sketch_spec.loader.exec_module(sketch_module)
import hub as hub_module
app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(['rviz_bridge_test'])


class FakeCameraNode:
    def __init__(self):
        self.subscriptions = {}

    def create_subscription(self, message_type, topic, callback, qos, callback_group=None):
        subscription = (topic, callback)
        self.subscriptions[topic] = subscription
        return subscription

    def destroy_subscription(self, subscription):
        self.subscriptions.pop(subscription[0], None)


class CameraViewsTest(unittest.TestCase):
    def test_image_pixels_are_detached_and_invalid_stride_is_rejected(self):
        message = Image(width=1, height=1, step=3, encoding='rgb8')
        message.data = array('B', [120, 20, 40])
        frame = camera_module.CameraGallery.decode_image(message, QtCore.QSize(1, 1))
        message.data[0] = 0
        self.assertEqual(frame.pixelColor(0, 0).getRgb(), (120, 20, 40, 255))
        message.step = 2
        self.assertTrue(camera_module.CameraGallery.decode_image(message, QtCore.QSize(1, 1)).isNull())

    def test_drag_swap_and_subscription_lifecycle(self):
        node = FakeCameraNode()
        host = QtWidgets.QWidget()
        row = QtWidgets.QHBoxLayout(host)
        main = QtWidgets.QFrame()
        main.setObjectName('mainViewSlot')
        main_layout = QtWidgets.QVBoxLayout(main)
        cloud = camera_module.RecordingViewport(QtWidgets.QWidget())
        main_layout.addWidget(cloud)
        gallery = camera_module.CameraGallery(node, preview=True)
        gallery.set_main_view(main, cloud)
        row.addWidget(main, 2)
        row.addWidget(gallery, 1)
        host.resize(1200, 720)
        host.show()
        gallery.set_active(True)
        app.processEvents()
        self.assertEqual(len(node.subscriptions), 5)
        front = gallery._tiles[0]
        handle = front.drag_handle

        def mouse(target, kind, position, button, buttons):
            local = target.mapFromGlobal(position)
            event = QtGui.QMouseEvent(kind, QtCore.QPointF(local), QtCore.QPointF(position),
                                      button, buttons, QtCore.Qt.NoModifier)
            QtWidgets.QApplication.sendEvent(target, event)
            app.processEvents()

        start = handle.mapToGlobal(handle.rect().center())
        destination = main.mapToGlobal(main.rect().center())
        mouse(handle, QtCore.QEvent.MouseButtonPress, start, QtCore.Qt.LeftButton, QtCore.Qt.LeftButton)
        mouse(handle, QtCore.QEvent.MouseButtonRelease, start, QtCore.Qt.LeftButton, QtCore.Qt.NoButton)
        self.assertIs(main_layout.itemAt(0).widget(), cloud)
        mouse(handle, QtCore.QEvent.MouseButtonPress, start, QtCore.Qt.LeftButton, QtCore.Qt.LeftButton)
        mouse(handle, QtCore.QEvent.MouseMove, destination, QtCore.Qt.NoButton, QtCore.Qt.LeftButton)
        mouse(handle, QtCore.QEvent.MouseButtonRelease, destination, QtCore.Qt.LeftButton, QtCore.Qt.NoButton)
        self.assertIs(main_layout.itemAt(0).widget(), front)
        self.assertIs(gallery._slots[0].layout().itemAt(0).widget(), cloud)
        QtTest.QTest.qWait(50)
        geometry = (cloud.geometry(), cloud.layout().itemAt(1).widget().geometry())
        cloud.set_recording(True)
        app.processEvents()
        self.assertTrue(cloud.findChild(QtWidgets.QLabel, 'recordingBadge').isVisible())
        self.assertEqual(cloud.layout().contentsMargins().left(), 1)
        self.assertEqual((cloud.geometry(), cloud.layout().itemAt(1).widget().geometry()), geometry)
        cloud.set_recording(False)
        app.processEvents()
        self.assertEqual(cloud.findChild(QtWidgets.QLabel, 'recordingBadge').text(), '')
        self.assertEqual(cloud.layout().contentsMargins().left(), 1)
        self.assertEqual((cloud.geometry(), cloud.layout().itemAt(1).widget().geometry()), geometry)

        # Swapping a second camera moves the first one into that camera's slot.
        rear = gallery._tiles[2]
        rear_handle = rear.drag_handle
        rear_start = rear_handle.mapToGlobal(rear_handle.rect().center())
        mouse(rear_handle, QtCore.QEvent.MouseButtonPress, rear_start, QtCore.Qt.LeftButton, QtCore.Qt.LeftButton)
        mouse(rear_handle, QtCore.QEvent.MouseMove, destination, QtCore.Qt.NoButton, QtCore.Qt.LeftButton)
        mouse(rear_handle, QtCore.QEvent.MouseButtonRelease, destination, QtCore.Qt.LeftButton, QtCore.Qt.NoButton)
        self.assertIs(main_layout.itemAt(0).widget(), rear)
        self.assertIs(gallery._slots[2].layout().itemAt(0).widget(), front)
        self.assertIs(gallery._slots[0].layout().itemAt(0).widget(), cloud)

        # A drop outside the main view leaves every slot alone.
        front_start = handle.mapToGlobal(handle.rect().center())
        outside = host.mapToGlobal(QtCore.QPoint(-20, -20))
        mouse(handle, QtCore.QEvent.MouseButtonPress, front_start, QtCore.Qt.LeftButton, QtCore.Qt.LeftButton)
        mouse(handle, QtCore.QEvent.MouseMove, outside, QtCore.Qt.NoButton, QtCore.Qt.LeftButton)
        mouse(handle, QtCore.QEvent.MouseButtonRelease, outside, QtCore.Qt.LeftButton, QtCore.Qt.NoButton)
        self.assertIs(main_layout.itemAt(0).widget(), rear)

        # The compact point cloud uses the same handle to return to the main view.
        cloud_handle = cloud.drag_handle
        cloud_start = cloud_handle.mapToGlobal(cloud_handle.rect().center())
        mouse(cloud_handle, QtCore.QEvent.MouseButtonPress, cloud_start, QtCore.Qt.LeftButton, QtCore.Qt.LeftButton)
        mouse(cloud_handle, QtCore.QEvent.MouseMove, destination, QtCore.Qt.NoButton, QtCore.Qt.LeftButton)
        mouse(cloud_handle, QtCore.QEvent.MouseButtonRelease, destination, QtCore.Qt.LeftButton, QtCore.Qt.NoButton)
        self.assertIs(main_layout.itemAt(0).widget(), cloud)
        self.assertIs(gallery._slots[0].layout().itemAt(0).widget(), rear)

        front_start = handle.mapToGlobal(handle.rect().center())
        mouse(handle, QtCore.QEvent.MouseButtonPress, front_start, QtCore.Qt.LeftButton, QtCore.Qt.LeftButton)
        mouse(handle, QtCore.QEvent.MouseMove, destination, QtCore.Qt.NoButton, QtCore.Qt.LeftButton)
        mouse(handle, QtCore.QEvent.MouseButtonRelease, destination, QtCore.Qt.LeftButton, QtCore.Qt.NoButton)
        self.assertIs(main_layout.itemAt(0).widget(), front)

        gallery.set_active(True, False)
        self.assertEqual(set(node.subscriptions), {camera_module.TOPICS[0]})
        callback = node.subscriptions[camera_module.TOPICS[0]][1]
        message = Image(width=1, height=1, step=3, encoding='rgb8')
        message.data = array('B', [120, 20, 40])
        thread = threading.Thread(target=callback, args=(message,))
        thread.start()
        thread.join()
        app.processEvents()
        self.assertTrue(front.is_live())
        gallery.set_active(False, False)
        callback(message)
        app.processEvents()
        self.assertFalse(front.is_live())
        self.assertFalse(node.subscriptions)
        gallery.set_surround_visible(False)
        self.assertIs(main_layout.itemAt(0).widget(), cloud)
        gallery.set_surround_visible(True)
        self.assertIs(main_layout.itemAt(0).widget(), front)
        gallery.set_active(True, False)
        self.assertEqual(set(node.subscriptions), {camera_module.TOPICS[0]})
        gallery.set_active(False, False)
        host.close()

    def test_touch_drag_reaches_handle_inside_scroll_area(self):
        # PyQt5 does not expose QtTest's createTouchDevice helper. Register a
        # temporary Qt test device so events follow the actual window routing.
        library = ctypes.CDLL(ctypes.util.find_library('Qt5Gui'))
        try:
            register = getattr(library, '_ZN22QWindowSystemInterface19registerTouchDeviceEPK12QTouchDevice')
            unregister = getattr(library, '_ZN22QWindowSystemInterface21unregisterTouchDeviceEPK12QTouchDevice')
        except AttributeError:
            self.skipTest('Qt touch-device test hooks are unavailable')
        register.argtypes = unregister.argtypes = [ctypes.c_void_p]
        register.restype = unregister.restype = None
        device = QtGui.QTouchDevice()
        device.setType(QtGui.QTouchDevice.TouchScreen)
        device.setCapabilities(QtGui.QTouchDevice.Position)
        register(sip.unwrapinstance(device))
        window = hub_module.HubWindow(hub_module.PreviewScene(), None, None, True)
        try:
            window.resize(1200, 800)
            window.show()
            window.activateWindow()
            app.setActiveWindow(window)
            app.processEvents()
            front = window.gallery._tiles[0]
            handle = front.drag_handle
            main = window.findChild(QtWidgets.QFrame, 'mainViewSlot')
            start = window.mapFromGlobal(handle.mapToGlobal(handle.rect().center()))
            end = window.mapFromGlobal(main.mapToGlobal(main.rect().center()))
            cloud = main.layout().itemAt(0).widget()
            second = start + QtCore.QPoint(15, 0)
            QtTest.QTest.touchEvent(window, device).press(0, start, window).commit()
            QtTest.QTest.touchEvent(window, device).stationary(0).press(1, second, window).commit()
            QtTest.QTest.touchEvent(window, device).move(0, end, window).stationary(1).commit()
            QtTest.QTest.touchEvent(window, device).release(0, end, window).release(1, second, window).commit()
            app.processEvents()
            self.assertIs(main.layout().itemAt(0).widget(), cloud)
            QtTest.QTest.touchEvent(window, device).press(0, start, window).commit()
            QtTest.QTest.touchEvent(window, device).move(0, end, window).commit()
            QtTest.QTest.touchEvent(window, device).release(0, end, window).commit()
            app.processEvents()
            self.assertIs(main.layout().itemAt(0).widget(), front)
        finally:
            window.cleanup()
            window.close()
            unregister(sip.unwrapinstance(device))


class TouchInteractionTest(unittest.TestCase):
    def setUp(self):
        library = ctypes.CDLL(ctypes.util.find_library('Qt5Gui'))
        try:
            self.register = getattr(library, '_ZN22QWindowSystemInterface19registerTouchDeviceEPK12QTouchDevice')
            self.unregister = getattr(library, '_ZN22QWindowSystemInterface21unregisterTouchDeviceEPK12QTouchDevice')
        except AttributeError:
            self.skipTest('Qt touch-device test hooks are unavailable')
        self.register.argtypes = self.unregister.argtypes = [ctypes.c_void_p]
        self.register.restype = self.unregister.restype = None
        self.device = QtGui.QTouchDevice()
        self.device.setType(QtGui.QTouchDevice.TouchScreen)
        self.device.setCapabilities(QtGui.QTouchDevice.Position)
        self.register(sip.unwrapinstance(self.device))

    def tearDown(self):
        self.unregister(sip.unwrapinstance(self.device))

    def test_bag_name_keyboard_accepts_touch(self):
        dialog = bag_module.BagNameDialog('')
        try:
            dialog.show()
            dialog.activateWindow()
            app.setActiveWindow(dialog)
            app.processEvents()
            key = next(button for button in dialog.findChildren(QtWidgets.QPushButton, 'bagNameKey')
                       if button.text() == 'a')
            position = dialog.mapFromGlobal(key.mapToGlobal(key.rect().center()))
            QtTest.QTest.touchEvent(dialog, self.device).press(0, position, dialog).commit()
            QtTest.QTest.touchEvent(dialog, self.device).release(0, position, dialog).commit()
            app.processEvents()
            self.assertEqual(dialog.name_input.text(), 'a')
        finally:
            dialog.close()

    def test_touch_draws_immediately_without_waiting_for_release(self):
        pad = sketch_module.SketchPad()
        try:
            pad.resize(1000, 700)
            pad.show()
            pad.activateWindow()
            app.setActiveWindow(pad)
            app.processEvents()
            view = pad.canvas
            start = pad.mapFromGlobal(view.viewport().mapToGlobal(view.mapFromScene(QtCore.QPointF(300, 300))))
            end = pad.mapFromGlobal(view.viewport().mapToGlobal(view.mapFromScene(QtCore.QPointF(550, 300))))
            QtTest.QTest.touchEvent(pad, self.device).press(0, start, pad).commit()
            app.processEvents()
            self.assertEqual(len(view.scene().items()), 1)
            QtTest.QTest.touchEvent(pad, self.device).move(0, end, pad).commit()
            app.processEvents()
            self.assertGreater(view.scene().items()[0].path().elementCount(), 2)
            QtTest.QTest.touchEvent(pad, self.device).release(0, end, pad).commit()
            app.processEvents()
            self.assertFalse(view._dragging)
            self.assertEqual(view._history_index, 1)
        finally:
            pad.canvas.shutdown()
            pad.close()

    def test_swipe_on_button_scrolls_and_tap_still_clicks(self):
        host = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(host)
        area = QtWidgets.QScrollArea()
        area.setWidgetResizable(True)
        contents = QtWidgets.QWidget()
        rows = QtWidgets.QVBoxLayout(contents)
        buttons = [QtWidgets.QPushButton(f'Row {index}') for index in range(12)]
        for button in buttons:
            button.setFixedHeight(50)
            rows.addWidget(button)
        area.setWidget(contents)
        layout.addWidget(area)
        scroll_filter = hub_module.TouchScrollFilter(host)
        scroll_filter.add(area)
        clicks = []
        buttons[2].clicked.connect(lambda: clicks.append(True))
        try:
            host.resize(400, 260)
            host.show()
            host.activateWindow()
            app.setActiveWindow(host)
            app.processEvents()
            start = host.mapFromGlobal(buttons[2].mapToGlobal(buttons[2].rect().center()))
            end = start - QtCore.QPoint(0, 100)
            QtTest.QTest.touchEvent(host, self.device).press(0, start, host).commit()
            QtTest.QTest.touchEvent(host, self.device).move(0, end, host).commit()
            QtTest.QTest.touchEvent(host, self.device).release(0, end, host).commit()
            app.processEvents()
            self.assertGreater(area.verticalScrollBar().value(), 0)
            self.assertLess(area.verticalScrollBar().value(), 100)
            self.assertFalse(clicks)
            area.verticalScrollBar().setValue(0)
            app.processEvents()
            QtTest.QTest.touchEvent(host, self.device).press(0, start, host).commit()
            QtTest.QTest.touchEvent(host, self.device).release(0, start, host).commit()
            app.processEvents()
            self.assertEqual(len(clicks), 1)
        finally:
            app.removeEventFilter(scroll_filter)
            host.close()

    def test_touch_selects_a_graph_node_inside_a_scroll_panel(self):
        host = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(host)
        area = QtWidgets.QScrollArea()
        area.setWidgetResizable(True)
        node = FakeCameraNode()
        details = calibration_module.CalibrationDetails(node)
        area.setWidget(details)
        layout.addWidget(area)
        scroll_filter = hub_module.TouchScrollFilter(host)
        scroll_filter.add(area)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'base_link', 'zed_front_camera_link'
        transform.transform.rotation.w = 1.0
        node.subscriptions['/tf_static'][1](TFMessage(transforms=[transform]))
        try:
            host.resize(800, 700)
            host.show()
            host.activateWindow()
            app.setActiveWindow(host)
            details.refresh()
            area.ensureWidgetVisible(details._graph_area)
            app.processEvents()
            graph = details._graph
            point = graph._nodes['zed_front_camera_link'].center().toPoint()
            start = host.mapFromGlobal(graph.mapToGlobal(point))
            self.assertTrue(host.rect().contains(start))
            QtTest.QTest.touchEvent(host, self.device).press(0, start, host).commit()
            QtTest.QTest.touchEvent(host, self.device).release(0, start, host).commit()
            app.processEvents()
            self.assertEqual(details._selected_tf, 'zed_front_camera_link')
        finally:
            details.set_idle(True)
            app.removeEventFilter(scroll_filter)
            host.close()

    def test_nested_list_scrolls_in_pixels_instead_of_jumping_rows(self):
        host = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(host)
        area = QtWidgets.QScrollArea()
        area.setWidgetResizable(True)
        tree = QtWidgets.QTreeWidget()
        tree.setHeaderHidden(True)
        for index in range(80):
            QtWidgets.QTreeWidgetItem(tree, [f'Topic {index}'])
        area.setWidget(tree)
        layout.addWidget(area)
        scroll_filter = hub_module.TouchScrollFilter(host)
        scroll_filter.add(area)
        try:
            host.resize(400, 260)
            host.show()
            host.activateWindow()
            app.setActiveWindow(host)
            app.processEvents()
            start = host.mapFromGlobal(tree.viewport().mapToGlobal(QtCore.QPoint(100, 140)))
            end = start - QtCore.QPoint(0, 80)
            QtTest.QTest.touchEvent(host, self.device).press(0, start, host).commit()
            QtTest.QTest.touchEvent(host, self.device).move(0, end, host).commit()
            QtTest.QTest.touchEvent(host, self.device).release(0, end, host).commit()
            app.processEvents()
            self.assertEqual(tree.verticalScrollMode(), QtWidgets.QAbstractItemView.ScrollPerPixel)
            self.assertGreater(tree.verticalScrollBar().value(), 0)
            self.assertLess(tree.verticalScrollBar().value(), 80)
        finally:
            app.removeEventFilter(scroll_filter)
            host.close()


class PageTransitionTest(unittest.TestCase):
    def setUp(self):
        self.native = Mock()
        self.native.pause.return_value = self.native.resume.return_value = True
        self.window = hub_module.HubWindow(hub_module.PreviewScene(), self.native, None, True)
        self.window.resize(1200, 800)
        self.window.show()
        QtTest.QTest.qWait(80)

    def tearDown(self):
        self.window.cleanup()
        self.window.close()
        app.processEvents()

    def test_drawers_slide_at_fixed_width_and_reverse_without_a_jump(self):
        window = self.window
        deck = window.panel_deck
        frame = QtGui.QImage(2, 2, QtGui.QImage.Format_RGB888)
        frame.fill(QtGui.QColor('red'))
        front = window.gallery._tiles[0]
        front.set_frame(frame)
        width = deck.width()
        window.show_page('tools')
        QtTest.QTest.qWait(75)
        self.assertEqual(deck.width(), width)
        self.assertIs(deck.currentWidget(), window.tools_area)
        self.assertIs(deck._previous, window.camera_area)
        self.assertGreater(window.tools_area.x(), 0)
        self.assertLess(window.camera_area.x(), 0)
        self.assertTrue(front.is_live(), 'outgoing camera should not flash blank')
        viewport_size = window.view_frame.size()
        QtTest.QTest.qWait(25)
        self.assertEqual(window.view_frame.size(), viewport_size)

        camera_x, tools_x = window.camera_area.x(), window.tools_area.x()
        window.show_page('surround')
        self.assertLessEqual(abs(window.camera_area.x() - camera_x), 1)
        self.assertLessEqual(abs(window.tools_area.x() - tools_x), 1)
        self.assertTrue(front.is_live(), 'reversing should keep the latest frame')
        QtTest.QTest.qWait(350)
        self.assertIs(deck.currentWidget(), window.camera_area)
        self.assertIsNone(deck._previous)
        self.assertEqual(window.camera_area.x(), 0)

        window.show_page('tools')
        QtTest.QTest.qWait(60)
        positions = (window.camera_area.x(), window.tools_area.x())
        window.show_page('calibration')
        self.assertEqual((window.camera_area.x(), window.tools_area.x()), positions)
        self.assertIs(deck.currentWidget(), window.drawer_area)
        window.show_page('tools')
        QtTest.QTest.qWait(350)
        self.assertFalse(front.is_live(), 'hidden camera frames should be released')
        window.show_page('calibration')
        QtTest.QTest.qWait(60)
        window.resize(1000, 700)
        QtTest.QTest.qWait(350)
        self.assertEqual(deck.width(), window._panel_width())
        self.assertIs(deck.currentWidget(), window.drawer_area)
        self.assertEqual(window.drawer_area.geometry(), deck.rect())
        self.assertIsNone(deck._previous)

    def test_sketch_reversals_and_idle_settle_the_animation(self):
        window = self.window
        window.show_page('sketch')
        self.assertEqual(window.content_pages._transition.state(), QtCore.QAbstractAnimation.Stopped)
        self.native.set_rendering.assert_called_with(False)
        window.show_page('surround')
        self.native.set_rendering.assert_called_with(True)
        self.assertEqual(window.content_pages.currentIndex(), 0)

        window.show_page('sketch')
        QtTest.QTest.qWait(350)
        self.native.set_rendering.assert_called_with(False)
        window.show_page('tools')
        self.assertEqual(window.content_pages.currentIndex(), 0)
        window.enter_idle()
        self.assertTrue(window.idling)
        self.assertTrue(window.idle_screen.isVisible())
        self.assertEqual(window.idle_fade.state(), QtCore.QAbstractAnimation.Running)
        self.assertEqual(window.content_pages._transition.state(), QtCore.QAbstractAnimation.Stopped)
        self.assertEqual(window.panel_deck._transition.state(), QtCore.QAbstractAnimation.Stopped)
        window.wake_from_idle()
        self.assertFalse(window.idling)
        self.assertEqual(window.current_page, 'tools')
        self.assertEqual(window.idle_fade.state(), QtCore.QAbstractAnimation.Running)
        # Qt may deliver its final animation frame late under software rendering.
        deadline = time.monotonic() + 1.5
        while window.idle_screen.isVisible() and time.monotonic() < deadline:
            QtTest.QTest.qWait(25)
        self.assertFalse(window.idle_screen.isVisible())

    def test_sketch_is_a_developer_tool(self):
        window = self.window
        self.assertNotIn('sketch', window.nav_buttons)
        window.show_page('tools')
        window.findChild(QtWidgets.QPushButton, 'toolSketch').click()
        self.assertEqual(window.current_page, 'sketch')
        self.assertTrue(window.nav_buttons['tools'].isChecked())
        window.sketch.close_button.click()
        self.assertEqual(window.current_page, 'tools')

    def test_update_waits_for_recording_and_unsaved_sketch(self):
        window = self.window
        with patch.object(window.recorder, 'is_busy', return_value=True):
            self.assertEqual(window._update_restart_blocker(), 'Stop recording before updating.')
        with patch.object(window.sketch.canvas, 'has_unsaved_changes', return_value=True):
            self.assertEqual(window._update_restart_blocker(), 'Save your sketch before updating.')
        self.assertEqual(window._update_restart_blocker(), '')

    def test_theme_keeps_primary_and_disabled_controls_distinct(self):
        window = self.window
        original_theme = hub_module.white_theme()
        try:
            window.set_theme(False)
            window.show_page('calibration')
            QtTest.QTest.qWait(350)
            start = window.begin
            start.setEnabled(True)
            app.processEvents()

            def background():
                image = start.grab().toImage()
                return image.pixelColor(start.width() // 2, start.height() // 4).lightness()

            self.assertGreater(background(), 200)
            self.assertTrue(window.findChild(QtWidgets.QPushButton, 'themeBlack').isChecked())
            start.setEnabled(False)
            app.processEvents()
            self.assertLess(background(), 110)

            window.set_theme(True)
            start.setEnabled(True)
            app.processEvents()
            self.assertLess(background(), 110)
            self.assertTrue(window.findChild(QtWidgets.QPushButton, 'themeWhite').isChecked())
            self.assertFalse(window.findChild(QtWidgets.QPushButton, 'themeBlack').isChecked())
        finally:
            window.set_theme(original_theme)

    def test_status_messages_transition_without_stale_text_or_background(self):
        host = QtWidgets.QWidget()
        host.setStyleSheet('QWidget { background: #000000; color: #ffffff; }')
        layout = QtWidgets.QVBoxLayout(host)
        label = hub_module.AnimatedStatusLabel('Preparing')
        label.setStyleSheet('background: transparent; color: #ffffff;')
        layout.addWidget(label)
        host.resize(420, 100)
        host.show()
        QtTest.QTest.qWait(50)
        try:
            label.setText('Waiting for preview')
            self.assertEqual(label.text(), 'Waiting for preview')
            self.assertEqual(label._transition.state(), QtCore.QAbstractAnimation.Running)
            QtTest.QTest.qWait(60)
            # A newer update keeps the current timing and immediately replaces
            # the accessible value instead of queuing obsolete messages.
            label.setText('Preview ready')
            self.assertEqual(label.text(), 'Preview ready')
            image = host.grab().toImage()
            point = label.mapTo(host, QtCore.QPoint(2, 2))
            self.assertEqual(image.pixelColor(point).name(), '#000000')
            QtTest.QTest.qWait(350)
            self.assertEqual(label._transition.state(), QtCore.QAbstractAnimation.Stopped)
            self.assertTrue(label._previous.isNull())
            self.assertTrue(label._current.isNull())

            label.set_message('')
            self.assertTrue(label.isVisible())
            QtTest.QTest.qWait(350)
            self.assertFalse(label.isVisible())
            label.set_message('Accepted')
            self.assertTrue(label.isVisible())
            self.assertEqual(label.text(), 'Accepted')
            self.assertEqual(label._transition.state(), QtCore.QAbstractAnimation.Running)
            label.hide()
            self.assertEqual(label._transition.state(), QtCore.QAbstractAnimation.Stopped)
            self.assertTrue(label._previous.isNull())
            self.assertTrue(label._current.isNull())
        finally:
            host.close()

    def test_camera_summary_uses_the_latest_state(self):
        button = hub_module.CameraButton('Front', '#4fc1e6')
        button.resize(250, 90)
        button.show()
        QtTest.QTest.qWait(30)
        try:
            button.set_summary('Reviewing')
            self.assertEqual(button._summary_animation.state(), QtCore.QAbstractAnimation.Running)
            button.set_summary('Accepted')
            self.assertEqual(button.summary, 'Accepted')
            self.assertIn('Accepted', button.accessibleName())
            QtTest.QTest.qWait(350)
            self.assertEqual(button._summary_progress, 1.0)
        finally:
            button.close()

    def test_status_fade_inside_a_fading_panel_has_no_painter_errors(self):
        host = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(host)
        label = hub_module.AnimatedStatusLabel('Waiting')
        layout.addWidget(label)
        host.setGraphicsEffect(QtWidgets.QGraphicsOpacityEffect(host))
        host.resize(300, 100)
        host.show()
        QtTest.QTest.qWait(30)
        messages = []
        previous = QtCore.qInstallMessageHandler(lambda _kind, _context, message: messages.append(message))
        try:
            label.setText('Ready')
            QtTest.QTest.qWait(90)
            host.grab()
            QtTest.QTest.qWait(300)
        finally:
            QtCore.qInstallMessageHandler(previous)
            host.close()
        errors = [message for message in messages if 'QPainter::' in message or
                  'QWidgetEffectSourcePrivate::pixmap' in message]
        self.assertEqual(errors, [])

    def test_review_closes_and_reopens_during_its_fade(self):
        window = self.window
        window.show_page('calibration')
        QtTest.QTest.qWait(350)
        window._show_review()
        window._advance_review(0.25)
        self.assertEqual(window._review_opacity.opacity(), 0.0)
        window._advance_review(0.75)
        self.assertEqual(window.review_frame.maximumHeight(), window._review_height)
        QtTest.QTest.qWait(260)
        self.assertTrue(window.review_frame.isVisible())
        self.assertTrue(window.review_frame.isEnabled())
        window._hide_review()
        self.assertFalse(window.review_frame.isEnabled())
        QtTest.QTest.qWait(70)
        window._show_review()
        QtTest.QTest.qWait(260)
        self.assertTrue(window.review_frame.isVisible())
        self.assertTrue(window.review_frame.isEnabled())
        self.assertAlmostEqual(window._review_progress, 1.0)
        window._hide_review()
        QtTest.QTest.qWait(260)
        self.assertTrue(window.review_frame.isHidden())
        self.assertFalse(window._review_opacity.isEnabled())


class CalibrationSweepTest(unittest.TestCase):
    def test_activity_marker_starts_and_stops(self):
        node, publisher = Mock(), Mock()
        node.create_publisher.return_value = publisher
        node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
        sweep = hub_module.CalibrationSweep(node, None)
        sweep.set_active(True)
        marker = publisher.publish.call_args.args[0]
        self.assertEqual(marker.header.frame_id, 'base_link')
        self.assertEqual(marker.type, marker.TRIANGLE_LIST)
        self.assertGreater(len(marker.points), 100)
        self.assertLess(len(marker.points), 1200)
        self.assertEqual(len(marker.points), len(marker.colors))
        self.assertGreater(max(color.a for color in marker.colors), 0.2)
        self.assertTrue(sweep.timer.isActive())
        sweep.set_active(False)
        self.assertEqual(publisher.publish.call_args.args[0].action, marker.DELETE)
        self.assertFalse(sweep.timer.isActive())


class SoftwareUpdateTest(unittest.TestCase):
    def test_checked_revision_is_applied_and_branch_is_hidden(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'scripts').mkdir()
            script = root / 'scripts' / 'vehicle.sh'
            script.write_text(
                '#!/bin/bash\n'
                'python3 -c "import os; print(os.getsid(0))" > "$NOVA_TEST_UPDATE_SESSION"\n'
                'if [ "$1" = check-update ]; then\n'
                '  printf "%s\\n" "$NOVA_TEST_UPDATE_CHECK"\n'
                'elif [ "$1" = check-jetson-update ]; then\n'
                '  printf "%s\\n" "$NOVA_TEST_JETSON_CHECK"\n'
                'fi\n', encoding='utf-8')
            target = 'a' * 40
            session = root / '.nova-ui-startup' / 'run.ABC123'
            session.mkdir(parents=True)
            (session / 'supervisor_pid').write_text('12345\n')
            env = {
                'NOVA_1811_STACK_ROOT': str(root),
                'NOVA_1811_STACK_BRANCH': 'blackwall',
                'NOVA_UI_SUPERVISOR_TOKEN': session.name,
                'NOVA_TEST_UPDATE_CHECK': f'AVAILABLE {target} 2 READY\nImprove sensors\n<unsafe text>\nFix blackwall',
                'NOVA_TEST_JETSON_CHECK': 'JETSON_DIRTY',
                'NOVA_TEST_UPDATE_SESSION': str(root / 'session'),
            }
            with patch.dict(os.environ, env), patch.object(tools_module.os, 'kill') as signal_launcher:
                panel = tools_module.SoftwareUpdate()
                panel.show()
                app.processEvents()
                action = panel.findChild(QtWidgets.QPushButton, 'updateAction')
                changes = panel.findChild(QtWidgets.QLabel, 'updateChanges')
                status = panel.findChild(QtWidgets.QLabel, 'updateStatus')

                def wait_finished():
                    end = time.monotonic() + 5
                    while panel.is_busy() and time.monotonic() < end:
                        app.processEvents()
                        time.sleep(0.01)
                    app.processEvents()
                    self.assertFalse(panel.is_busy())

                action.click()
                wait_finished()
                self.assertEqual(action.text(), 'Update now')
                self.assertEqual(changes.textFormat(), QtCore.Qt.PlainText)
                self.assertIn('<unsafe text>', changes.text())
                self.assertIn('[branch]', changes.text())
                self.assertNotIn('blackwall', changes.text())
                self.assertNotEqual(int((root / 'session').read_text()), os.getsid(0))
                action.click()
                wait_finished()
                self.assertEqual(status.text(), 'Update unavailable')
                self.assertIn('Jetson has local changes', panel.findChild(QtWidgets.QLabel, 'updateDetail').text())
                signal_launcher.assert_not_called()
                self.assertFalse((session / 'request').exists())
                os.environ['NOVA_TEST_JETSON_CHECK'] = 'JETSON_READY'
                action.click()
                wait_finished()
                self.assertEqual(action.text(), 'Update now')
                action.click()
                end = time.monotonic() + 5
                while not (session / 'request').exists() and time.monotonic() < end:
                    app.processEvents()
                    time.sleep(0.01)
                self.assertEqual((session / 'request').read_text().strip(), target)
                signal_launcher.assert_called_once_with(12345, tools_module.signal.SIGUSR1)
                (session / 'accepted').touch()
                wait_finished()
                self.assertEqual(status.text(), 'Restarting…')
                self.assertFalse(action.isEnabled())
                panel.close()


class BagRecorderTest(unittest.TestCase):
    def test_keyboard_accepts_or_cancels_the_name(self):
        panel = bag_module.RosbagRecorder(None, preview=True)
        panel._name.setText('Original')
        panel.show()

        def edit_and_close(accept):
            dialog = QtWidgets.QApplication.activeModalWidget()
            self.assertIsInstance(dialog, bag_module.BagNameDialog)
            dialog.name_input.setText('New name')
            dialog.accept() if accept else dialog.reject()

        try:
            QtCore.QTimer.singleShot(0, lambda: edit_and_close(False))
            panel._name_keyboard.click()
            self.assertEqual(panel._name.text(), 'Original')
            QtCore.QTimer.singleShot(0, lambda: edit_and_close(True))
            panel._name_keyboard.click()
            self.assertEqual(panel._name.text(), 'New name')
        finally:
            panel.set_idle(True)
            panel.close()

    def test_record_and_save_selected_topic(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fake_ros = root / 'ros2'
            fake_ros.write_text(
                '#!/usr/bin/env python3\n'
                'import os, signal, sys, time\n'
                'if "--help" in sys.argv:\n'
                '    print("usage: ros2 bag record --topics TOPIC")\n'
                '    sys.exit(0)\n'
                'path = sys.argv[sys.argv.index("-o") + 1]\n'
                'open(os.environ["NOVA_TEST_BAG_ARGS"], "w").write("\\n".join(sys.argv))\n'
                'stopping = False\n'
                'def stop(*args):\n'
                '    global stopping\n'
                '    stopping = True\n'
                'signal.signal(signal.SIGINT, stop)\n'
                'os.mkdir(path)\n'
                'while not stopping: time.sleep(0.02)\n'
                'open(os.path.join(path, "metadata.yaml"), "w").write("saved")\n', encoding='utf-8')
            fake_ros.chmod(0o755)
            env = {'PATH': str(root) + os.pathsep + os.environ['PATH'], 'NOVA_TEST_BAG_ARGS': str(root / 'args')}
            with patch.dict(os.environ, env):
                panel = bag_module.RosbagRecorder(None)
                panel.set_directory(str(root / 'bags'))
                panel.show()
                app.processEvents()
                topics = panel.findChild(QtWidgets.QTreeWidget, 'recordingTopics')
                topic = topics.findItems('/ouster/points', QtCore.Qt.MatchExactly, 0)[0]
                topic.setCheckState(0, QtCore.Qt.Checked)
                panel._name.setText('Parking test')
                panel.start_recording()

                def wait_for(predicate, timeout=7):
                    end = time.monotonic() + timeout
                    while time.monotonic() < end:
                        app.processEvents()
                        if predicate():
                            return True
                        time.sleep(0.01)
                    return False

                self.assertTrue(wait_for(lambda: panel._state == 'recording'), panel._message.text())
                self.assertEqual(Path(panel._bag_path).name, 'Parking test')
                self.assertFalse(panel._name.isEnabled())
                self.assertIn('--topics\n/ouster/points', (root / 'args').read_text())
                first_bag = Path(panel._bag_path)
                finished = []
                panel.stop_and_then(lambda: finished.append(True))
                self.assertTrue(wait_for(lambda: bool(finished)), panel._message.text())
                self.assertFalse(panel.is_busy())
                self.assertTrue((Path(panel._bag_path) / 'metadata.yaml').exists())
                self.assertTrue(panel._message.text().startswith('Saved: '))
                self.assertTrue(panel._name.isEnabled())
                panel.start_recording()
                self.assertTrue(wait_for(lambda: panel._state == 'recording'), panel._message.text())
                self.assertEqual(Path(panel._bag_path).name, 'Parking test_1')
                panel.stop_recording()
                self.assertTrue(wait_for(lambda: not panel.is_busy()), panel._message.text())
                self.assertTrue((first_bag / 'metadata.yaml').exists())
                self.assertTrue((Path(panel._bag_path) / 'metadata.yaml').exists())
                panel.set_idle(True)
                panel.close()

    def test_name_rejects_paths_and_leaves_empty_name_for_timestamp(self):
        panel = bag_module.RosbagRecorder(None)
        with tempfile.TemporaryDirectory() as directory:
            panel.set_directory(directory)
            panel._selected.add('/ouster/points')
            try:
                for name in ('../escape', '.', 'folder\\name', '界' * 120):
                    with self.subTest(name=name):
                        panel._name.setText(name)
                        panel.start_recording()
                        self.assertEqual(panel._state, 'idle')
                        self.assertTrue(panel._message.property('recordingError'))
                        self.assertFalse(list(Path(directory).iterdir()))
                panel._name.clear()
                # Inspect the chosen path without starting a subprocess.
                with patch.object(panel, '_probe'):
                    panel.start_recording()
                self.assertTrue(Path(panel._bag_path).name.startswith('bag_'))
                panel._finish_idle('Canceled')
                dangling = Path(directory) / 'existing'
                dangling.symlink_to(Path(directory) / 'missing')
                panel._name.setText('existing')
                with patch.object(panel, '_probe'):
                    panel.start_recording()
                self.assertEqual(Path(panel._bag_path).name, 'existing_1')
                self.assertTrue(dangling.is_symlink())
                panel._finish_idle('Canceled')
            finally:
                panel.set_idle(True)
                panel.close()

    @unittest.skipUnless(sys.platform.startswith('linux'), 'parent-death signals are Linux-specific')
    def test_recorder_guard_interrupts_child_when_parent_dies(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            child = root / 'child.py'
            child.write_text(
                'import os, signal, time\n'
                'from pathlib import Path\n'
                'def stop(*args):\n'
                '    Path(os.environ["NOVA_TEST_STOPPED"]).write_text("stopped")\n'
                '    raise SystemExit(0)\n'
                'signal.signal(signal.SIGINT, stop)\n'
                'Path(os.environ["NOVA_TEST_READY"]).write_text("ready")\n'
                'time.sleep(5)\n', encoding='utf-8')
            parent = root / 'parent.py'
            parent.write_text(
                'import os, subprocess, sys, time\n'
                'from pathlib import Path\n'
                f'subprocess.Popen([sys.executable, {str(source.with_name("bag_recorder.py"))!r}, '
                f'"--record-guard", str(os.getpid()), sys.executable, {str(child)!r}])\n'
                'deadline = time.monotonic() + 4\n'
                'while not Path(os.environ["NOVA_TEST_READY"]).exists() and time.monotonic() < deadline:\n'
                '    time.sleep(0.01)\n'
                'os._exit(0)\n', encoding='utf-8')
            env = dict(os.environ, NOVA_TEST_READY=str(root / 'ready'), NOVA_TEST_STOPPED=str(root / 'stopped'))
            subprocess.run([sys.executable, str(parent)], env=env, timeout=8, check=True)
            end = time.monotonic() + 3
            while not (root / 'stopped').exists() and time.monotonic() < end:
                time.sleep(0.01)
            self.assertTrue((root / 'stopped').exists(), 'recorder did not receive SIGINT after its parent exited')


class RealBagTest(unittest.TestCase):
    @unittest.skipUnless(shutil.which('ros2'), 'ros2 command is unavailable')
    def test_selected_topic_is_recorded_and_closed(self):
        # Use temporary storage and a synthetic topic; no vehicle or device is
        # needed to check the real rosbag CLI and its metadata.
        import yaml

        rclpy.init(args=['hub_bag_test'])
        node = rclpy.create_node('hub_bag_test_ui')
        publisher_node = rclpy.create_node('hub_bag_test_source')
        topic = '/nova_bag_test_' + str(os.getpid())
        publisher = publisher_node.create_publisher(String, topic, 10)
        last_publish = 0.0

        def wait_for(predicate, timeout=12):
            nonlocal last_publish
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                app.processEvents()
                if time.monotonic() - last_publish >= 0.1:
                    publisher.publish(String(data='test message'))
                    last_publish = time.monotonic()
                if predicate():
                    return True
                time.sleep(0.01)
            return False

        with tempfile.TemporaryDirectory(prefix='nova_hub_bag_') as directory:
            panel = bag_module.RosbagRecorder(node)
            try:
                panel.set_directory(directory)
                panel._name.setText('Drive test')
                panel.set_active(True)
                panel.show()
                tree = panel.findChild(QtWidgets.QTreeWidget, 'recordingTopics')

                def discover():
                    panel._refresh_topics()
                    return tree.findItems(topic, QtCore.Qt.MatchExactly, 0)

                self.assertTrue(wait_for(discover), 'synthetic ROS topic was not discovered')
                tree.findItems(topic, QtCore.Qt.MatchExactly, 0)[0].setCheckState(0, QtCore.Qt.Checked)
                panel.start_recording()
                self.assertTrue(wait_for(lambda: panel._state == 'recording'), panel._message.text())
                self.assertEqual(Path(panel._bag_path).name, 'Drive test')
                start = time.monotonic()
                self.assertTrue(wait_for(lambda: time.monotonic() - start >= 2.0, 5))
                panel.stop_recording()
                self.assertTrue(wait_for(lambda: not panel.is_busy(), 15), panel._message.text())
                self.assertTrue(panel._message.text().startswith('Saved:'), panel._message.text())
                metadata = yaml.safe_load((Path(panel._bag_path) / 'metadata.yaml').read_text())
                information = metadata['rosbag2_bagfile_information']
                self.assertTrue(any(Path(name).name.startswith('Drive test_')
                                    for name in information['relative_file_paths']))
                entries = information['topics_with_message_count']
                self.assertTrue(any(entry['topic_metadata']['name'] == topic and entry['message_count'] > 0
                                    for entry in entries), entries)
            finally:
                if panel.is_busy():
                    panel.stop_recording()
                    wait_for(lambda: not panel.is_busy(), 15)
                panel.set_idle(True)
                panel.close()
                node.destroy_node()
                publisher_node.destroy_node()
                rclpy.shutdown()


class ScreensTest(unittest.TestCase):
    def test_startup_wait_has_a_visible_timeout_for_missing_inputs(self):
        decide = hub_module._startup_wait_result
        self.assertEqual(decide(True, [], 1), (True, []))
        self.assertEqual(decide(True, ['Front'], 29), (False, []))
        self.assertEqual(decide(True, ['Front'], 30), (True, ['Front']))
        self.assertEqual(decide(False, ['LiDAR'], 30), (True, ['3D view', 'LiDAR']))

    def test_startup_waits_for_lidar_and_selected_cameras(self):
        tiles = [Mock(), Mock(), Mock(), Mock()]
        for tile in tiles:
            tile.is_live.return_value = False
        self.assertEqual(hub_module._startup_cameras('front,rear'), ['front', 'rear'])
        self.assertEqual(hub_module._startup_cameras('false'), [])
        self.assertEqual(hub_module._missing_startup_inputs(0, ['front', 'rear'], tiles, 10),
                         ['LiDAR', 'Front', 'Rear'])
        tiles[0].is_live.return_value = tiles[2].is_live.return_value = True
        self.assertEqual(hub_module._missing_startup_inputs(9, ['front', 'rear'], tiles, 10), [])
        self.assertEqual(hub_module._missing_startup_inputs(2, [], tiles, 10), ['LiDAR'])

    def test_early_startup_screen_waits_for_hub_readiness(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory) / 'session.test123'
            session.mkdir()
            env = dict(os.environ, QT_QPA_PLATFORM='offscreen')
            process = subprocess.Popen([sys.executable, str(source.with_name('screens.py')), '--host-splash',
                                        str(session), str(os.getpid())], env=env,
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            try:
                deadline = time.monotonic() + 8
                while not (session / 'visible').exists() and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue((session / 'visible').exists(), 'early splash was not shown')
                (session / 'status').write_text('Waiting for LiDAR, Front')
                time.sleep(0.3)
                self.assertIsNone(process.poll(), 'screen closed before sensors were ready')
                (session / 'ready').touch()
                _, error = process.communicate(timeout=5)
                self.assertEqual(process.returncode, 0, error.decode(errors='replace'))
                self.assertFalse(session.exists())
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate()

    def test_early_startup_screen_acknowledges_operator_skip(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory) / 'session.test456'
            session.mkdir()
            env = dict(os.environ, QT_QPA_PLATFORM='offscreen')
            process = subprocess.Popen([sys.executable, str(source.with_name('screens.py')), '--host-splash',
                                        str(session), str(os.getpid())], env=env,
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            try:
                deadline = time.monotonic() + 8
                while not (session / 'visible').exists() and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue((session / 'visible').exists())
                splash = screens_module.StartupProcess(str(source.with_name('hub.py')), session)
                splash.start(False, QtCore.QSize())
                self.assertTrue(splash.available())
                (session / 'skip').touch()
                self.assertTrue(splash.skipped())
                _, error = process.communicate(timeout=5)
                self.assertEqual(process.returncode, 0, error.decode(errors='replace'))
                self.assertFalse(session.exists())
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate()

    def test_early_startup_screen_closes_when_launcher_exits(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory) / 'session.test789'
            session.mkdir()
            owner = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
            env = dict(os.environ, QT_QPA_PLATFORM='offscreen')
            process = subprocess.Popen([sys.executable, str(source.with_name('screens.py')), '--host-splash',
                                        str(session), str(owner.pid)], env=env,
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            try:
                deadline = time.monotonic() + 8
                while not (session / 'visible').exists() and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue((session / 'visible').exists())
                owner.terminate()
                owner.wait(timeout=3)
                _, error = process.communicate(timeout=5)
                self.assertEqual(process.returncode, 0, error.decode(errors='replace'))
                self.assertFalse(session.exists())
            finally:
                if owner.poll() is None:
                    owner.kill()
                    owner.wait()
                if process.poll() is None:
                    process.kill()
                    process.communicate()

    def test_early_startup_screen_times_out_after_launch_if_hub_hangs(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory) / 'session.test999'
            session.mkdir()
            (session / 'launching').touch()
            observed_error = []
            monitor = QtCore.QTimer()
            monitor.setInterval(50)
            monitor.timeout.connect(lambda: observed_error.append(True) if (session / 'error').exists() else None)
            monitor.start()
            guard = QtCore.QTimer()
            guard.setSingleShot(True)
            guard.timeout.connect(app.quit)
            guard.start(5000)
            try:
                with patch.object(screens_module, 'HOST_START_TIMEOUT_S', 0.2), \
                     patch.object(screens_module, 'HOST_START_ERROR_S', 0.4):
                    result = screens_module.run_host_startup_screen(app, session, os.getpid())
            finally:
                guard.stop()
                monitor.stop()
            self.assertEqual(result, 0)
            self.assertTrue(observed_error, 'startup timeout did not report the failure')
            self.assertFalse(session.exists())

    def test_idle_requires_confirmation_without_running_timers(self):
        screen = screens_module.IdleScreen()
        screen.resize(800, 560)
        screen.show()
        app.processEvents()
        self.assertLess(screen.grab().toImage().pixelColor(1, 1).red(), 10)
        wake = screen.findChild(QtWidgets.QPushButton, 'idleWake')
        cancel = screen.findChild(QtWidgets.QPushButton, 'idleCancel')
        confirm = screen.findChild(QtWidgets.QPushButton, 'idleConfirm')
        calls = []
        screen.wake_callback = lambda: calls.append(True)
        self.assertTrue(wake.isVisible())
        self.assertFalse(confirm.isVisible())
        wake.click()
        self.assertTrue(confirm.isVisible())
        cancel.click()
        self.assertFalse(calls)
        wake.click()
        confirm.click()
        self.assertEqual(calls, [True])
        screen.prepare()
        self.assertTrue(wake.isVisible())
        self.assertFalse(any(timer.isActive() for timer in screen.findChildren(QtCore.QTimer)))
        screen.close()

    def test_startup_art_and_dismiss_animation(self):
        logo = source.parents[1] / 'assets' / 'startup.png'
        screen = screens_module.StartupScreen(logo)
        screen.resize(800, 560)
        screen.show()
        app.processEvents()
        self.assertTrue(screen.has_logo())
        first = screen.grab().toImage()
        screen.set_status('Waiting for LiDAR')
        changed_at = screen._status_changed
        screen.set_status('Waiting for cameras')
        self.assertEqual(screen._status, 'Waiting for cameras')
        self.assertEqual(screen._previous_status, 'STARTING')
        self.assertEqual(screen._status_changed, changed_at)
        end = time.monotonic() + 0.4
        while time.monotonic() < end:
            app.processEvents()
            time.sleep(0.01)
        self.assertNotEqual(first, screen.grab().toImage())
        screen.dismiss()
        end = time.monotonic() + 2
        while screen.isVisible() and time.monotonic() < end:
            app.processEvents()
            time.sleep(0.01)
        self.assertFalse(screen.isVisible())

    def test_startup_helper_waits_for_ready_or_pipe_close(self):
        executable = source.with_name('hub.py')
        env = dict(os.environ, QT_QPA_PLATFORM='offscreen')
        for ready in (True, False):
            process = subprocess.Popen([sys.executable, str(executable), '--startup-splash', '--windowed',
                                        '--preview-size', '800x560'], stdin=subprocess.PIPE,
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
            try:
                readable, _, _ = select.select([process.stdout], [], [], 8)
                self.assertTrue(readable, 'startup helper did not report readiness')
                self.assertEqual(process.stdout.readline(), b'VISIBLE\n')
                self.assertIsNone(process.poll())
                if ready:
                    process.stdin.write(b'READY\n')
                    process.stdin.flush()
                process.stdin.close()
                process.stdin = None
                _, error = process.communicate(timeout=5)
                self.assertEqual(process.returncode, 0, error.decode(errors='replace'))
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate()

    def test_preview_flags_and_saved_theme(self):
        executable = source.with_name('hub.py')
        with tempfile.TemporaryDirectory() as directory:
            env = dict(os.environ, QT_QPA_PLATFORM='offscreen', XDG_CONFIG_HOME=directory)
            for page in ('calibration', 'vehicle', 'recording', 'tools', 'updates', 'network', 'sketch', 'idle'):
                image = Path(directory) / (page + '.png')
                result = subprocess.run([sys.executable, str(executable), '--preview', '--' + page, '--windowed',
                                         '--preview-size', '800x560', '--snapshot', str(image)],
                                        env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=15)
                self.assertEqual(result.returncode, 0, result.stderr.decode(errors='replace'))
                screenshot = QtGui.QImage(str(image))
                self.assertFalse(screenshot.isNull(), page)
                if page == 'idle':
                    self.assertLess(screenshot.pixelColor(1, 1).red(), 10)
            settings_dir = Path(directory) / 'NOVA'
            settings_dir.mkdir(exist_ok=True)
            settings = QtCore.QSettings(str(settings_dir / 'VehicleHub.ini'), QtCore.QSettings.IniFormat)
            for theme in ('black', 'white'):
                settings.setValue('theme', theme)
                settings.sync()
                image = Path(directory) / (theme + '.png')
                result = subprocess.run([sys.executable, str(executable), '--preview', '--windowed',
                                         '--preview-size', '800x560', '--snapshot', str(image)],
                                        env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=15)
                self.assertEqual(result.returncode, 0, result.stderr.decode(errors='replace'))
                screenshot = QtGui.QImage(str(image))
                self.assertFalse(screenshot.isNull())
                self.assertTrue(screenshot.pixelColor(2, 2).red() > 240 if theme == 'white' else
                                screenshot.pixelColor(2, 2).red() < 15)
                self.assertLess(screenshot.pixelColor(2, 70).red(), 15)


class NetworkStatusTest(unittest.TestCase):
    def test_ports_and_bounded_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            port = root / 'sysfs' / 'enp4s0'
            (port / 'device').mkdir(parents=True)
            (port / 'statistics').mkdir()
            for key, value in {'type': '1', 'flags': '0x1', 'carrier': '1', 'operstate': 'up',
                               'speed': '1000', 'address': '00:11:22:33:44:55'}.items():
                (port / key).write_text(value)
            for key in ('rx_errors', 'tx_errors', 'rx_dropped', 'tx_dropped'):
                (port / 'statistics' / key).write_text('0')
            fake = root / 'tcpdump'
            fake.write_text('#!/bin/sh\ni=0\nwhile [ "$i" -lt 1000 ]; do echo "packet $i"; i=$((i+1)); done\n')
            fake.chmod(0o755)
            with patch.dict(os.environ, {'PATH': str(root) + os.pathsep + os.environ['PATH']}):
                panel = network_module.NetworkStatus(False, sysfs_root=root / 'sysfs', enable_external_checks=False)
                panel.show()
                panel.set_active(True)
                app.processEvents()
                tree = panel.findChild(QtWidgets.QTreeWidget, 'networkPorts')
                self.assertEqual(tree.topLevelItemCount(), 1)
                self.assertEqual(tree.topLevelItem(0).text(1), 'Connected')
                self.assertIn('1000 Mb/s', panel.findChild(QtWidgets.QLabel, 'networkPortDetail').text())
                panel.start_capture()
                end = time.monotonic() + 5
                while panel._capture.state() != QtCore.QProcess.NotRunning and time.monotonic() < end:
                    app.processEvents()
                    time.sleep(0.01)
                app.processEvents()
                self.assertEqual(panel.findChild(QtWidgets.QLabel, 'networkCaptureStatus').text(), 'Capture complete.')
                output = panel.findChild(QtWidgets.QPlainTextEdit, 'networkCaptureOutput')
                self.assertLessEqual(output.document().blockCount(), 400)
                (port / 'carrier').write_text('0')
                panel.refresh_ports()
                self.assertEqual(tree.topLevelItem(0).text(1), 'Disconnected')
                # Closing the hub must reap an active capture, even if Qt is
                # about to stop processing its regular shutdown timer.
                fake.write_text('#!/bin/sh\nexec sleep 30\n')
                panel.start_capture()
                self.assertTrue(panel._capture.waitForStarted(2000))
                panel.shutdown()
                self.assertEqual(panel._capture.state(), QtCore.QProcess.NotRunning)
                self.assertFalse(any(timer.isActive() for timer in panel.findChildren(QtCore.QTimer)))
                panel.close()

    def test_weather_uses_bounded_async_responses_and_stops_in_idle(self):
        class Reply(QtNetwork.QNetworkReply):
            def __init__(self, request, data, delay, parent):
                super().__init__(parent)
                self._data, self._offset = data, 0
                self.setRequest(request)
                self.setUrl(request.url())
                self.setAttribute(QtNetwork.QNetworkRequest.HttpStatusCodeAttribute, 200)
                self.open(QtCore.QIODevice.ReadOnly | QtCore.QIODevice.Unbuffered)
                self._timer = QtCore.QTimer(self)
                self._timer.setSingleShot(True)
                self._timer.timeout.connect(self._complete)
                self._timer.start(delay)

            def _complete(self):
                if not self.isFinished():
                    self.readyRead.emit()
                    self.setFinished(True)
                    self.finished.emit()

            def abort(self):
                if not self.isFinished():
                    self.setError(QtNetwork.QNetworkReply.OperationCanceledError, 'Canceled')
                    self.setFinished(True)
                    self.finished.emit()

            def bytesAvailable(self):
                return len(self._data) - self._offset + super().bytesAvailable()

            def readData(self, maximum):
                data = self._data[self._offset:self._offset + maximum]
                self._offset += len(data)
                return data

        class Manager(QtNetwork.QNetworkAccessManager):
            def __init__(self):
                super().__init__()
                self.requests, self.delay = [], 0

            def get(self, request):
                self.requests.append(request.url().toString())
                data = (b'{"success":true,"city":"Richardson","latitude":32.95,"longitude":-96.73}'
                        if request.url().host() == 'ipwho.is' else
                        b'{"current":{"temperature_2m":71.6,"weather_code":2}}')
                return Reply(request, data, self.delay, self)

        manager = Manager()
        panel = network_module.NetworkStatus(False, sysfs_root='/nonexistent', manager=manager, enable_probe=False)
        values = []
        panel.set_weather_callback(lambda text, detail: values.append((text, detail)))
        end = time.monotonic() + 3
        while not any(text == '72°F' for text, _ in values) and time.monotonic() < end:
            app.processEvents()
            time.sleep(0.01)
        self.assertEqual(values[-1][0], '72°F')
        self.assertIn('Richardson', values[-1][1])
        self.assertEqual(len(manager.requests), 2)
        manager.delay = 200
        panel._refresh_weather()
        panel.set_idle(True)
        after_idle = len(values)
        end = time.monotonic() + 0.3
        while time.monotonic() < end:
            app.processEvents()
            time.sleep(0.01)
        self.assertEqual(len(values), after_idle)
        self.assertFalse(any(timer.isActive() for timer in panel.findChildren(QtCore.QTimer)))
        panel.close()


class SketchPadTest(unittest.TestCase):
    def test_drawing_text_history_and_file_round_trip(self):
        pad = sketch_module.SketchPad()
        pad.resize(1000, 700)
        pad.show()
        # The test also opens other top-level windows; give the sketch window
        # focus before simulating the touch that starts text editing.
        pad.activateWindow()
        app.setActiveWindow(pad)
        app.processEvents()
        view = pad.canvas
        self.assertFalse(view.has_unsaved_changes())

        def mouse(kind, point, button, buttons):
            position = view.mapFromScene(QtCore.QPointF(*point))
            event = QtGui.QMouseEvent(kind, QtCore.QPointF(position),
                                      QtCore.QPointF(view.viewport().mapToGlobal(position)),
                                      button, buttons, QtCore.Qt.NoModifier)
            QtWidgets.QApplication.sendEvent(view.viewport(), event)
            app.processEvents()

        mouse(QtCore.QEvent.MouseButtonPress, (250, 200), QtCore.Qt.LeftButton, QtCore.Qt.LeftButton)
        mouse(QtCore.QEvent.MouseMove, (500, 200), QtCore.Qt.NoButton, QtCore.Qt.LeftButton)
        mouse(QtCore.QEvent.MouseButtonRelease, (500, 200), QtCore.Qt.LeftButton, QtCore.Qt.NoButton)
        self.assertEqual(len(view.scene().items()), 1)
        self.assertTrue(view.has_unsaved_changes())
        pad.findChild(QtWidgets.QPushButton, 'sketchUndo').click()
        self.assertFalse(view.scene().items())
        pad.findChild(QtWidgets.QPushButton, 'sketchRedo').click()
        self.assertEqual(len(view.scene().items()), 1)
        pad.findChild(QtWidgets.QPushButton, 'sketchSelect').click()
        stroke = view.scene().items()[0]
        stroke.setSelected(True)
        pad.findChild(QtWidgets.QToolButton, 'sketchColorRed').click()
        pad.findChild(QtWidgets.QSlider, 'sketchSize').setValue(9)
        self.assertEqual(stroke.pen().color(), QtGui.QColor('#c8323e'))
        self.assertEqual(stroke.pen().widthF(), 9)
        pad.findChild(QtWidgets.QPushButton, 'sketchText').click()
        mouse(QtCore.QEvent.MouseButtonPress, (700, 350), QtCore.Qt.LeftButton, QtCore.Qt.LeftButton)
        mouse(QtCore.QEvent.MouseButtonRelease, (700, 350), QtCore.Qt.LeftButton, QtCore.Qt.NoButton)
        keyboard = pad.findChild(QtWidgets.QFrame, 'sketchKeyboard')
        self.assertTrue(keyboard.isVisible())
        key = next(button for button in pad.findChildren(QtWidgets.QPushButton, 'sketchKey') if button.text() == 'h')
        key.click()
        text = next(item for item in view.scene().items() if isinstance(item, QtWidgets.QGraphicsTextItem))
        self.assertEqual(text.toPlainText(), 'h')
        def key(label):
            return next(button for button in pad.findChildren(QtWidgets.QPushButton, 'sketchKey')
                        if button.isVisible() and button.text() == label)
        key('Shift').click()
        key('A').click()
        key('123').click()
        key('1').click()
        self.assertEqual(text.toPlainText(), 'hA1')
        key('Back').click()
        self.assertEqual(text.toPlainText(), 'hA')
        done = next(button for button in pad.findChildren(QtWidgets.QPushButton, 'sketchKey') if button.text() == 'Done')
        done.click()
        app.processEvents()
        self.assertFalse(keyboard.isVisible())
        with tempfile.TemporaryDirectory() as directory:
            project, image = Path(directory) / 'plan.nova-sketch', Path(directory) / 'plan.png'
            view.save(project)
            self.assertFalse(view.has_unsaved_changes())
            view.export_png(image)
            self.assertEqual(QtGui.QImage(str(image)).size(), QtCore.QSize(1600, 900))
            count = len(view.scene().items())
            view.clear()
            self.assertTrue(view.has_unsaved_changes())
            self.assertFalse(view.scene().items())
            view.undo()
            self.assertEqual(len(view.scene().items()), count)
            view.clear()
            view.load(project)
            self.assertEqual(len(view.scene().items()), count)
            self.assertFalse(view.has_unsaved_changes())
            bad = Path(directory) / 'invalid.nova-sketch'
            bad.write_text('{invalid')
            with self.assertRaises(ValueError):
                view.load(bad)
            self.assertEqual(len(view.scene().items()), count)
        view.shutdown()
        pad.close()


class VehicleStatusTest(unittest.TestCase):
    def test_live_stale_and_idle_values(self):
        class Subscription:
            def __init__(self, topic, callback):
                self.topic_name, self.callback = topic, callback

        class Node:
            def __init__(self):
                self.subscriptions = {}

            def create_subscription(self, message, topic, callback, qos):
                subscription = Subscription(topic, callback)
                self.subscriptions[topic] = subscription
                return subscription

            def destroy_subscription(self, subscription):
                self.subscriptions.pop(subscription.topic_name, None)

            def count_publishers(self, topic):
                return 1

        node = Node()
        status = status_module.VehicleStatus(node)
        panel = status.panel
        value = lambda name: panel.findChild(QtWidgets.QLabel, name).text()
        status.set_sensors({'lidar_online': True, 'cameras': {'front': {'sensor_online': False}}}, True)
        self.assertEqual(value('sensor_lidar'), 'Receiving')
        self.assertEqual(value('sensor_front'), 'No recent data')
        node.subscriptions['/guardian/mode'].callback(String(data='MANUAL'))
        node.subscriptions['/vehicle_command'].callback(VehicleCommand(throttle=0.18, steer=-0.06, brake=0.25))
        node.subscriptions['/vehicle_state'].callback(VehicleState(battery_v=0.0))
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        odom.header.stamp.sec = 1
        odom.pose.pose.orientation.w = 1.0
        node.subscriptions['/odometry'].callback(odom)
        odom.header.stamp.nanosec = 100_000_000
        odom.pose.pose.position.x = 0.1
        node.subscriptions['/odometry'].callback(odom)
        diagnostic = DiagnosticStatus(name='Motor controller', message='<test warning>')
        diagnostic.level = bytes([1]) if isinstance(diagnostic.level, bytes) else 1
        node.subscriptions['/diagnostics'].callback(DiagnosticArray(status=[diagnostic]))
        status.refresh()
        self.assertEqual(value('vehicleMode'), 'MANUAL')
        self.assertEqual(value('vehicleSpeed'), '2.2 mph')
        self.assertEqual(value('vehicleBattery'), '—')
        self.assertEqual(value('commandThrottle'), '18 %')
        self.assertEqual(value('commandSteering'), '-6 %')
        self.assertEqual(value('commandBrake'), '25 %')
        self.assertIn('&lt;test warning&gt;', value('diagnosticStatus'))
        node.subscriptions['/vehicle_state'].callback(VehicleState(battery_v=24.6))
        node.subscriptions['/odometry'].callback(odom)
        status.refresh()
        self.assertEqual(value('vehicleBattery'), '24.6 V')
        self.assertEqual(value('vehicleSpeed'), '—')
        old_callback = node.subscriptions['/vehicle_command'].callback
        status.set_idle(True)
        self.assertFalse(node.subscriptions)
        old_callback(VehicleCommand(throttle=0.8))
        status.set_idle(False)
        status.refresh()
        self.assertEqual(value('commandThrottle'), '—')
        self.assertEqual(len(node.subscriptions), 6)
        status.set_idle(True)
        panel.close()


class CalibrationDetailsTest(unittest.TestCase):
    def test_matrices_tf_tree_and_idle(self):
        node = FakeCameraNode()
        details = calibration_module.CalibrationDetails(node)
        details.resize(500, 800)
        details.show()
        app.processEvents()
        self.assertEqual(set(node.subscriptions), {'/tf', '/tf_static'})
        matrix = [[1, 0, 0, 1.5], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        details.set_status({'front': {'transform': {
            'source': 'review', 'cloud_to_lidar': matrix, 'mount_to_base': matrix,
            'quality': {'icp_fitness': 0.8}, 'body_frame': 'zed_front_camera_link'}}})
        self.assertEqual(details.findChild(QtWidgets.QLabel, 'transformSource').text(), 'Review snapshot · not applied')
        self.assertIn('1.5000', details.findChild(QtWidgets.QLabel, 'cloudMatrix').text())
        self.assertIn('0.800', details.findChild(QtWidgets.QLabel, 'calibrationQuality').text())
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'base_link', 'zed_front_camera_link'
        transform.transform.rotation.w = 1.0
        transform.transform.translation.x = 1.5
        node.subscriptions['/tf_static'][1](TFMessage(transforms=[transform]))
        details.refresh()
        graph = details.findChild(calibration_module.TfGraph, 'tfGraph')
        self.assertIn('base_link', graph._nodes)
        self.assertIn('zed_front_camera_link', graph._nodes)
        QtTest.QTest.mouseClick(graph, QtCore.Qt.LeftButton,
                                pos=graph._nodes['zed_front_camera_link'].center().toPoint())
        self.assertEqual(details._selected_tf, 'zed_front_camera_link')
        tree = details.findChild(QtWidgets.QTreeWidget, 'tfTree')
        items = tree.findItems('zed_front_camera_link', QtCore.Qt.MatchRecursive, 0)
        self.assertEqual(len(items), 1)
        tree.setCurrentItem(items[0])
        self.assertIn('static TF', details.findChild(QtWidgets.QLabel, 'tfDirection').text())
        self.assertIn('1.5000', details.findChild(QtWidgets.QLabel, 'tfMatrix').text())
        old_callback = node.subscriptions['/tf_static'][1]
        details.set_idle(True)
        self.assertFalse(node.subscriptions)
        old_callback(TFMessage(transforms=[transform]))
        details.refresh()
        self.assertEqual(tree.topLevelItemCount(), 0)
        details.close()


class HubIntegrationTest(unittest.TestCase):
    @unittest.skipUnless(os.environ.get('NOVA_RVIZ_INTEGRATION') == '1' and os.environ.get('DISPLAY') and
                         os.environ.get('NOVA_RVIZ_QT_MAJOR') == '5', 'Requires Qt 5 RViz and an X display')
    def test_ros_cameras_calibration_swap_and_idle(self):
        with tempfile.TemporaryDirectory() as directory:
            prefix = Path(directory)
            (prefix / 'lib').mkdir()
            (prefix / 'lib' / 'libvehicle_hub_rviz_bridge.so').symlink_to(os.environ['NOVA_RVIZ_LIBRARY'])
            (prefix / 'rviz_qt_major').write_text('5\n', encoding='utf-8')
            with patch.object(module, 'get_package_prefix', return_value=str(prefix)), \
                    patch.object(module, 'get_package_share_directory', return_value=str(prefix)):
                self._exercise_hub(Path(os.environ['NOVA_RVIZ_CONFIG']))

    def _exercise_hub(self, config):
        context = rclpy.context.Context()
        rclpy.init(args=['hub_integration_test'], context=context)
        prefix = '/hub_python_test_' + str(os.getpid())
        cameras = camera_module.TOPICS
        topics = list(cameras) + ['/lidar_perception/hub/status', '/tf', '/tf_static']
        remaps = ['--ros-args']
        for index, topic in enumerate(topics):
            remaps += ['-r', topic + ':=' + prefix + '/t' + str(index)]
        for action in ('review', 'accept'):
            remaps += ['-r', '/lidar_perception/hub/front/' + action + ':=' + prefix + '/' + action]
        node = rclpy.create_node('python_hub_test', cli_args=remaps, context=context)
        peer = rclpy.create_node('python_hub_test_peer', context=context)
        pubs = [peer.create_publisher(Image, prefix + '/t' + str(i), qos_profile_sensor_data) for i in range(5)]
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        status_pub = peer.create_publisher(String, prefix + '/t5', latched)
        sweeps = []
        sweep_sub = peer.create_subscription(Marker, '/vehicle_ui_hub/calibration_sweep', sweeps.append,
                                             qos_profile_sensor_data)
        dynamic_tf = peer.create_publisher(TFMessage, prefix + '/t6', qos_profile_sensor_data)
        static_tf = peer.create_publisher(TFMessage, prefix + '/t7', latched)
        requests = []

        def make_service(action):
            def callback(_request, response):
                requests.append(action)
                response.success, response.message = True, action + ' succeeded'
                return response
            return peer.create_service(Trigger, prefix + '/' + action, callback)
        services = [make_service('review'), make_service('accept')]
        peer_executor = MultiThreadedExecutor(num_threads=2, context=context)
        peer_executor.add_node(peer)
        peer_thread = threading.Thread(target=peer_executor.spin, daemon=True)
        peer_thread.start()
        executor = hub_module.RosExecutor(node)
        native = module.RvizViewport(['hub_integration_test'], config)
        real_network = hub_module.NetworkStatus
        with patch.object(hub_module, 'NetworkStatus',
                          side_effect=lambda preview: real_network(preview, enable_external_checks=False)):
            window = hub_module.HubWindow(native.widget, native, node, False, executor)
        window.resize(1200, 800)
        window.show()
        executor.start()
        message = Image(width=2, height=1, step=6, encoding='rgb8')
        message.data = array('B', [255, 0, 0, 0, 255, 0])
        snapshot = {'active': True, 'lidar_online': True, 'cameras': {
            key: {'state': 'ready', 'sensor_online': True, 'confirmations': 3, 'required': 3}
            for key in ('front', 'left', 'rear', 'right')}}
        last_publish = 0.0

        def wait_for(predicate, timeout=6):
            nonlocal last_publish
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                app.processEvents()
                if time.monotonic() - last_publish > 0.2:
                    for publisher in pubs:
                        publisher.publish(message)
                    status_pub.publish(String(data=json.dumps(snapshot)))
                    last_publish = time.monotonic()
                if predicate():
                    return True
                time.sleep(0.01)
            return False

        def counts(front, others):
            return all(pub.get_subscription_count() == (front if i == 0 else others) for i, pub in enumerate(pubs))

        def drag(handle, target):
            start = handle.mapToGlobal(handle.rect().center())
            end = target.mapToGlobal(target.rect().center())
            for kind, position, button, buttons in (
                    (QtCore.QEvent.MouseButtonPress, start, QtCore.Qt.LeftButton, QtCore.Qt.LeftButton),
                    (QtCore.QEvent.MouseMove, end, QtCore.Qt.NoButton, QtCore.Qt.LeftButton),
                    (QtCore.QEvent.MouseButtonRelease, end, QtCore.Qt.LeftButton, QtCore.Qt.NoButton)):
                event = QtGui.QMouseEvent(kind, QtCore.QPointF(handle.mapFromGlobal(position)),
                                          QtCore.QPointF(position), button, buttons, QtCore.Qt.NoModifier)
                QtWidgets.QApplication.sendEvent(handle, event)
                app.processEvents()

        try:
            self.assertTrue(wait_for(lambda: counts(1, 1) and window.gallery._tiles[0].is_live()))
            self.assertTrue(wait_for(lambda: native.frame_count() > 5))
            geometry = (native.widget.geometry(), window.view_frame.geometry(), window.panel_deck.geometry(),
                        window.clock_header.geometry())
            window._recording_changed('Recording  00:01', True)
            QtTest.QTest.qWait(60)
            self.assertIn('REC', window.recording_header.text())
            self.assertFalse(window.view_frame.findChild(QtWidgets.QFrame, 'recordingBanner').isVisible())
            self.assertEqual((native.widget.geometry(), window.view_frame.geometry(), window.panel_deck.geometry(),
                              window.clock_header.geometry()), geometry)
            # RViz has internal padding: the old implementation colored that
            # whole area. Only the outermost pixel should become red.
            origin = window.view_frame.mapToGlobal(QtCore.QPoint(0, window.view_frame.height() // 2))
            screen = app.primaryScreen().grabWindow(0).toImage()
            self.assertEqual(screen.pixelColor(origin).name(), '#854348')
            self.assertNotEqual(screen.pixelColor(origin + QtCore.QPoint(2, 0)).name(), '#854348')
            frames = native.frame_count()
            self.assertTrue(wait_for(lambda: native.frame_count() > frames))
            window._recording_changed('', False)
            app.processEvents()
            self.assertEqual((native.widget.geometry(), window.view_frame.geometry(), window.panel_deck.geometry(),
                              window.clock_header.geometry()), geometry)
            self.assertTrue(wait_for(lambda: any(marker.action == Marker.ADD for marker in sweeps)))
            self.assertTrue(wait_for(lambda: node.count_subscribers('/vehicle_ui_hub/calibration_sweep') >= 2))
            main = window.findChild(QtWidgets.QFrame, 'mainViewSlot')
            front = window.gallery._tiles[0]
            drag(front.drag_handle, main)
            self.assertIs(main.layout().itemAt(0).widget(), front)
            window.show_page('calibration')
            self.assertTrue(wait_for(lambda: counts(0, 0) and window.camera_buttons['front'].isEnabled()))
            window.camera_buttons['front'].click()
            self.assertTrue(wait_for(lambda: window.reviewing == 'front' and 'review' in requests))
            snapshot['cameras']['front'].update(state='reviewing', review_visible=True)
            self.assertTrue(wait_for(window.accept.isEnabled))
            window.accept.click()
            self.assertTrue(wait_for(lambda: 'accept' in requests and not window.reviewing))
            window.show_page('tools')
            QtTest.QTest.qWait(60)
            stable_size = window.view_frame.size()
            frames = native.frame_count()
            QtTest.QTest.qWait(140)
            self.assertEqual(window.view_frame.size(), stable_size)
            self.assertGreater(native.frame_count(), frames)
            window.show_page('surround')
            self.assertTrue(wait_for(lambda: counts(1, 1) and main.layout().itemAt(0).widget() is front))
            # Keep the native viewport still across page changes and pause it
            # behind Sketch before resuming it on the 3D page.
            window.show_page('sketch')
            self.assertTrue(wait_for(lambda: window.content_pages._transition.state() ==
                                     QtCore.QAbstractAnimation.Stopped))
            QtTest.QTest.qWait(100)
            paused = native.frame_count()
            QtTest.QTest.qWait(200)
            self.assertEqual(native.frame_count(), paused)
            window.show_page('surround')
            self.assertTrue(wait_for(lambda: native.frame_count() > paused and counts(1, 1)))
            self.assertTrue(wait_for(lambda: counts(1, 1) and window.panel_deck.width() == window._panel_width()))
            QtTest.QTest.qWait(80)
            drag(window.view_frame.drag_handle, main)
            self.assertIs(main.layout().itemAt(0).widget(), window.view_frame)
            drag(front.drag_handle, main)
            self.assertIs(main.layout().itemAt(0).widget(), front)
            pool = executor.executor._executor
            window.enter_idle()
            self.assertTrue(wait_for(lambda: counts(0, 0) and dynamic_tf.get_subscription_count() == 0 and
                                     static_tf.get_subscription_count() == 0))
            self.assertTrue(window.idling)
            self.assertIsNone(executor.thread)
            self.assertFalse(any(thread.is_alive() for thread in pool._threads))
            window.wake_from_idle()
            self.assertTrue(wait_for(lambda: counts(1, 1) and not window.idling and executor.thread is not None))
        finally:
            window.cleanup()
            window.close()
            native.close()
            node.destroy_node()
            peer_executor.shutdown()
            peer_thread.join(timeout=3)
            peer.destroy_node()
            rclpy.shutdown(context=context)


class RvizViewportTest(unittest.TestCase):
    def test_qt_mismatch_is_rejected_before_loading_native_libraries(self):
        with tempfile.TemporaryDirectory() as directory:
            share = Path(directory)
            (share / 'rviz_qt_major').write_text('6\n', encoding='utf-8')
            with patch.object(module, 'get_package_share_directory', return_value=str(share)), \
                    patch.object(module.ctypes, 'CDLL') as load:
                with self.assertRaisesRegex(RuntimeError, 'RViz uses Qt 6'):
                    module.RvizViewport(['rviz_bridge_test'])
                load.assert_not_called()

    @unittest.skipUnless(os.environ.get('NOVA_RVIZ_INTEGRATION') == '1' and os.environ.get('DISPLAY'),
                         'Set NOVA_RVIZ_INTEGRATION=1 with an X display to test native RViz')
    def test_native_rviz_in_pyqt(self):
        library = Path(os.environ['NOVA_RVIZ_LIBRARY'])
        config = Path(os.environ['NOVA_RVIZ_CONFIG'])
        major = os.environ['NOVA_RVIZ_QT_MAJOR']
        if major != '5':
            self.skipTest('PyQt5 requires an RViz build using Qt 5')
        with tempfile.TemporaryDirectory() as directory:
            prefix = Path(directory)
            (prefix / 'lib').mkdir()
            (prefix / 'lib' / 'libvehicle_hub_rviz_bridge.so').symlink_to(library)
            (prefix / 'rviz_qt_major').write_text(f'{major}\n', encoding='utf-8')
            with patch.object(module, 'get_package_prefix', return_value=str(prefix)), \
                    patch.object(module, 'get_package_share_directory', return_value=str(prefix)):
                self._exercise_native(config)

    def _exercise_native(self, config):
        rclpy.init(args=['rviz_bridge_test'])
        node = rclpy.create_node('rviz_bridge_test_publisher')
        cloud = node.create_publisher(PointCloud2, '/bridge_test/points', qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        robot = node.create_publisher(String, '/robot_description', latched)
        tf = node.create_publisher(TFMessage, '/tf_static', latched)
        robot.publish(String(data='<robot name="test"><link name="base_link"><visual>'
                                  '<geometry><box size="1 1 1"/></geometry></visual></link></robot>'))
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'base_link', 'test_sensor'
        transform.transform.rotation.w = 1.0
        tf.publish(TFMessage(transforms=[transform]))
        viewport = module.RvizViewport(
            ['rviz_bridge_test', '--ros-args', '-r', '/ouster/points:=/bridge_test/points'], config)
        window = QtWidgets.QMainWindow()
        host = QtWidgets.QWidget()
        row = QtWidgets.QHBoxLayout(host)
        main, side = QtWidgets.QFrame(), QtWidgets.QFrame()
        QtWidgets.QVBoxLayout(main).addWidget(viewport.widget)
        QtWidgets.QVBoxLayout(side)
        row.addWidget(main, 2)
        row.addWidget(side, 1)
        window.setCentralWidget(host)
        window.resize(800, 600)
        window.show()
        header = Header(frame_id='base_link')
        last_publish = 0.0

        def wait_for(predicate, timeout=8):
            nonlocal last_publish
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                app.processEvents()
                rclpy.spin_once(node, timeout_sec=0)
                if time.monotonic() - last_publish > 0.15:
                    header.stamp = node.get_clock().now().to_msg()
                    cloud.publish(point_cloud2.create_cloud_xyz32(header, [[1., 0., 0.], [2., 1., 0.]]))
                    last_publish = time.monotonic()
                if predicate():
                    return True
                time.sleep(0.01)
            return False

        try:
            self.assertTrue(wait_for(lambda: viewport.frame_count() > 5), 'RViz did not render')
            self.assertTrue(wait_for(lambda: node.count_subscribers('/bridge_test/points') >= 1))
            self.assertTrue(wait_for(lambda: node.count_subscribers('/robot_description') >= 1))
            self.assertTrue(wait_for(lambda: node.count_subscribers('/tf_static') >= 1))
            # Moving the QWidget must preserve the same native rendering surface.
            for slot in (side, main):
                previous = viewport.frame_count()
                slot.layout().addWidget(viewport.widget)
                viewport.widget.show()
                self.assertTrue(wait_for(lambda: viewport.frame_count() > previous + 5))
            self.assertTrue(viewport.set_display_enabled('LiDAR', False))
            self.assertTrue(wait_for(lambda: node.count_subscribers('/bridge_test/points') == 0))
            self.assertTrue(viewport.set_display_enabled('LiDAR', True))
            self.assertTrue(wait_for(lambda: node.count_subscribers('/bridge_test/points') >= 1))
            self.assertTrue(viewport.pause())
            self.assertTrue(viewport.pause())
            self.assertTrue(wait_for(lambda: node.count_subscribers('/bridge_test/points') == 0))
            self.assertTrue(wait_for(lambda: node.count_subscribers('/tf_static') == 0))
            self.assertTrue(viewport.resume())
            self.assertTrue(wait_for(lambda: node.count_subscribers('/bridge_test/points') >= 1))
            self.assertTrue(wait_for(lambda: node.count_subscribers('/tf_static') >= 1))
            widget = viewport.widget
            viewport.close()
            self.assertTrue(sip.isdeleted(widget))
        finally:
            viewport.close()
            window.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
