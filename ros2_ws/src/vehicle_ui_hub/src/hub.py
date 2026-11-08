#!/usr/bin/env python3
# Assemble the PyQt vehicle hub and connect its pages to ROS and native RViz.
import json
import math
import os
import re
import signal
import sys
import threading
import time
from pathlib import Path

import rclpy
from PyQt5 import QtCore, QtGui, QtWidgets
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

from bag_recorder import RosbagRecorder
from calibration_details import CalibrationDetails
from camera_views import CameraGallery, RecordingViewport
from developer_tools import SoftwareUpdate
from hub_widgets import (BLACK_STYLE, WHITE_STYLE, CAMERAS, CameraButton, LayerButton, NavigationButton, PreviewScene,
                         SlidingStack, ToolRow, TouchScrollFilter, color, fullscreen_icon, navigation_icon, toggle_icon, white_theme)
from network_status import NetworkStatus
from rviz_viewport import RvizViewport
from screens import IdleScreen, StartupProcess, run_startup_screen
from sketch_pad import SketchPad
from vehicle_status import VehicleStatus


HUB_PREFIX = '/lidar_perception/hub'
PAGE_ORDER = {'surround': 0, 'vehicle': 1, 'sketch': 2, 'tools': 3,
              'calibration': 4, 'recording': 5, 'updates': 6, 'network': 7}


def _label(text, name):
    label = QtWidgets.QLabel(text)
    label.setObjectName(name)
    return label


class RosExecutor:
    # Recreate the executor after idle, so its worker threads do not keep
    # processing callbacks while the hub is left overnight.
    def __init__(self, node):
        self.node = node
        self.executor = self.thread = None
        self.running = threading.Event()

    def start(self):
        if self.thread:
            return
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.node.context)
        self.executor.add_node(self.node)
        self.running.set()

        def spin():
            try:
                while self.running.is_set() and rclpy.ok(context=self.node.context):
                    self.executor.spin_once(timeout_sec=0.2)
            except ExternalShutdownException:
                pass
        self.thread = threading.Thread(target=spin, name='vehicle-hub-ros', daemon=True)
        self.thread.start()

    def stop(self):
        if not self.thread:
            return
        self.running.clear()
        self.executor.wake()
        self.thread.join()
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        # Humble's rclpy shutdown waits for callbacks but leaves its private
        # ThreadPoolExecutor alive. Join that pool too so idle has no workers.
        pool = getattr(self.executor, '_executor', None)
        if pool is not None:
            pool.shutdown(wait=True)
        self.executor, self.thread = None, None


class CalibrationSweep(QtCore.QObject):
    # This is a decorative activity indicator, not a sensor measurement. RViz
    # draws it natively in base_link, so no transparent widget covers its GL view.
    def __init__(self, node, preview_scene, parent=None):
        super().__init__(parent)
        self.node, self.preview_scene = node, preview_scene
        self.publisher = None
        if node:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.publisher = node.create_publisher(Marker, '/vehicle_ui_hub/calibration_sweep', qos)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(33)
        self.timer.timeout.connect(self._publish)

    def set_active(self, active):
        if bool(active) == self.timer.isActive():
            return
        if active:
            self.timer.start()
            self._publish()
        else:
            self.timer.stop()
            if self.preview_scene:
                self.preview_scene.set_scan_angle(None)
            if self.publisher:
                marker = Marker()
                marker.header.frame_id, marker.ns, marker.id, marker.action = 'base_link', 'calibration_sweep', 0, Marker.DELETE
                self.publisher.publish(marker)

    def _publish(self):
        angle = time.monotonic() * 2 * math.pi / 5.0
        if self.preview_scene:
            self.preview_scene.set_scan_angle(angle)
        if not self.publisher:
            return
        marker = Marker()
        marker.header.frame_id, marker.ns, marker.id = 'base_link', 'calibration_sweep', 0
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.type, marker.action = Marker.LINE_LIST, Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035
        marker.color.a = 1.0
        marker.lifetime.nanosec = 400000000
        for index in range(8):
            phase = angle - index * 0.065
            alpha = float(0.48 * (1.0 - index / 8.0))
            marker.points.extend((Point(x=0.0, y=0.0, z=0.05),
                                  Point(x=5.5 * math.cos(phase), y=5.5 * math.sin(phase), z=0.05)))
            marker.colors.extend((ColorRGBA(r=0.32, g=0.74, b=1.0, a=alpha),
                                  ColorRGBA(r=0.32, g=0.74, b=1.0, a=alpha)))
        self.publisher.publish(marker)


class HubWindow(QtWidgets.QMainWindow):
    _status_received = QtCore.pyqtSignal(int, str)

    def __init__(self, viewport, rviz, node, preview=False, ros_executor=None):
        super().__init__()
        self.rviz, self.node, self.preview, self.ros_executor = rviz, node, preview, ros_executor
        self.preview_scene = viewport if preview else None
        self.current_page, self.idling, self.cameras_visible = 'surround', False, True
        self.internet_connected = self.calibration_active = self.stop_pending = False
        self.stop_request_id = self.service_generation = self.pending_services = 0
        self.status_generation, self.status_subscription, self.last_status = 0, None, 0.0
        self.clients, self.service_timers = {}, set()
        self.camera_status, self.camera_buttons, self.layer_buttons = {}, {}, {}
        self.touch_scroll = TouchScrollFilter(self)
        self.reviewing = ''
        self.sweep = CalibrationSweep(node, self.preview_scene, self)
        self.setWindowTitle('1811 Vehicle Hub')
        self.resize(1440, 900)
        self.setMinimumSize(800, 560)
        settings = QtCore.QSettings(QtCore.QSettings.IniFormat, QtCore.QSettings.UserScope, 'NOVA', 'VehicleHub')
        QtWidgets.qApp.setProperty('vehicleHubWhiteTheme', settings.value('theme', 'black') == 'white')
        self.setStyleSheet(self._styles())
        root = QtWidgets.QWidget()
        root.setObjectName('root')
        outer = QtWidgets.QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        header = self._make_header()
        outer.addWidget(header)
        body = QtWidgets.QWidget()
        body_layout = QtWidgets.QHBoxLayout(body)
        body_layout.setContentsMargins(0, 0, 0, 0)
        body_layout.setSpacing(0)
        self.vehicle_status = VehicleStatus(node, preview, self)
        self.vehicle_status.rail.setMinimumWidth(0)
        self.vehicle_status.rail.setMaximumWidth(0)
        self.vehicle_status.rail.hide()
        body_layout.addWidget(self.vehicle_status.rail)
        main = QtWidgets.QWidget()
        main_layout = QtWidgets.QVBoxLayout(main)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        self.view_frame = RecordingViewport(viewport)
        main_view = QtWidgets.QFrame()
        main_view.setObjectName('mainViewSlot')
        main_view_layout = QtWidgets.QVBoxLayout(main_view)
        main_view_layout.setContentsMargins(0, 0, 0, 0)
        main_view_layout.setSpacing(0)
        main_view_layout.addWidget(self.view_frame)
        main_layout.addWidget(main_view, 1)
        self.legend = self._make_legend()
        main_layout.addWidget(self.legend)
        body_layout.addWidget(main, 1)
        self.gallery = CameraGallery(node, preview)
        self.gallery.set_main_view(main_view, self.view_frame, self.legend.setVisible)
        self.camera_area = self._scroll('cameraArea', self.gallery, initially_hidden=False)
        self.camera_toggle_rail = QtWidgets.QFrame()
        self.camera_toggle_rail.setObjectName('cameraToggleRail')
        self.camera_toggle_rail.setFixedWidth(40)
        toggle_layout = QtWidgets.QVBoxLayout(self.camera_toggle_rail)
        toggle_layout.setContentsMargins(0, 0, 0, 0)
        toggle_layout.setSpacing(0)
        toggle_layout.addStretch()
        self.camera_toggle = QtWidgets.QPushButton()
        self.camera_toggle.setObjectName('cameraToggle')
        self.camera_toggle.setFixedSize(40, 64)
        self.camera_toggle.setIconSize(QtCore.QSize(22, 22))
        self.camera_toggle.setCursor(QtCore.Qt.PointingHandCursor)
        self._update_camera_toggle()
        self.camera_toggle.clicked.connect(self._toggle_cameras)
        toggle_layout.addWidget(self.camera_toggle)
        toggle_layout.addStretch()
        body_layout.addWidget(self.camera_toggle_rail)
        # All right-side pages share one slot, so changing tools does not
        # repeatedly resize the live 3D viewport.
        self.panel_deck = SlidingStack()
        self.panel_deck.setObjectName('panelDeck')
        self.panel_deck.addWidget(self.camera_area)
        self.panel_deck.transition_finished.connect(lambda _: self.gallery.clear_inactive_frames())
        body_layout.addWidget(self.panel_deck)
        self.drawer = self._make_calibration_drawer()
        self.drawer_area = self._scroll('drawerArea', self.drawer)
        self.panel_deck.addWidget(self.drawer_area)
        self.status_area = self._scroll('statusArea', self.vehicle_status.panel)
        self.panel_deck.addWidget(self.status_area)
        self.recorder = RosbagRecorder(node, preview)
        self.recording_area = self._scroll('recordingArea', self.recorder)
        self.panel_deck.addWidget(self.recording_area)
        self.tools = self._make_tools_drawer()
        self.tools_area = self._scroll('toolsArea', self.tools)
        self.panel_deck.addWidget(self.tools_area)
        self.updater = SoftwareUpdate(preview)
        self.update_area = self._scroll('updateArea', self.updater)
        self.panel_deck.addWidget(self.update_area)
        self.network_status = NetworkStatus(preview)
        self.network_area = self._scroll('networkArea', self.network_status)
        self.panel_deck.addWidget(self.network_area)
        self.network_status.set_internet_callback(self._internet_changed)
        self.network_status.set_weather_callback(self._weather_changed)
        self.recorder.set_status_callback(self._recording_changed)
        self.vehicle_status.close_button.clicked.connect(lambda: self.show_page('surround'))
        for button in (self.recorder.close_button, self.updater.close_button, self.network_status.close_button):
            button.clicked.connect(lambda: self.show_page('tools'))
        self.content_pages = SlidingStack()
        self.content_pages.addWidget(body)
        self.sketch = SketchPad()
        self.sketch.close_button.clicked.connect(lambda: self.show_page('tools'))
        self.content_pages.addWidget(self.sketch)
        self.content_pages.transition_finished.connect(self._content_transition_finished)
        outer.addWidget(self.content_pages, 1)
        navigation = self._make_navigation()
        outer.addWidget(navigation)
        self.root_pages = QtWidgets.QStackedWidget()
        self.root_pages.addWidget(root)
        self.idle_screen = IdleScreen()
        self.idle_screen.wake_callback = self.wake_from_idle
        self.root_pages.addWidget(self.idle_screen)
        self.setCentralWidget(self.root_pages)
        QtCore.QTimer.singleShot(0, lambda: (self._fade_in(header), self._fade_in(navigation)))
        self.panel_deck.setFixedWidth(self._panel_width())
        self.gallery.set_active(True)
        self._fullscreen_shortcut = QtWidgets.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_F11), self)
        self._fullscreen_shortcut.activated.connect(self.toggle_fullscreen)
        self._escape_shortcut = QtWidgets.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Escape), self)
        self._escape_shortcut.activated.connect(lambda: self.toggle_fullscreen() if self.isFullScreen() else None)
        self._status_received.connect(self._receive_status, QtCore.Qt.QueuedConnection)
        self.watchdog = None
        if node:
            self._set_status_subscription(True)
            self.watchdog = QtCore.QTimer(self)
            self.watchdog.setInterval(1000)
            self.watchdog.timeout.connect(self._update_connection)
            self.watchdog.start()
        else:
            self._populate_preview()

    @staticmethod
    def _styles():
        return BLACK_STYLE + (WHITE_STYLE if white_theme() else '')

    def _scroll(self, name, child, initially_hidden=True):
        area = QtWidgets.QScrollArea()
        area.setObjectName(name)
        area.setMinimumWidth(0)
        if initially_hidden:
            area.setMaximumWidth(0)
        area.setWidgetResizable(True)
        area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        area.setWidget(child)
        self.touch_scroll.add(area)
        if initially_hidden:
            area.hide()
        return area

    def _panel_width(self):
        return max(360, min(560, self.width() * 38 // 100))

    def _rail_width(self):
        return max(180, min(240, self.width() * 16 // 100)) if self.current_page == 'vehicle' and self.width() >= 1200 else 0

    def closeEvent(self, event):
        self.network_status.set_active(False)
        if self.recorder.is_busy():
            event.ignore()
            self.recorder.set_active(False)
            self.recorder.stop_and_then(self.close)
            return
        if self.updater.is_busy():
            event.ignore()
            self.updater.finish_then(self.close)
            return
        super().closeEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, 'date_header'):
            self.date_header.setVisible(self.width() >= 1100)
        if hasattr(self, 'panel_deck'):
            width = self._panel_width()
            if self.current_page != 'sketch':
                visible = self.current_page != 'surround' or self.cameras_visible
                self._set_panel_width(self.panel_deck, width if visible else 0)
                self._set_panel_width(self.vehicle_status.rail, self._rail_width())

    def changeEvent(self, event):
        super().changeEvent(event)
        if hasattr(self, 'fullscreen_button') and event.type() == QtCore.QEvent.WindowStateChange:
            self._update_fullscreen_button()

    def _set_status_subscription(self, active):
        self.status_generation += 1
        generation = self.status_generation
        if self.status_subscription and self.node:
            self.node.destroy_subscription(self.status_subscription)
        self.status_subscription = None
        if self.node and active:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.status_subscription = self.node.create_subscription(
                String, HUB_PREFIX + '/status', lambda msg: self._status_received.emit(generation, msg.data), qos)

    def _update_connection(self):
        if self.last_status and time.monotonic() - self.last_status <= 3:
            return
        self.calibration_active = False
        self._update_sweep()
        self.connection.setText('Calibration offline')
        self.connection.show()
        self.scene_status.setText('LiDAR status unknown')
        self.scene_status.show()
        for button in self.layer_buttons.values():
            button.set_rate(None)
        self.vehicle_status.set_sensors({}, False)
        self.calibration_details.set_status({}, False)
        self.begin.setEnabled(False)
        self.stop.setEnabled(False)
        for button in self.camera_buttons.values():
            button.setEnabled(False)
        self.accept.setEnabled(False)

    def enter_idle(self):
        if self.idling:
            return
        if self.recorder.is_busy():
            QtWidgets.QMessageBox.information(self, 'Idle', 'Stop recording and wait for the bag to finish saving first.')
            return
        if self.updater.is_busy():
            QtWidgets.QMessageBox.information(self, 'Idle', 'Wait for the software update to finish first.')
            return
        if self.pending_services:
            QtWidgets.QMessageBox.information(self, 'Idle', 'Wait for the calibration request to finish first.')
            return
        # Settle moving pages before suspending RViz and its subscriptions.
        self.panel_deck.finish_transition()
        self.content_pages.finish_transition()
        try:
            paused = not self.rviz or self.rviz.pause(self.current_page != 'sketch')
        except RuntimeError as error:
            print(f'[hub] Could not pause RViz: {error}', file=sys.stderr)
            paused = False
        if not paused:
            QtWidgets.QMessageBox.warning(self, 'Idle unavailable', 'Could not pause the 3D view.')
            return
        self.idling = True
        self._update_sweep()
        self.sketch.finish_editing()
        self.gallery.set_active(False)
        self._set_status_subscription(False)
        if self.watchdog:
            self.watchdog.stop()
        self.clock_timer.stop()
        self.last_status = 0.0
        if self.node:
            self._update_connection()
        self.vehicle_status.set_idle(True)
        self.calibration_details.set_idle(True)
        self.recorder.set_idle(True)
        self.network_status.set_idle(True)
        self.service_generation += 1
        self.stop_request_id += 1
        self.stop_pending = False
        for timer in tuple(self.service_timers):
            timer.stop()
            timer.deleteLater()
        self.service_timers.clear()
        if self.node:
            for client in self.clients.values():
                self.node.destroy_client(client)
        self.clients.clear()
        if self.ros_executor:
            self.ros_executor.stop()
        self.idle_screen.prepare()
        self.root_pages.setCurrentWidget(self.idle_screen)

    def wake_from_idle(self):
        if not self.idling:
            return
        try:
            resumed = not self.rviz or self.rviz.resume(self.current_page != 'sketch')
        except RuntimeError as error:
            print(f'[hub] Could not wake RViz: {error}', file=sys.stderr)
            resumed = False
        if not resumed:
            QtWidgets.QMessageBox.warning(self, 'Wake unavailable', 'Could not restore the 3D view. Try again.')
            return
        self.idling = False
        self._update_sweep()
        self.vehicle_status.set_idle(False)
        self.calibration_details.set_idle(False)
        self.recorder.set_idle(False)
        self.recorder.set_active(self.current_page == 'recording')
        self.network_status.set_idle(False)
        self.network_status.set_active(self.current_page == 'network')
        self._set_status_subscription(True)
        self.gallery.set_active(self.current_page == 'surround' and (self.cameras_visible or self.gallery.has_main_camera()),
                                self.cameras_visible)
        if self.ros_executor:
            self.ros_executor.start()
        if self.watchdog:
            self.watchdog.start()
        self._update_clock()
        self.clock_timer.start()
        self.root_pages.setCurrentIndex(0)

    @staticmethod
    def _set_panel_width(widget, target):
        # Resizing the native RViz window on every animation frame can flicker
        # on the vehicle GPU. Resize once; the contents still slide inside.
        widget.setFixedWidth(target)
        widget.setVisible(target > 0)

    @staticmethod
    def _fade_in(widget):
        effect = widget.graphicsEffect()
        if not isinstance(effect, QtWidgets.QGraphicsOpacityEffect):
            effect = QtWidgets.QGraphicsOpacityEffect(widget)
            widget.setGraphicsEffect(effect)
        animation = effect.findChild(QtCore.QPropertyAnimation, 'opacityTransition', QtCore.Qt.FindDirectChildrenOnly)
        if animation is None:
            animation = QtCore.QPropertyAnimation(effect, b'opacity', effect)
            animation.setObjectName('opacityTransition')
            animation.setDuration(200)
            animation.setEasingCurve(QtCore.QEasingCurve.OutCubic)
            animation.finished.connect(lambda: effect.setEnabled(False))
        animation.stop()
        effect.setEnabled(True)
        animation.setStartValue(0.35)
        animation.setEndValue(1.0)
        animation.start()

    def _content_transition_finished(self, index):
        self.gallery.clear_inactive_frames()
        # The native scene stays still during page changes. Stop rendering once
        # the whiteboard covers it; returning to the 3D view resumes it first.
        if self.rviz and index == 1 and self.current_page == 'sketch' and not self.idling:
            try:
                self.rviz.set_rendering(False)
            except RuntimeError as error:
                print(f'[hub] Could not pause RViz after changing pages: {error}', file=sys.stderr)

    def show_page(self, page):
        for name, button in self.nav_buttons.items():
            button.setChecked(name == page or name == 'tools' and page in ('sketch', 'calibration', 'recording', 'updates', 'network'))
        if page == self.current_page:
            return
        leaving_sketch = self.current_page == 'sketch'
        if leaving_sketch:
            self.sketch.finish_editing()
        if self.rviz and leaving_sketch:
            try:
                self.rviz.set_rendering(True)
            except RuntimeError as error:
                print(f'[hub] Could not change RViz rendering: {error}', file=sys.stderr)
                QtWidgets.QMessageBox.warning(self, '3D view unavailable', 'Could not change the 3D view.')
                for name, button in self.nav_buttons.items():
                    button.setChecked(name == self.current_page or name == 'tools' and
                                      self.current_page in ('sketch', 'calibration', 'recording', 'updates', 'network'))
                return
        if page != 'calibration':
            if self.reviewing:
                self.call('release', self.reviewing)
                self.reviewing = ''
            self.accept.setEnabled(False)
            self.review_frame.hide()
        previous_page = self.current_page
        self.current_page = page
        self._update_sweep()
        self.camera_toggle_rail.setVisible(page == 'surround')
        self.gallery.set_surround_visible(page == 'surround')
        self.gallery.set_active(page == 'surround' and (self.cameras_visible or self.gallery.has_main_camera()),
                                self.cameras_visible, preserve_frames=previous_page == 'surround' or page == 'surround')
        self.recorder.set_active(page == 'recording')
        self.network_status.set_active(page == 'network')
        direction = 1 if PAGE_ORDER[page] >= PAGE_ORDER[previous_page] else -1
        if page != 'sketch':
            panels = {'surround': self.camera_area, 'calibration': self.drawer_area, 'vehicle': self.status_area,
                      'recording': self.recording_area, 'tools': self.tools_area, 'updates': self.update_area,
                      'network': self.network_area}
            # Child tools slide in from the right and back to DevTools from the
            # left. Primary pages follow the order of the bottom navigation.
            self.panel_deck.setCurrentWidget(panels[page], direction, animated=not leaving_sketch)
            visible = page != 'surround' or self.cameras_visible
            self._set_panel_width(self.panel_deck, self._panel_width() if visible else 0)
            self._set_panel_width(self.vehicle_status.rail, self._rail_width())
        self.content_pages.setCurrentIndex(1 if page == 'sketch' else 0, direction, animated=False)

    def _make_header(self):
        frame = QtWidgets.QFrame()
        frame.setObjectName('header')
        frame.setFixedHeight(48)
        layout = QtWidgets.QHBoxLayout(frame)
        layout.setContentsMargins(16, 0, 16, 0)
        layout.setSpacing(10)
        self.recording_header = _label('', 'recordingHeader')
        self.recording_header.setAccessibleName('Bag recording status')
        self.recording_header.hide()
        layout.addWidget(self.recording_header, 0, QtCore.Qt.AlignVCenter)
        self.connection = _label('Calibration offline', 'connection')
        self.connection.setVisible(not self.preview)
        layout.addWidget(self.connection, 0, QtCore.Qt.AlignVCenter)
        self.scene_status = _label('LiDAR status unknown', 'sceneStatus')
        self.scene_status.setVisible(not self.preview)
        layout.addWidget(self.scene_status, 0, QtCore.Qt.AlignVCenter)
        layout.addStretch()
        self.date_header = _label('', 'dateHeader')
        self.date_header.setVisible(self.width() >= 1100)
        layout.addWidget(self.date_header)
        self.weather_header = _label('Weather —', 'weatherHeader')
        layout.addWidget(self.weather_header, 0, QtCore.Qt.AlignVCenter)
        self.internet_button = QtWidgets.QPushButton('○  Internet')
        self.internet_button.setObjectName('internetButton')
        self.internet_button.setToolTip('Open network status')
        self.internet_button.setMinimumHeight(40)
        self.internet_button.clicked.connect(lambda: self.show_page('network'))
        layout.addWidget(self.internet_button)
        self.clock_header = _label('', 'clock')
        self._update_clock()
        self.clock_timer = QtCore.QTimer(frame)
        self.clock_timer.setInterval(10000)
        self.clock_timer.timeout.connect(self._update_clock)
        self.clock_timer.start()
        layout.addWidget(self.clock_header)
        return frame

    def _update_clock(self):
        self.clock_header.setText(QtCore.QTime.currentTime().toString('h:mm AP'))
        self.clock_header.setToolTip(QtCore.QDate.currentDate().toString('dddd, MMMM d, yyyy'))
        self.date_header.setText(QtCore.QDate.currentDate().toString('ddd, MMM d'))

    def _make_navigation(self):
        nav = QtWidgets.QFrame()
        nav.setObjectName('navigation')
        nav.setFixedHeight(72)
        layout = QtWidgets.QHBoxLayout(nav)
        layout.setContentsMargins(20, 6, 20, 6)
        layout.setSpacing(6)
        layout.addSpacing(48)
        layout.addStretch(1)
        self.nav_buttons = {}
        for page, text in (('surround', 'Surround'), ('vehicle', 'Vehicle'), ('tools', 'DevTools')):
            button = NavigationButton(text)
            button.setObjectName('navButton')
            button.setIcon(navigation_icon(page))
            button.setIconSize(QtCore.QSize(22, 22))
            button.setFixedWidth(126)
            button.setChecked(page == 'surround')
            button.clicked.connect(lambda checked=False, page=page: self.show_page(page))
            self.nav_buttons[page] = button
            layout.addWidget(button)
        layout.addStretch(1)
        self.fullscreen_button = QtWidgets.QPushButton()
        self.fullscreen_button.setObjectName('fullscreenButton')
        self.fullscreen_button.setFixedSize(48, 48)
        self.fullscreen_button.setIconSize(QtCore.QSize(22, 22))
        self.fullscreen_button.setCursor(QtCore.Qt.PointingHandCursor)
        self._update_fullscreen_button()
        self.fullscreen_button.clicked.connect(self.toggle_fullscreen)
        layout.addWidget(self.fullscreen_button)
        return nav

    def toggle_fullscreen(self):
        self.showNormal() if self.isFullScreen() else self.showFullScreen()

    def _toggle_cameras(self):
        self.cameras_visible = not self.cameras_visible
        self._update_camera_toggle()
        self.gallery.set_active(self.current_page == 'surround' and (self.cameras_visible or self.gallery.has_main_camera()),
                                self.cameras_visible)
        self._set_panel_width(self.panel_deck, self._panel_width() if self.cameras_visible else 0)

    def _update_camera_toggle(self):
        action = 'Hide cameras' if self.cameras_visible else 'Show cameras'
        self.camera_toggle.setIcon(toggle_icon(self.cameras_visible))
        self.camera_toggle.setAccessibleName(action)
        self.camera_toggle.setToolTip(action)

    def _update_fullscreen_button(self):
        action = 'Exit full screen' if self.isFullScreen() else 'Full screen'
        self.fullscreen_button.setIcon(fullscreen_icon(self.isFullScreen()))
        self.fullscreen_button.setAccessibleName(action)
        self.fullscreen_button.setToolTip(action + ' (F11)')

    def _internet_changed(self, state, detail):
        self.internet_connected = state == 'Reachable'
        self.internet_button.setText('●  Internet' if self.internet_connected else '○  Internet ?')
        self.internet_button.setAccessibleName('Internet ' + state + '. Open network status.')
        self.internet_button.setToolTip(detail)
        self._update_internet_color()

    def _update_internet_color(self):
        tint = color('#96d7b2', '#176d45') if self.internet_connected else color('#b0b0b0', '#555555')
        self.internet_button.setStyleSheet('color: ' + tint.name() + ';')

    def _weather_changed(self, state, detail):
        self.weather_header.setText(state)
        self.weather_header.setToolTip(detail)

    def _recording_changed(self, text, recording):
        self.recording_header.setText(('●  ' + text if recording else text) if text else '')
        self.recording_header.setVisible(bool(text))
        self.view_frame.set_recording(recording)

    def set_theme(self, white):
        if white_theme() == white:
            return
        QtWidgets.qApp.setProperty('vehicleHubWhiteTheme', white)
        self.setStyleSheet(self._styles())
        for page, button in self.nav_buttons.items():
            button.setIcon(navigation_icon(page))
        for name, page in (('toolSketch', 'sketch'), ('toolCalibration', 'calibration'), ('toolRecording', 'recording'),
                           ('toolUpdates', 'updates'), ('toolNetwork', 'network'), ('toolIdle', 'idle')):
            button = self.findChild(QtWidgets.QPushButton, name)
            if button:
                button.setIcon(navigation_icon(page))
        self._update_camera_toggle()
        self._update_fullscreen_button()
        self._update_internet_color()
        self.vehicle_status.refresh()
        self.calibration_details.refresh_theme()
        self.recorder.refresh_theme()
        self.network_status.refresh_ports(True)
        self.sketch.apply_theme()
        for tile in self.gallery._tiles:
            tile.update()
        settings = QtCore.QSettings(QtCore.QSettings.IniFormat, QtCore.QSettings.UserScope, 'NOVA', 'VehicleHub')
        settings.setValue('theme', 'white' if white else 'black')
        settings.sync()
        if settings.status() != QtCore.QSettings.NoError:
            print('[hub] Could not save the theme preference.', file=sys.stderr)

    def _make_legend(self):
        panel = QtWidgets.QFrame()
        panel.setObjectName('legend')
        panel.setFixedHeight(60)
        layout = QtWidgets.QHBoxLayout(panel)
        layout.setContentsMargins(6, 3, 6, 3)
        layout.setSpacing(0)
        layout.addStretch()
        for text, tint, display in [('LiDAR', '#d2e0f0', 'LiDAR')] + [
                (item[0].capitalize(), item[2], item[3]) for item in CAMERAS]:
            button = LayerButton(text, tint)
            button.setToolTip('Toggle ' + text + ' point cloud. Rate is measured by registration on the Jetson.')
            button.toggled.connect(lambda enabled, display=display: self._set_layer(display, enabled))
            self.layer_buttons[display] = button
            layout.addWidget(button)
        layout.addStretch()
        return panel

    def _set_layer(self, display, enabled):
        if self.preview_scene:
            self.preview_scene.set_layer_visible(display, enabled)
        if self.rviz:
            try:
                self.rviz.set_display_enabled(display, enabled)
            except RuntimeError as error:
                print(f'[hub] Could not change RViz layer {display}: {error}', file=sys.stderr)

    def _make_tools_drawer(self):
        panel = QtWidgets.QFrame()
        panel.setObjectName('toolsPanel')
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(28, 24, 28, 24)
        layout.setSpacing(0)
        heading = QtWidgets.QHBoxLayout()
        heading.addWidget(_label('DevTools', 'drawerTitle'))
        heading.addStretch()
        close = QtWidgets.QPushButton('×')
        close.setObjectName('closeButton')
        close.setAccessibleName('Close developer tools')
        close.setFixedSize(44, 44)
        close.clicked.connect(lambda: self.show_page('surround'))
        heading.addWidget(close)
        layout.addLayout(heading)
        layout.addSpacing(16)
        appearance = QtWidgets.QHBoxLayout()
        appearance.addWidget(_label('Theme', 'sectionTitle'))
        appearance.addStretch()
        theme_buttons = QtWidgets.QButtonGroup(panel)
        for name, value in (('Black', False), ('White', True)):
            button = QtWidgets.QPushButton(name)
            button.setObjectName('theme' + name)
            button.setAccessibleName(name + ' theme')
            button.setCheckable(True)
            button.setMinimumSize(84, 44)
            button.setCursor(QtCore.Qt.PointingHandCursor)
            button.setChecked(white_theme() == value)
            button.clicked.connect(lambda checked=False, value=value: self.set_theme(value))
            theme_buttons.addButton(button)
            appearance.addWidget(button)
        layout.addLayout(appearance)
        layout.addSpacing(12)
        for title, page, object_name in (('Sketch', 'sketch', 'toolSketch'),
                                         ('Calibration', 'calibration', 'toolCalibration'),
                                         ('Record', 'recording', 'toolRecording'),
                                         ('Updates', 'updates', 'toolUpdates'),
                                         ('Network', 'network', 'toolNetwork'), ('Idle', 'idle', 'toolIdle')):
            button = ToolRow(title, navigation_icon(page))
            button.setObjectName(object_name)
            button.clicked.connect(lambda checked=False, page=page: self.enter_idle() if page == 'idle' else self.show_page(page))
            layout.addWidget(button)
        layout.addStretch()
        return panel

    def _make_calibration_drawer(self):
        panel = QtWidgets.QFrame()
        panel.setObjectName('drawer')
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(28, 24, 28, 24)
        layout.setSpacing(14)
        title = QtWidgets.QHBoxLayout()
        title.addWidget(_label('Calibration', 'drawerTitle'))
        title.addStretch()
        close = QtWidgets.QPushButton('×')
        close.setObjectName('closeButton')
        close.setFixedSize(44, 44)
        close.setAccessibleName('Close calibration')
        close.clicked.connect(lambda: self.show_page('tools'))
        title.addWidget(close)
        layout.addLayout(title)
        description = _label('Keep the vehicle still.\nThis could take a few minutes.', 'bodyText')
        description.setWordWrap(True)
        layout.addWidget(description)
        self.begin = QtWidgets.QPushButton('Start')
        self.begin.setObjectName('primaryButton')
        self.begin.setMinimumHeight(52)
        self.begin.setEnabled(self.preview)
        self.begin.clicked.connect(lambda: self.call('start', ''))
        self.stop = QtWidgets.QPushButton('Stop')
        self.stop.setObjectName('secondaryButton')
        self.stop.setMinimumHeight(52)
        self.stop.setEnabled(False)
        self.stop.clicked.connect(self._stop_calibration)
        controls = QtWidgets.QHBoxLayout()
        controls.setSpacing(10)
        controls.addWidget(self.begin)
        controls.addWidget(self.stop)
        layout.addLayout(controls)
        note = _label('Stopping keeps accepted poses.', 'statusHint')
        note.setWordWrap(True)
        layout.addWidget(note)
        layout.addSpacing(8)
        layout.addWidget(_label('Cameras', 'sectionTitle'))
        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(10)
        for index, (name, _, tint, _) in enumerate(CAMERAS):
            button = CameraButton(name.capitalize(), tint)
            button.setObjectName('camera_' + name)
            button.setEnabled(False)
            button.clicked.connect(lambda checked=False, name=name: self._camera_clicked(name))
            self.camera_buttons[name] = button
            grid.addWidget(button, index // 2, index % 2)
        layout.addLayout(grid)
        self.details = _label('', 'hint')
        self.details.setWordWrap(True)
        self.details.hide()
        layout.addWidget(self.details)
        self.review_frame = QtWidgets.QFrame()
        self.review_frame.setObjectName('reviewFrame')
        review_layout = QtWidgets.QVBoxLayout(self.review_frame)
        review_layout.setContentsMargins(14, 14, 14, 14)
        review_layout.setSpacing(8)
        self.review_title = _label('Review camera', 'reviewTitle')
        self.review_message = _label('Waiting for preview…', 'bodyText')
        self.review_message.setWordWrap(True)
        review_layout.addWidget(self.review_title)
        review_layout.addWidget(self.review_message)
        self.accept = QtWidgets.QPushButton('Accept')
        self.accept.setObjectName('primaryButton')
        self.accept.setMinimumHeight(48)
        self.accept.setEnabled(False)
        self.accept.clicked.connect(lambda: self._review_action('accept'))
        retry = QtWidgets.QPushButton('Decline and retry')
        retry.setObjectName('secondaryButton')
        retry.setMinimumHeight(48)
        retry.clicked.connect(lambda: self._review_action('recompute'))
        cancel = QtWidgets.QPushButton('Close')
        cancel.setObjectName('textButton')
        cancel.clicked.connect(self._close_review)
        review_layout.addWidget(self.accept)
        review_layout.addWidget(retry)
        review_layout.addWidget(cancel)
        self.review_frame.hide()
        layout.addWidget(self.review_frame)
        self.calibration_details = CalibrationDetails(self.node, self.preview)
        layout.addWidget(self.calibration_details)
        self.message = _label('', 'message')
        self.message.setWordWrap(True)
        layout.addWidget(self.message)
        layout.addStretch()
        return panel

    def _stop_calibration(self):
        self.stop_request_id += 1
        request_id = self.stop_request_id
        self.stop_pending = True
        self.begin.setEnabled(False)
        self.stop.setEnabled(False)
        self.accept.setEnabled(False)
        for button in self.camera_buttons.values():
            button.setEnabled(False)

        def finished(ok, text):
            if request_id != self.stop_request_id:
                return
            self.message.setText(text)
            self.stop_pending = False
            if ok:
                self.reviewing = ''
                self.accept.setEnabled(False)
                self.review_frame.hide()
            elif self.preview:
                self._populate_preview()
        self.call('stop', '', finished)

        def timeout():
            if self.stop_pending and request_id == self.stop_request_id:
                self.stop_request_id += 1
                self.stop_pending = False
                self.message.setText('Stop not confirmed. Check the connection.')
        QtCore.QTimer.singleShot(5000, timeout)

    def _camera_clicked(self, name):
        self.calibration_details.select_camera(name)
        if not self.calibration_active or self.stop_pending:
            return
        state = self.camera_status.get(name, {}).get('state', '')
        if state == 'accepted':
            self.call('recompute', name)
            return
        if state not in ('ready', 'reviewing'):
            return
        for key, _, _, display in CAMERAS:
            if key == name:
                self.layer_buttons[display].setChecked(True)
                break
        if self.reviewing and self.reviewing != name:
            self.call('release', self.reviewing)
            self.reviewing = ''
        if state == 'reviewing':
            self.reviewing = name
            self.review_title.setText('Review ' + name)
            self._show_review()
            return

        def finished(ok, _text):
            if ok and not self.stop_pending and self.calibration_active:
                if self.current_page != 'calibration':
                    self.call('release', name)
                    return
                self.reviewing = name
                self.review_title.setText('Review ' + name)
                self.review_message.setText('Waiting for preview. Other cameras keep calibrating.')
                self.accept.setEnabled(False)
                self._show_review()
        self.call('review', name, finished)

    def _show_review(self):
        animate = self.review_frame.isHidden()
        target = self.review_frame.sizeHint().height()
        self.review_frame.show()
        if not animate:
            self.drawer_area.ensureWidgetVisible(self.review_frame, 0, 16)
            return
        animation = self.review_frame.findChild(QtCore.QVariantAnimation, 'reviewTransition', QtCore.Qt.FindDirectChildrenOnly)
        if animation is None:
            animation = QtCore.QVariantAnimation(self.review_frame)
            animation.setObjectName('reviewTransition')
            animation.setDuration(200)
            animation.setEasingCurve(QtCore.QEasingCurve.OutCubic)
            animation.valueChanged.connect(lambda height: self.review_frame.setMaximumHeight(int(height)))

            def finished():
                self.review_frame.setMaximumHeight(16777215)
                if self.review_frame.isVisible():
                    self.drawer_area.ensureWidgetVisible(self.review_frame, 0, 16)
            animation.finished.connect(finished)
        animation.stop()
        self.review_frame.setMaximumHeight(0)
        animation.setStartValue(0)
        animation.setEndValue(target)
        animation.start()

    def _review_action(self, action):
        camera = self.reviewing

        def finished(ok, _text):
            if ok:
                self.reviewing = ''
                self.accept.setEnabled(False)
                self.review_frame.hide()
        self.call(action, camera, finished)

    def _close_review(self):
        self.call('release', self.reviewing)
        self.reviewing = ''
        self.accept.setEnabled(False)
        self.review_frame.hide()

    def call(self, action, camera, done=None):
        if self.idling:
            return
        if self.preview:
            text = 'Calibration is unavailable in preview.'
            self.message.setText(text)
            if done:
                done(False, text)
            return
        if not self.node:
            return
        session_action = action in ('start', 'stop')
        if not session_action and camera not in {item[0] for item in CAMERAS}:
            self.message.setText('Select a camera first.')
            if done:
                done(False, 'No camera selected')
            return
        service = HUB_PREFIX + ('/' + action if session_action else '/' + camera + '/' + action)
        if service not in self.clients:
            self.clients[service] = self.node.create_client(Trigger, service)
        client = self.clients[service]
        generation, request_id = self.service_generation, self.stop_request_id
        self.pending_services += 1
        timer = QtCore.QTimer(self)
        timer.setObjectName('calibrationServicePoll')
        timer.setInterval(100)
        self.service_timers.add(timer)
        started = time.monotonic()
        state = {'complete': False, 'future': None}

        def finish(ok, text):
            if state['complete']:
                return
            state['complete'] = True
            self.pending_services -= 1
            timer.stop()
            self.service_timers.discard(timer)
            timer.deleteLater()
            if action != 'stop':
                self.message.setText(text)
            if done:
                done(ok, text)

        def poll():
            if state['complete']:
                return
            still_needed = generation == self.service_generation and (
                action != 'stop' or self.stop_pending and request_id == self.stop_request_id)
            if not still_needed:
                finish(False, 'Calibration request canceled.')
                return
            elapsed = time.monotonic() - started
            future = state['future']
            if future is not None:
                if future.done():
                    try:
                        result = future.result()
                        finish(bool(result.success), result.message)
                    except Exception as error:
                        finish(False, str(error))
                elif elapsed >= 5:
                    finish(False, 'Calibration request timed out.')
            elif client.service_is_ready():
                self.message.setText('Sending…')
                try:
                    state['future'] = client.call_async(Trigger.Request())
                except Exception as error:
                    finish(False, str(error))
            elif elapsed >= 2:
                finish(False, 'Calibration unavailable. Start the stack with hub:=true.')
        self.message.setText('Connecting…')
        timer.timeout.connect(poll)
        timer.start()
        poll()

    @QtCore.pyqtSlot(int, str)
    def _receive_status(self, generation, data):
        if generation != self.status_generation:
            return
        try:
            status = json.loads(data)
        except (TypeError, ValueError):
            return
        if not isinstance(status, dict):
            return
        self.last_status = time.monotonic()
        self.layer_buttons['LiDAR'].set_rate(status.get('lidar_hz'))
        cameras = status.get('cameras', {})
        cameras = cameras if isinstance(cameras, dict) else {}
        for key, _, _, display in CAMERAS:
            camera = cameras.get(key, {})
            self.layer_buttons[display].set_rate(camera.get('cloud_hz') if isinstance(camera, dict) else None)
        active = status.get('active') is True
        self.calibration_active = active
        self._update_sweep()
        if not active and self.stop_pending:
            self.stop_request_id += 1
            self.stop_pending = False
            self.reviewing = ''
            self.accept.setEnabled(False)
            self.review_frame.hide()
            self.message.setText('Stopped. Accepted poses kept.')
        self.connection.hide()
        self.scene_status.setText('Waiting for LiDAR')
        self.scene_status.setVisible(not self.preview and status.get('lidar_online') is not True)
        self.vehicle_status.set_sensors(status, True)
        self.begin.setEnabled(not active and not self.stop_pending)
        self.stop.setEnabled(active and not self.stop_pending)
        self.begin.setText('Running…' if active else 'Start')
        self.camera_status = {key: value for key, value in cameras.items() if isinstance(value, dict)}
        self.calibration_details.set_status(self.camera_status)
        diagnostics = []
        for name, title, _, _ in CAMERAS:
            details = self.camera_status.get(name, {})
            state, online = details.get('state', 'idle'), details.get('sensor_online') is True
            if state == 'accepted':
                summary = 'Accepted · recalibrate' if active else 'Accepted'
            elif state == 'applying':
                summary = 'Applying'
            elif state == 'reviewing':
                summary = 'Reviewing'
            elif state == 'ready':
                count, required = details.get('confirmations', 0), details.get('required', 0)
                count = count if isinstance(count, int) and not isinstance(count, bool) else 0
                required = required if isinstance(required, int) and not isinstance(required, bool) else 0
                summary = (f'Ready · {count} / {required}' if count > 0 else 'Ready') if online else 'Camera offline'
            elif state == 'searching':
                summary = f"Searching · {details.get('confirmations', 0)} / {details.get('required', 0)}"
            else:
                summary = 'Waiting'
            button = self.camera_buttons[name]
            button.set_summary(summary)
            detail = details.get('detail', '')
            detail = detail if isinstance(detail, str) else ''
            button.setToolTip(detail)
            button.setEnabled(active and not self.stop_pending and
                              (state in ('accepted', 'reviewing') or state == 'ready' and online))
            if state in ('searching', 'applying') and detail:
                diagnostics.append(f'{title} · {detail}')
            if 'update failed' in detail.lower() or detail.startswith('Could not apply') or 'could not save' in detail.lower():
                self.message.setText(f'{title}: {detail}')
        self.details.setText('\n'.join(diagnostics))
        self.details.setVisible(bool(diagnostics))
        if self.reviewing:
            current = self.camera_status.get(self.reviewing, {})
            visible = current.get('review_visible') is True and current.get('sensor_online') is True
            self.accept.setEnabled(not self.stop_pending and visible and current.get('state') == 'reviewing')
            self.review_message.setText('Review this pose in 3D. Other cameras keep calibrating.' if visible else
                                        'Waiting for a live preview.')
        else:
            self.accept.setEnabled(False)

    def _populate_preview(self):
        cameras = {}
        for name, _, _, _ in CAMERAS:
            cameras[name] = {'state': 'searching' if name == 'left' else 'ready', 'sensor_online': True,
                             'confirmations': 1 if name == 'left' else 3, 'required': 3}
        self._receive_status(self.status_generation, json.dumps({'active': True, 'lidar_online': True, 'cameras': cameras}))

    def _update_sweep(self):
        self.sweep.set_active(self.calibration_active and not self.idling and self.current_page != 'sketch')

    def cleanup(self):
        # Stop callbacks before deleting Qt widgets or their ROS node.
        self.panel_deck.finish_transition()
        self.content_pages.finish_transition()
        self.sweep.set_active(False)
        self.gallery.set_active(False)
        self.vehicle_status.set_idle(True)
        self.calibration_details.set_idle(True)
        self.recorder.set_idle(True)
        self.network_status.shutdown()
        self._set_status_subscription(False)
        if self.ros_executor:
            self.ros_executor.stop()
        if self.node:
            for client in self.clients.values():
                self.node.destroy_client(client)
            self.clients.clear()
        self.sketch.canvas.shutdown()


def _preview_size(args):
    if '--preview-size' in args:
        index = args.index('--preview-size')
        if index + 1 < len(args):
            try:
                width, height = (int(value) for value in args[index + 1].split('x'))
                if width > 0 and height > 0:
                    return QtCore.QSize(width, height)
            except ValueError:
                pass
    return QtCore.QSize(1440, 900)


def _startup_cameras(value):
    value = value.strip().lower()
    if value in ('false', 'none', '0'):
        return []
    if value in ('true', 'all', '1', ''):
        return ['front', 'left', 'rear', 'right']
    return list(dict.fromkeys(name.strip() for name in value.split(',') if name.strip()))


def _missing_startup_inputs(lidar_time, expected_cameras, tiles, now):
    missing = [] if lidar_time and now - lidar_time <= 3 else ['LiDAR']
    cameras = {name.lower(): tile for name, tile in zip(('Front', 'Left', 'Rear', 'Right'), tiles)}
    missing.extend(name.capitalize() for name in expected_cameras if name not in cameras or not cameras[name].is_live())
    return missing


def _startup_session():
    token = os.environ.get('NOVA_UI_STARTUP_TOKEN', '')
    if not re.fullmatch(r'session\.[A-Za-z0-9]{6,}', token):
        return None
    path = Path('/vehicle_1811/.nova-ui-startup') / token
    return path if path.is_dir() else None


def main():
    args = list(sys.argv)
    preview = '--preview' in args
    app = QtWidgets.QApplication(args)
    window_size = _preview_size(args)
    windowed = '--windowed' in args
    if '--startup-splash' in args:
        return run_startup_screen(app, windowed, window_size)
    splash = StartupProcess(args[0], _startup_session())
    splash.start(windowed, window_size)
    node = rviz = executor = window = None
    try:
        if preview:
            viewport = PreviewScene()
        else:
            # Python owns control subscriptions; RViz separately owns its native node.
            rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
            node = rclpy.create_node('vehicle_hub_controls')
            rviz = RvizViewport(args)
            viewport = rviz.widget
            executor = RosExecutor(node)
        window = HubWindow(viewport, rviz, node, preview, executor)
        if preview:
            window.resize(window_size)
        window.show() if windowed else window.showFullScreen()
        if preview:
            for flag, page in (('--calibration', 'calibration'), ('--vehicle', 'vehicle'),
                               ('--recording', 'recording'), ('--tools', 'tools'), ('--updates', 'updates'),
                               ('--network', 'network'), ('--sketch', 'sketch'), ('--idle', 'idle')):
                if flag in args:
                    window.enter_idle() if page == 'idle' else window.show_page(page)
                    break
            if '--orbit-preview' in args:
                QtCore.QTimer.singleShot(100, lambda: viewport.rotate_by(QtCore.QPointF(100, 50)))
            if '--snapshot' in args:
                index = args.index('--snapshot')
                if index + 1 < len(args):
                    path = args[index + 1]

                    def snapshot():
                        window.grab().save(path)
                        app.quit()
                    QtCore.QTimer.singleShot(500, snapshot)
        lidar_seen = [0.0]
        startup_subscription = None
        if node:
            # Raw messages avoid deserializing full clouds merely to see that
            # the LiDAR is publishing. This temporary subscription is removed
            # as soon as startup completes or the operator opens the hub.
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            startup_subscription = node.create_subscription(
                PointCloud2, '/ouster/points', lambda _: lidar_seen.__setitem__(0, time.monotonic()), qos, raw=True)
        expected_cameras = _startup_cameras(os.environ.get('NOVA_UI_STARTUP_CAMERAS', 'true'))
        if executor:
            executor.start()
        # ROS launch sends SIGINT/SIGTERM. Closing through Qt lets a recording
        # write its metadata and lets an in-progress software update finish.
        signal.signal(signal.SIGINT, lambda *_: QtCore.QTimer.singleShot(0, window.close))
        signal.signal(signal.SIGTERM, lambda *_: QtCore.QTimer.singleShot(0, window.close))
        initial_frame = rviz.frame_count() if rviz else 0
        if rviz:
            rviz.queue_render()
        startup_check = QtCore.QTimer(window)
        startup_check.setInterval(30)

        def check_startup():
            nonlocal startup_subscription
            painted = viewport.painted if preview else rviz.frame_count() > initial_frame
            if preview:
                missing = []
            else:
                missing = _missing_startup_inputs(lidar_seen[0], expected_cameras, window.gallery._tiles, time.monotonic())
            splash.set_status('Waiting for ' + ', '.join(missing) if missing else 'Finishing startup')
            if not splash.available() or splash.skipped() or painted and not missing:
                startup_check.stop()
                if startup_subscription:
                    node.destroy_subscription(startup_subscription)
                    startup_subscription = None
                splash.finish()
        startup_check.timeout.connect(check_startup)
        startup_check.start()

        return app.exec_()
    finally:
        if window:
            window.cleanup()
        elif executor:
            executor.stop()
        if node:
            node.destroy_node()
        if rviz:
            rviz.close()
        if not preview and rclpy.ok():
            rclpy.shutdown()
        splash.close()


if __name__ == '__main__':
    sys.exit(main())
