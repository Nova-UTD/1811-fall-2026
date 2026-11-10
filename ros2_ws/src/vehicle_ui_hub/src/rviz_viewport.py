# Wrap the native RViz widget for a PyQt5 hub without copying rendered frames.
import ctypes
import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from PyQt5 import QtCore, QtWidgets, sip


class RvizViewport:
    # Create and use the viewport on the Qt GUI thread. The caller owns the
    # surrounding layouts and must call close() before QApplication is deleted.
    def __init__(self, ros_args=None, config_path=None):
        app = QtWidgets.QApplication.instance()
        if app is None or QtCore.QThread.currentThread() != app.thread():
            raise RuntimeError('Create RViz on the QApplication thread')

        share = Path(get_package_share_directory('vehicle_ui_hub'))
        qt_major = (share / 'rviz_qt_major').read_text(encoding='utf-8').strip()
        if qt_major != str(QtCore.QT_VERSION >> 16):
            raise RuntimeError(
                f'RViz uses Qt {qt_major}, but PyQt5 uses Qt {QtCore.QT_VERSION >> 16}. '
                'Build the hub with a matching ROS and Qt installation.'
            )
        library_path = Path(get_package_prefix('vehicle_ui_hub')) / 'lib' / 'libvehicle_hub_rviz_bridge.so'
        self._lib = ctypes.CDLL(os.fspath(library_path))
        self._configure_functions()
        args = list(sys.argv if ros_args is None else ros_args)
        if not args:
            args = ['vehicle_ui_hub']
        encoded_args = [os.fsencode(arg) for arg in args]
        argv = (ctypes.c_char_p * len(encoded_args))(*encoded_args)
        config = os.fsencode(config_path or share / 'config' / 'hub.rviz')
        self._handle = self._lib.nova_rviz_create(config, len(encoded_args), argv)
        if not self._handle:
            error = self._lib.nova_rviz_error().decode('utf-8', errors='replace')
            raise RuntimeError(f'Could not create RViz: {error}')
        address = self._lib.nova_rviz_widget(self._handle)
        if not address:
            self.close()
            raise RuntimeError('RViz did not create a viewport')
        # wrapinstance shares RViz's QWidget directly; no screenshot or video
        # copy sits between OGRE and the display.
        self.widget = sip.wrapinstance(address, QtWidgets.QWidget)

    def _configure_functions(self):
        lib = self._lib
        lib.nova_rviz_error.restype = ctypes.c_char_p
        lib.nova_rviz_create.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.POINTER(ctypes.c_char_p)]
        lib.nova_rviz_create.restype = ctypes.c_void_p
        lib.nova_rviz_widget.argtypes = [ctypes.c_void_p]
        lib.nova_rviz_widget.restype = ctypes.c_void_p
        lib.nova_rviz_set_rendering.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.nova_rviz_set_rendering.restype = ctypes.c_int
        lib.nova_rviz_queue_render.argtypes = [ctypes.c_void_p]
        lib.nova_rviz_queue_render.restype = ctypes.c_int
        lib.nova_rviz_frame_count.argtypes = [ctypes.c_void_p]
        lib.nova_rviz_frame_count.restype = ctypes.c_uint64
        lib.nova_rviz_set_display.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
        lib.nova_rviz_set_display.restype = ctypes.c_int
        lib.nova_rviz_pause.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.nova_rviz_pause.restype = ctypes.c_int
        lib.nova_rviz_resume.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.nova_rviz_resume.restype = ctypes.c_int
        lib.nova_rviz_destroy.argtypes = [ctypes.c_void_p]
        lib.nova_rviz_destroy.restype = None

    def _require_open(self):
        app = QtWidgets.QApplication.instance()
        if app is None or QtCore.QThread.currentThread() != app.thread():
            raise RuntimeError('Use RViz on the QApplication thread')
        if not self._handle or sip.isdeleted(self.widget):
            raise RuntimeError('The RViz viewport is closed')

    def set_rendering(self, enabled):
        self._require_open()
        self._check(self._lib.nova_rviz_set_rendering(self._handle, bool(enabled)))

    def queue_render(self):
        self._require_open()
        self._check(self._lib.nova_rviz_queue_render(self._handle))

    def frame_count(self):
        self._require_open()
        return self._lib.nova_rviz_frame_count(self._handle)

    def set_display_enabled(self, name, enabled):
        self._require_open()
        return self._check(self._lib.nova_rviz_set_display(self._handle, name.encode('utf-8'), bool(enabled)))

    def pause(self, restore_render_on_failure=True):
        self._require_open()
        return self._check(self._lib.nova_rviz_pause(self._handle, bool(restore_render_on_failure)))

    def resume(self, start_render=True):
        self._require_open()
        return self._check(self._lib.nova_rviz_resume(self._handle, bool(start_render)))

    def _check(self, result):
        if result < 0:
            error = self._lib.nova_rviz_error().decode('utf-8', errors='replace')
            raise RuntimeError(f'RViz operation failed: {error}')
        return bool(result)

    def close(self):
        if not self._handle:
            return
        app = QtWidgets.QApplication.instance()
        if app is None or QtCore.QThread.currentThread() != app.thread():
            raise RuntimeError('Close RViz on the QApplication thread')
        handle, self._handle = self._handle, None
        self._lib.nova_rviz_destroy(handle)
