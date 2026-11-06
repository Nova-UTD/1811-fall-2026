"""Check that the ZED floor wrapper is built in an isolated, reusable overlay."""

import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('floor_overlay', ROOT / 'scripts/prepare_zed_floor_overlay.py')
floor_overlay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(floor_overlay)


class FloorOverlayTest(unittest.TestCase):
    def test_release_patch_matches_real_wrapper_context(self):
        # Context from Stereolabs zed-ros2-wrapper v5.2.2, commit b47c727.
        # Keep its blank lines: they are significant to git apply.
        snippets = {
            'include/zed_camera_component.hpp': (
                '  objPub mPubBodyTrk;\n'
                '  depthInfoPub mPubDepthInfo;\n'
                '  planePub mPubPlane;\n'
                '  markerPub mPubMarker;\n'
                '\n'
                '  geoPosePub mPubGeoPose;\n'
            ),
            'src/zed_camera_component_main.cpp': (
                '  }\n'
                '\n'
                '  initVideoDepthPublishers();\n'
                '\n'
                '  // POS. TRACKING and GNSS FUSION parameters\n'
                '#if (ZED_SDK_MAJOR_VERSION * 10 + ZED_SDK_MINOR_VERSION) >= 52\n'
            ),
            'src/zed_camera_component_video_depth.cpp': (
                '#include "zed_camera_component.hpp"\n'
                '#include "sl_logging.hpp"\n'
                '#include "sl_tools.hpp"\n'
                '\n'
                '#include <sensor_msgs/distortion_models.hpp>\n'
                '#include <sensor_msgs/image_encodings.hpp>\n'
                '#include <sensor_msgs/msg/point_field.hpp>\n'
                '\n'
                'void ZedCamera::processPointCloud()\n'
                '{\n'
                '      DEBUG_STREAM_PC(\n'
                '        " * [processPointCloud] Retrieved point cloud size: " << mMatCloud.getWidth() << "x" <<\n'
                '          mMatCloud.getHeight());\n'
                '\n'
                '      // Signal Pointcloud thread that a new pointcloud is ready\n'
                '      mPcDataReadyCondVar.notify_one();\n'
                '      mPcDataReady = true;\n'
                '}\n'
            ),
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name, contents in snippets.items():
                target = root / 'zed_components/src/zed_camera' / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text(contents)
            result = subprocess.run(['git', 'apply', '--check', str(floor_overlay.PATCH)],
                                    cwd=root, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_patch_builds_a_copy_and_reuses_it_until_source_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            zed_ws = root / 'zed_ws'
            source = zed_ws / 'src/zed-ros2-wrapper/zed_components'
            source.mkdir(parents=True)
            (source / 'package.xml').write_text('<package><version>5.2.2</version></package>')
            (source / 'sample.cpp').write_text('before\n')
            patch_file = root / 'change.patch'
            patch_file.write_text('--- a/zed_components/sample.cpp\n'
                                  '+++ b/zed_components/sample.cpp\n'
                                  '@@ -1 +1 @@\n-before\n+after\n')
            builds = []

            def run(command, cwd):
                if command[0] == 'git':
                    subprocess.run(command, cwd=cwd, check=True, capture_output=True)
                else:
                    builds.append(command)
                    install = Path(command[command.index('--install-base') + 1])
                    (install / 'zed_components/lib').mkdir(parents=True)
                    (install / 'local_setup.bash').write_text('')
                    (install / 'zed_components/lib/libzed_camera_component.so').write_text('fake')

            with patch.object(floor_overlay, 'PATCH', patch_file), patch.object(floor_overlay, 'run', run):
                first = floor_overlay.prepare(zed_ws, root / 'cache')
                second = floor_overlay.prepare(zed_ws, root / 'cache')
                self.assertEqual(first, second)
                self.assertEqual(len(builds), 1)
                self.assertEqual((source / 'sample.cpp').read_text(), 'before\n')
                self.assertEqual((first.parents[1] / 'src/zed_components/sample.cpp').read_text(), 'after\n')
                (source / 'other.cpp').write_text('user change\n')
                third = floor_overlay.prepare(zed_ws, root / 'cache')
                self.assertNotEqual(first, third)
                self.assertEqual(len(builds), 2)
                self.assertEqual((source / 'other.cpp').read_text(), 'user change\n')

    def test_incompatible_patch_or_version_does_not_modify_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            zed_ws = root / 'zed_ws'
            source = zed_ws / 'src/zed_components'
            source.mkdir(parents=True)
            (source / 'package.xml').write_text('<package><version>5.2.2</version></package>')
            target = source / 'sample.cpp'
            target.write_text('user content\n')
            patch_file = root / 'change.patch'
            patch_file.write_text('--- a/zed_components/sample.cpp\n'
                                  '+++ b/zed_components/sample.cpp\n'
                                  '@@ -1 +1 @@\n-missing\n+after\n')
            with patch.object(floor_overlay, 'PATCH', patch_file):
                with self.assertRaises(subprocess.CalledProcessError):
                    floor_overlay.prepare(zed_ws, root / 'cache')
                self.assertEqual(target.read_text(), 'user content\n')
                (source / 'package.xml').write_text('<package><version>5.3.0</version></package>')
                with self.assertRaisesRegex(RuntimeError, 'requires zed_components 5.2.2'):
                    floor_overlay.prepare(zed_ws, root / 'cache')


if __name__ == '__main__':
    unittest.main()
