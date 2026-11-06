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
