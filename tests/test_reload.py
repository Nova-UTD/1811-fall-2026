"""Exercise recovery without touching a vehicle or an existing workspace."""

import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
VEHICLE = ROOT / 'scripts/vehicle.sh'
HEALTH = ROOT / 'ros2_ws/src/obc_bringup/scripts/lidar_healthcheck.sh'


class ReloadWorkspaceTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='1811-reload-test-')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.ws = self.root / 'ros2_ws'
        self.ws.mkdir()
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        scripts = self.root / 'scripts'
        scripts.mkdir()
        shutil.copy2(VEHICLE, scripts / 'vehicle.sh')
        shutil.copy2(ROOT / 'scripts/prepare_reload_cache.py', scripts / 'prepare_reload_cache.py')
        self.vehicle = scripts / 'vehicle.sh'
        for name, content in {
            'src/ouster-ros/ouster-ros/package.xml': '<package/>',
            'src/ouster-ros/ouster-ros/ouster-sdk/CMakeLists.txt': '',
            'src/ouster-ros/ouster-ros/launch/sensor.launch.xml': '<launch/>',
            'src/kiss-icp/ros/package.xml': '<package/>',
            'src/kiss-icp/cpp/kiss_icp/CMakeLists.txt': '',
            'src/kiss-icp/cpp/kiss_icp/3rdparty/sophus/sophus.cmake':
                'https://github.com/strasdat/Sophus/archive/refs/tags/1.24.6.tar.gz',
            'src/kiss-icp/cpp/kiss_icp/3rdparty/tsl_robin/tsl_robin.cmake':
                'https://github.com/Tessil/robin-map/archive/refs/tags/v1.4.0.tar.gz',
            'src/obc_bringup/package.xml': '<package/>',
            'src/obc_bringup/launch/bringup.launch.py': 'pass\n',
            'src/localization/package.xml': '<package/>',
            'src/vehicle_ui_hub/package.xml': '<package/>',
            'src/vehicle_ui_hub/CMakeLists.txt': '',
            'src/vehicle_ui_hub/src/hub.py': 'pass\n',
            'src/vehicle_ui_hub/config/hub.rviz': 'Visualization Manager: {}\n',
            'build/original': 'original build',
            'install/original': 'original install',
            'log/old-log': 'keep this log',
        }.items():
            path = self.ws / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        self.events = self.root / 'events.jsonl'
        self.env = os.environ.copy()
        self.env.update(PATH=f'{self.bin}:{self.env["PATH"]}', RELOAD_TEST_EVENTS=str(self.events))
        self.stub('flock', '#!/bin/sh\nshift\nexec "$@"\n')
        self.stub('pgrep', '#!/bin/sh\n[ "${RELOAD_TEST_RUNNING:-0}" = 1 ]\n')
        self.stub('colcon', '''#!/usr/bin/env python3
import json, os, pathlib, shutil, sys
with open(os.environ['RELOAD_TEST_EVENTS'], 'a') as stream:
    stream.write(json.dumps(sys.argv[1:]) + '\\n')
if sys.argv[1:2] == ['list']:
    if os.environ.get('RELOAD_TEST_LIST_FAIL'):
        sys.exit(8)
    print('ouster_ros\\nouster_sensor_msgs\\nkiss_icp\\nlocalization\\nobc_bringup\\nvehicle_ui_hub')
    sys.exit(0)
if sys.argv[1:2] != ['build']:
    sys.exit(2)
for name in ('build', 'install'):
    pathlib.Path(name).mkdir(exist_ok=True)
    pathlib.Path(name, 'fresh').write_text('new output')
if os.environ.get('RELOAD_TEST_BUILD_FAIL'):
    sys.exit(9)
files = (
    'install/setup.bash',
    'install/ouster_ros/share/ouster_ros/launch/sensor.launch.xml',
    'install/localization/share/localization/launch/localization.launch.py',
    'install/obc_bringup/share/obc_bringup/launch/bringup.launch.py',
    'install/obc_bringup/share/obc_bringup/scripts/lidar_healthcheck.sh',
    'install/vehicle_ui_hub/lib/vehicle_ui_hub/vehicle_ui_hub',
)
for name in files:
    path = pathlib.Path(name)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('# generated\\n')
    if name.endswith('/vehicle_ui_hub'):
        path.chmod(0o755)
if os.environ.get('RELOAD_TEST_BAD_SETUP'):
    pathlib.Path('install/setup.bash').write_text('return 9\\n')
for name in (
    'install/ouster_ros/lib/ouster_ros/os_driver',
    'install/ouster_ros/lib/libos_driver_component.so',
    'install/kiss_icp/lib/kiss_icp/kiss_icp_node',
    'install/kiss_icp/lib/libodometry_component.so',
    'install/vehicle_ui_hub/lib/libvehicle_hub_rviz_bridge.so',
):
    path = pathlib.Path(name)
    path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile('/bin/true', path)
    path.chmod(0o755)
    if os.environ.get('RELOAD_TEST_CORRUPT') and name.endswith('/os_driver'):
        path.write_bytes(b'corrupted')
''')

    def stub(self, name, content):
        path = self.bin / name
        path.write_text(content)
        path.chmod(0o755)

    def run_reload(self, **overrides):
        env = self.env.copy()
        env.update(overrides)
        return subprocess.run(['bash', str(self.vehicle), 'reload-workspace'], cwd=self.ws,
                              env=env, text=True, capture_output=True, timeout=20)

    def backups(self):
        return list((self.ws / '.reload-backups').glob('reload.*'))

    def test_clean_rebuild_preserves_previous_outputs_and_source(self):
        result = self.run_reload()
        self.assertEqual(result.returncode, 0, result.stderr)
        backup, = self.backups()
        self.assertEqual((backup / 'build/original').read_text(), 'original build')
        self.assertEqual((backup / 'install/original').read_text(), 'original install')
        self.assertEqual((self.ws / 'build/fresh').read_text(), 'new output')
        self.assertEqual((self.ws / 'log/old-log').read_text(), 'keep this log')
        self.assertEqual((self.ws / 'src/vehicle_ui_hub/src/hub.py').read_text(), 'pass\n')
        self.assertTrue((self.ws / '.reload-backups/COLCON_IGNORE').exists())
        events = [json.loads(line) for line in self.events.read_text().splitlines()]
        self.assertEqual(events[0][0], 'list')
        self.assertIn('--parallel-workers', next(event for event in events if event[0] == 'build'))

    def test_failed_build_restores_outputs_and_preserves_partial_results(self):
        result = self.run_reload(RELOAD_TEST_BUILD_FAIL='1')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual((self.ws / 'build/original').read_text(), 'original build')
        self.assertEqual((self.ws / 'install/original').read_text(), 'original install')
        backup, = self.backups()
        self.assertEqual((backup / 'failed-build/fresh').read_text(), 'new output')
        self.assertEqual((backup / 'failed-install/fresh').read_text(), 'new output')
        self.assertIn('Previous outputs restored', result.stderr)

    def test_corrupt_rebuilt_driver_is_rejected_and_originals_restored(self):
        result = self.run_reload(RELOAD_TEST_CORRUPT='1')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('executable or library is damaged', result.stderr)
        self.assertTrue((self.ws / 'build/original').exists())
        self.assertTrue((self.ws / 'install/original').exists())

    def test_missing_runtime_library_is_rejected_and_originals_restored(self):
        self.stub('ldd', '#!/bin/sh\nprintf "libbroken.so => not found\\n"\n')
        result = self.run_reload()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('missing or invalid shared libraries', result.stderr)
        self.assertTrue((self.ws / 'build/original').exists())
        self.assertTrue((self.ws / 'install/original').exists())

    def test_setup_runtime_failure_restores_previous_outputs(self):
        result = self.run_reload(RELOAD_TEST_BAD_SETUP='1')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Rebuilt ROS setup failed when sourced', result.stderr)
        self.assertTrue((self.ws / 'build/original').exists())
        self.assertTrue((self.ws / 'install/original').exists())

    def test_running_stack_or_missing_source_does_not_move_outputs(self):
        result = self.run_reload(RELOAD_TEST_RUNNING='1')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Stop the running', result.stderr)
        self.assertTrue((self.ws / 'build/original').exists())
        self.assertFalse((self.ws / '.reload-backups').exists())
        (self.ws / 'src/kiss-icp/ros/package.xml').unlink()
        result = self.run_reload()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Required source is missing', result.stderr)
        self.assertTrue((self.ws / 'install/original').exists())
        self.assertFalse((self.ws / '.reload-backups').exists())

    def test_discovery_failure_and_linked_build_leave_outputs_untouched(self):
        result = self.run_reload(RELOAD_TEST_LIST_FAIL='1')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Package discovery failed', result.stderr)
        self.assertFalse((self.ws / '.reload-backups').exists())
        original = self.ws / 'build'
        original.rename(self.ws / 'outside-build')
        original.symlink_to(self.ws / 'outside-build', target_is_directory=True)
        result = self.run_reload()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Refusing to move unexpected path build', result.stderr)
        self.assertEqual((self.ws / 'outside-build/original').read_text(), 'original build')

    def test_low_disk_space_stops_before_moving_outputs(self):
        self.stub('df', '#!/bin/sh\nprintf "Filesystem 1024-blocks Used Available Capacity Mounted\\n/dev/test 100 99 1 99%% /\\n"\n')
        result = self.run_reload()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Not enough free disk space', result.stderr)
        self.assertTrue((self.ws / 'build/original').exists())
        self.assertTrue((self.ws / 'install/original').exists())
        self.assertFalse((self.ws / '.reload-backups').exists())

    def test_backup_move_failure_restores_the_first_directory(self):
        self.stub('mv', '''#!/bin/sh
if [ "$2" = install ]; then exit 9; fi
exec /usr/bin/mv "$@"
''')
        result = self.run_reload()
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual((self.ws / 'build/original').read_text(), 'original build')
        self.assertEqual((self.ws / 'install/original').read_text(), 'original install')


class ReloadCommandTest(unittest.TestCase):
    def test_reload_forwards_options_and_requests_recovery(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            shutil.copy2(ROOT / 'reload', root / 'reload')
            (root / 'g').write_text('#!/bin/sh\nprintf "%s\\n" "$NOVA_1811_RELOAD_BUILD" "$@"\n')
            (root / 'g').chmod(0o755)
            (root / 'pgrep').write_text('#!/bin/sh\nexit 1\n')
            (root / 'pgrep').chmod(0o755)
            env = os.environ.copy()
            env['PATH'] = f'{root}:{env["PATH"]}'
            literal = 'lidar_ip:=literal $(touch should-not-exist)'
            result = subprocess.run([str(root / 'reload'), literal], cwd=root,
                                    env=env, text=True, capture_output=True, check=True)
            self.assertEqual(result.stdout.splitlines(), ['1', literal])
            self.assertFalse((root / 'should-not-exist').exists())

    def test_reload_refuses_to_start_when_vehicle_stack_is_running(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            shutil.copy2(ROOT / 'reload', root / 'reload')
            (root / 'g').write_text('#!/bin/sh\ntouch launched\n')
            (root / 'g').chmod(0o755)
            (root / 'pgrep').write_text('#!/bin/sh\nexit 0\n')
            (root / 'pgrep').chmod(0o755)
            env = os.environ.copy()
            env['PATH'] = f'{root}:{env["PATH"]}'
            result = subprocess.run([str(root / 'reload')], cwd=root, env=env,
                                    text=True, capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('Stop the running', result.stderr)
            self.assertFalse((root / 'launched').exists())


class DependencyCacheTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='1811-cache-test-')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / 'source'
        self.previous = self.root / 'previous'
        self.fresh = self.root / 'fresh'
        for name, url in (
                ('sophus/sophus.cmake', 'https://github.com/strasdat/Sophus/archive/refs/tags/1.24.6.tar.gz'),
                ('tsl_robin/tsl_robin.cmake', 'https://github.com/Tessil/robin-map/archive/refs/tags/v1.4.0.tar.gz')):
            path = self.source / 'cpp/kiss_icp/3rdparty' / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(url)
        self.fresh.mkdir()

    def archive(self, name, folder, filename, member='CMakeLists.txt'):
        path = self.previous / 'kiss_icp/_deps' / f'{name}-subbuild' / f'{name}-populate-prefix/src' / filename
        path.parent.mkdir(parents=True, exist_ok=True)
        data = b'project(test)\n'
        with tarfile.open(path, 'w:gz') as archive:
            entry = tarfile.TarInfo(f'{folder}/{member}')
            entry.size = len(data)
            entry.mode = 0o644
            archive.addfile(entry, io.BytesIO(data))
        return path

    def prepare(self, previous=None, fresh=None):
        return subprocess.run([sys.executable, str(ROOT / 'scripts/prepare_reload_cache.py'),
                               str(self.source), str(previous or self.previous), str(fresh or self.fresh)],
                              text=True, capture_output=True, check=True)

    def test_intact_cached_archives_support_repeated_offline_rebuilds(self):
        self.archive('sophus', 'Sophus-1.24.6', '1.24.6.tar.gz')
        self.archive('tessil', 'robin-map-1.4.0', 'v1.4.0.tar.gz')
        result = self.prepare()
        meta = Path(result.stdout.strip())
        args = json.loads(meta.read_text())['names']['kiss_icp']['cmake-args']
        self.assertTrue(any(arg.startswith('-DFETCHCONTENT_SOURCE_DIR_SOPHUS=') for arg in args))
        self.assertTrue(any(arg.startswith('-DFETCHCONTENT_SOURCE_DIR_TESSIL=') for arg in args))
        self.assertEqual((self.fresh / '.reload-dependencies/sophus/CMakeLists.txt').read_bytes(), b'project(test)\n')
        second = self.root / 'second'
        second.mkdir()
        result = self.prepare(previous=self.fresh, fresh=second)
        self.assertTrue(Path(result.stdout.strip()).is_file())
        self.assertTrue((second / '.reload-dependencies/archives/tessil-v1.4.0.tar.gz').is_file())

    def test_corrupt_or_unsafe_archives_are_never_extracted(self):
        damaged = self.archive('sophus', 'Sophus-1.24.6', '1.24.6.tar.gz')
        damaged.write_bytes(b'broken archive')
        self.archive('tessil', 'robin-map-1.4.0', 'v1.4.0.tar.gz', member='../../escape')
        result = self.prepare()
        self.assertEqual(result.stdout, '')
        self.assertIn('No intact sophus archive', result.stderr)
        self.assertIn('No intact tessil archive', result.stderr)
        self.assertFalse((self.root / 'escape').exists())
        self.assertFalse((self.fresh / '.reload-dependencies/sophus').exists())
        self.assertFalse((self.fresh / '.reload-dependencies/tessil').exists())

    def test_a_damaged_cache_falls_back_to_an_older_preserved_copy(self):
        backups = self.root / '.reload-backups'
        current = backups / 'reload.current' / 'build'
        older = backups / 'reload.older' / 'build'
        self.previous = current
        damaged = self.archive('sophus', 'Sophus-1.24.6', '1.24.6.tar.gz')
        damaged.write_bytes(b'broken archive')
        self.previous = older
        self.archive('sophus', 'Sophus-1.24.6', '1.24.6.tar.gz')
        result = self.prepare(previous=current)
        self.assertTrue(Path(result.stdout.strip()).is_file())
        self.assertIn('trying another copy', result.stderr)
        self.assertTrue((self.fresh / '.reload-dependencies/sophus/CMakeLists.txt').is_file())


class LidarHealthCheckTest(unittest.TestCase):
    def run_check(self, timeout_status):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            timeout = root / 'timeout'
            timeout.write_text('#!/bin/sh\nif [ "$RELOAD_TEST_TIMEOUT" != 0 ]; then exit "$RELOAD_TEST_TIMEOUT"; fi\nshift\nexec "$@"\n')
            timeout.chmod(0o755)
            ros2 = root / 'ros2'
            ros2.write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n')
            ros2.chmod(0o755)
            env = os.environ.copy()
            env.update(PATH=f'{root}:{env["PATH"]}', RELOAD_TEST_TIMEOUT=str(timeout_status))
            return subprocess.run(['bash', str(HEALTH)], env=env, text=True, capture_output=True, timeout=5)

    def test_real_cloud_check_uses_best_effort(self):
        result = self.run_check(0)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('best_effort', result.stdout)
        self.assertIn('/ouster/points', result.stdout)
        self.assertIn('sensor_msgs/msg/PointCloud2', result.stdout)
        self.assertIn('LiDAR cloud received', result.stdout)

    def test_timeout_and_ros_failure_do_not_report_lidar_ready(self):
        for status in (124, 2):
            with self.subTest(status=status):
                result = self.run_check(status)
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn('LiDAR cloud received', result.stdout)
                self.assertIn('No LiDAR cloud' if status == 124 else 'sensor status is unknown', result.stderr)


if __name__ == '__main__':
    unittest.main()
