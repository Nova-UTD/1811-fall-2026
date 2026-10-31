"""Exercise the vehicle shortcut without Docker, ROS, or vehicle hardware."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


REPO = Path(__file__).resolve().parents[1]


class GStartupTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / "scripts").mkdir()
        (self.root / "bin").mkdir()
        for name in ("g", "scripts/dev.sh", "scripts/full_stack.sh"):
            shutil.copy2(REPO / name, self.root / name)

        self.events = self.root / "events.jsonl"
        self.env = os.environ.copy()
        self.env.update({
            "PATH": f"{self.root / 'bin'}:{self.env['PATH']}",
            "DISPLAY": ":test",
            "NOVA_TEST_ROOT": str(self.root),
            "NOVA_TEST_EVENTS": str(self.events),
        })
        # dev.sh normally prepares ~/.ssh; never touch the real home in tests.
        self.stub("mkdir", "#!/bin/sh\nexit 0\n")
        self.stub("chmod", "#!/bin/sh\nexit 0\n")
        self.stub("flock", "#!/bin/sh\nshift\nexec \"$@\"\n")
        self.stub("git", '''#!/usr/bin/env python3
import os, sys
args = sys.argv[1:]
if args == ['symbolic-ref', '--quiet', '--short', 'HEAD']:
    print('feature/test')
elif args == ['rev-parse', '--verify', 'HEAD']:
    print('a' * 40)
elif args[:2] == ['status', '--porcelain']:
    print(os.environ.get('NOVA_TEST_GIT_CHANGES', ''), end='')
else:
    sys.exit(2)
''')
        self.stub("xhost", '''#!/usr/bin/env python3
import json, os, sys
with open(os.environ['NOVA_TEST_EVENTS'], 'a') as f:
    f.write(json.dumps(['xhost', *sys.argv[1:]]) + '\\n')
sys.exit(int(os.environ.get('NOVA_TEST_XHOST_EXIT', '0')))
''')
        self.stub("colcon", '''#!/usr/bin/env python3
import json, os, pathlib, sys
with open(os.environ['NOVA_TEST_EVENTS'], 'a') as f:
    f.write(json.dumps(['colcon', *sys.argv[1:]]) + '\\n')
if os.environ.get('NOVA_TEST_BUILD_FAIL'):
    sys.exit(9)
pathlib.Path('install').mkdir(exist_ok=True)
pathlib.Path('install/setup.bash').write_text('export NOVA_TEST_SOURCED=1\\n')
''')
        self.stub("ros2", '''#!/usr/bin/env python3
import json, os, sys
with open(os.environ['NOVA_TEST_EVENTS'], 'a') as f:
    f.write(json.dumps(['ros2', os.environ.get('NOVA_TEST_SOURCED'), *sys.argv[1:]]) + '\\n')
''')
        self.stub("docker", '''#!/usr/bin/env python3
import json, os, sys
args = sys.argv[1:]
with open(os.environ['NOVA_TEST_EVENTS'], 'a') as f:
    f.write(json.dumps(['docker', *args[:3]]) + '\\n')
if args[:3] == ['compose', 'up', '-d']:
    sys.exit(0)
if args[:5] != ['compose', 'exec', 'dev', 'bash', '-ic']:
    sys.exit(2)
script, command = args[5], args[6:]
command = [os.path.join(os.environ['NOVA_TEST_ROOT'], 'scripts/full_stack.sh')
           if arg == '/vehicle_1811/scripts/full_stack.sh' else arg for arg in command]
os.execv('/bin/bash', ['bash', '-c', script, *command])
''')

    def stub(self, name, content):
        path = self.root / "bin" / name
        path.write_text(content)
        path.chmod(0o755)

    def run_g(self, *args, display=True, **extra_env):
        env = self.env.copy()
        env.update(extra_env)
        if not display:
            env.pop("DISPLAY", None)
        result = subprocess.run(
            [str(self.root / "g"), "port:=/dev/test", "lidar_ip:=1.2.3.4", *args],
            cwd=self.root, env=env, capture_output=True, text=True, timeout=10,
        )
        events = [json.loads(line) for line in self.events.read_text().splitlines()]
        return result, events

    def test_default_starts_panorama_rviz_and_incremental_build(self):
        result, events = self.run_g()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(["xhost", "+SI:localuser:root"], events)
        self.assertIn(["colcon", "build", "--symlink-install", "--packages-ignore",
                       "umrr_ros2_driver", "umrr_ros2_msgs", "smart_rviz_plugin"], events)
        launch = next(event for event in events if event[0] == "ros2")
        self.assertEqual(launch[1:5], ["1", "launch", "obc_bringup", "bringup.launch.py"])
        self.assertIn("panorama:=true", launch)
        self.assertIn("rviz:=true", launch)
        self.assertIn("jetson_update:=true", launch)
        self.assertIn("jetson_branch:=feature/test", launch)
        self.assertIn("jetson_revision:=" + "a" * 40, launch)

    def test_jetson_update_can_be_disabled(self):
        result, events = self.run_g("jetson_update:=false")
        self.assertEqual(result.returncode, 0, result.stderr)
        launch = next(event for event in events if event[0] == "ros2")
        self.assertIn("jetson_update:=false", launch)
        self.assertFalse(any(arg.startswith("jetson_revision:=") for arg in launch))

    def test_uncommitted_jetson_source_blocks_remote_start(self):
        result, events = self.run_g(NOVA_TEST_GIT_CHANGES=" M scripts/jetson_cameras.sh")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("uncommitted changes", result.stderr)
        launch = next(event for event in events if event[0] == "ros2")
        self.assertIn("jetson_revision:=", launch)

    def test_explicit_overrides_are_preserved(self):
        result, events = self.run_g("rviz:=false", "panorama:=false", "cameras:=false")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(any(event[0] == "xhost" for event in events))
        launch = next(event for event in events if event[0] == "ros2")
        self.assertEqual([arg for arg in launch if arg.startswith("rviz:=")], ["rviz:=false"])
        self.assertEqual([arg for arg in launch if arg.startswith("panorama:=")], ["panorama:=false"])
        self.assertIn("cameras:=false", launch)

    def test_no_display_starts_headless(self):
        result, events = self.run_g(display=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("No DISPLAY", result.stderr)
        self.assertFalse(any(event[0] == "xhost" for event in events))
        launch = next(event for event in events if event[0] == "ros2")
        self.assertIn("panorama:=true", launch)
        self.assertFalse(any(arg.startswith("rviz:=") for arg in launch))

    def test_disabling_cameras_does_not_enable_panorama(self):
        result, events = self.run_g("cameras:=false")
        self.assertEqual(result.returncode, 0, result.stderr)
        launch = next(event for event in events if event[0] == "ros2")
        self.assertFalse(any(arg.startswith("panorama:=") for arg in launch))

    def test_build_failure_does_not_start_vehicle(self):
        result, events = self.run_g(NOVA_TEST_BUILD_FAIL="1")
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(any(event[0] == "ros2" for event in events))

    def test_arguments_are_forwarded_without_shell_evaluation(self):
        marker = self.root / "unexpected"
        argument = f"jetson_repo:=literal $(touch {marker})"
        result, events = self.run_g(argument)
        self.assertEqual(result.returncode, 0, result.stderr)
        launch = next(event for event in events if event[0] == "ros2")
        self.assertIn(argument, launch)
        self.assertFalse(marker.exists())

    def test_xhost_failure_does_not_stop_vehicle(self):
        result, events = self.run_g(NOVA_TEST_XHOST_EXIT="1")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Could not grant RViz display access", result.stderr)
        self.assertTrue(any(event[0] == "ros2" for event in events))


if __name__ == "__main__":
    unittest.main()
