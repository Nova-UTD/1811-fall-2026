"""Exercise the trigger gesture without ROS, Qt, or a physical gamepad."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from recording_shortcut import TriggerDoublePress


class TriggerDoublePressTest(unittest.TestCase):
    def setUp(self):
        self.detector = TriggerDoublePress()

    def send(self, value, now, axis=5, rests_at_plus_one=True):
        axes = [0.0] * (axis + 1)
        axes[axis] = value
        return self.detector.update(axes, now, axis, rests_at_plus_one)

    def test_two_distinct_presses_toggle_once_and_can_toggle_again(self):
        self.assertFalse(self.send(1.0, 0.0))
        self.assertFalse(self.send(-1.0, 0.1))
        self.assertFalse(self.send(-1.0, 0.2))  # Held, not another press.
        self.assertFalse(self.send(1.0, 0.3))
        self.assertTrue(self.send(-1.0, 0.5))
        self.assertFalse(self.send(-1.0, 0.6))
        self.assertFalse(self.send(1.0, 0.7))
        self.assertFalse(self.send(-1.0, 1.0))
        self.assertFalse(self.send(1.0, 1.1))
        self.assertTrue(self.send(-1.0, 1.3))

    def test_late_second_press_becomes_the_first_of_a_new_pair(self):
        self.send(1.0, 0.0)
        self.send(-1.0, 0.1)
        self.send(1.0, 0.2)
        self.assertFalse(self.send(-1.0, 2.2))
        self.send(1.0, 2.3)
        self.assertTrue(self.send(-1.0, 2.5))

    def test_startup_held_trigger_noise_and_bounce_do_not_toggle(self):
        self.assertFalse(self.send(-1.0, 0.0))  # No released sample yet.
        self.assertFalse(self.send(1.0, 0.1))
        self.assertFalse(self.send(-1.0, 0.2))
        self.assertFalse(self.send(-0.1, 0.21))  # Between hysteresis thresholds.
        self.assertFalse(self.send(-1.0, 0.22))
        self.assertFalse(self.send(1.0, 0.23))
        self.assertFalse(self.send(-1.0, 0.24))  # Too close to be deliberate.
        self.assertFalse(self.send(1.0, 0.3))
        self.assertTrue(self.send(-1.0, 0.5))

    def test_bad_message_or_new_mapping_requires_release_and_two_new_presses(self):
        self.send(1.0, 0.0)
        self.send(-1.0, 0.1)
        self.assertFalse(self.detector.update([], 0.2))
        self.assertFalse(self.send(-1.0, 0.3))
        self.send(1.0, 0.4)
        self.assertFalse(self.send(-1.0, 0.6))
        self.assertFalse(self.send(float('nan'), 0.7))
        self.assertFalse(self.send(0.0, 0.8, axis=4, rests_at_plus_one=False))
        self.assertFalse(self.send(1.0, 1.0, axis=4, rests_at_plus_one=False))
        self.assertFalse(self.send(0.0, 1.2, axis=4, rests_at_plus_one=False))
        self.assertTrue(self.send(1.0, 1.4, axis=4, rests_at_plus_one=False))


if __name__ == '__main__':
    unittest.main()
