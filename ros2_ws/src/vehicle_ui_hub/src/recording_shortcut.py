"""Recognize deliberate double presses of an analog gamepad trigger."""

import math


class TriggerDoublePress:
    def __init__(self, window_s=2.0, debounce_s=0.1):
        self.window_s = window_s
        self.debounce_s = debounce_s
        self._mapping = None
        self.reset()

    def reset(self):
        # Require a released sample before recognizing a press, including
        # after startup, a malformed message, or a controller mapping change.
        self._released = False
        self._first_press = None

    def update(self, axes, now, axis=5, rests_at_plus_one=True):
        mapping = (axis, rests_at_plus_one)
        if mapping != self._mapping:
            self.reset()
            self._mapping = mapping
        if (not isinstance(axis, int) or isinstance(axis, bool) or axis < 0 or
                axis >= len(axes) or not isinstance(rests_at_plus_one, bool)):
            self.reset()
            return False
        value = axes[axis]
        if not math.isfinite(value) or value < -1.1 or value > 1.1:
            self.reset()
            return False

        # Hysteresis prevents trigger noise near the threshold from counting
        # as another press. The alternate convention rests at zero and rises.
        amount = (1.0 - value) / 2.0 if rests_at_plus_one else value
        if amount <= 0.2:
            self._released = True
            return False
        if amount < 0.6 or not self._released:
            return False
        self._released = False

        if self._first_press is None or now < self._first_press or now - self._first_press > self.window_s:
            self._first_press = now
            return False
        if now - self._first_press < self.debounce_s:
            return False
        self._first_press = None
        return True
