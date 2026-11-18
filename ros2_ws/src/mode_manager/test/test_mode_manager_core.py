"""
Unit tests for mode_manager.mode_manager_core -- no ROS required.

Run with `pytest test/test_mode_manager_core.py`, or as part of
`colcon test --packages-select mode_manager`.
"""
import pytest

from mode_manager import mode_manager_core as mmc

G = mmc.GamepadMap()
N_BUTTONS = 11
TRIGGER_RELEASED = 1.0   # this pad's trigger rests at +1.0
TRIGGER_PRESSED = -1.0

MANUAL_CMD = mmc.Command(throttle=0.3, steer=-0.1, brake=0.0)
AUTO_CMD = mmc.Command(throttle=0.4, steer=0.2, brake=0.0)


def joy(*held, brake_raw=TRIGGER_RELEASED):
    buttons = [0] * N_BUTTONS
    for index in held:
        buttons[index] = 1
    axes = [0.0] * 6
    axes[G.brake_axis] = brake_raw
    return buttons, axes


def press(mm, now, *held, brake_raw=TRIGGER_RELEASED):
    """One press-and-release of `held`, so the next press is a fresh rising edge."""
    events = mm.on_joy(*joy(*held, brake_raw=brake_raw), now)
    mm.on_joy(*joy(brake_raw=brake_raw), now)
    return events


def is_stop(cmd, mm):
    return cmd.throttle == 0.0 and cmd.steer == 0.0 and cmd.brake == mm.stop_brake


@pytest.fixture
def mm():
    return mmc.ModeManager()


def autonomous(mm, now=0.0):
    """Get into AUTONOMOUS with fresh inputs."""
    mm.on_auto(AUTO_CMD, now)
    press(mm, now, G.autonomous_button)
    assert mm.mode == mmc.AUTONOMOUS
    return mm


# --- startup / DISABLED --------------------------------------------------

def test_starts_disabled_and_brakes(mm):
    mm.on_manual(MANUAL_CMD, 0.0)
    mm.on_auto(AUTO_CMD, 0.0)
    assert mm.mode == mmc.DISABLED
    assert is_stop(mm.output(0.0), mm)       # guide A3: DISABLED -> zero throttle, brake on


def test_trigger_brake_normalization():
    assert mmc.trigger_to_brake(1.0, True) == 0.0
    assert mmc.trigger_to_brake(-1.0, True) == 1.0
    assert mmc.trigger_to_brake(0.0, True) == pytest.approx(0.5)
    assert mmc.trigger_to_brake(0.7, False) == pytest.approx(0.7)
    assert mmc.trigger_to_brake(5.0, False) == 1.0


# --- MANUAL --------------------------------------------------------------

def test_manual_passes_the_gamepad_through(mm):
    press(mm, 0.0, G.manual_button)
    mm.on_manual(MANUAL_CMD, 0.0)
    mm.on_auto(AUTO_CMD, 0.0)
    assert mm.mode == mmc.MANUAL
    assert mm.output(0.1) == MANUAL_CMD


def test_manual_stops_when_gamepad_goes_quiet(mm):
    press(mm, 0.0, G.manual_button)
    mm.on_manual(MANUAL_CMD, 0.0)
    assert is_stop(mm.output(mm.input_timeout_s + 0.01), mm)


def test_manual_with_no_input_yet_stops(mm):
    press(mm, 0.0, G.manual_button)
    assert is_stop(mm.output(0.0), mm)


# --- AUTONOMOUS ------------------------------------------------------------

def test_autonomous_drives_only_while_deadman_held(mm):
    autonomous(mm)
    mm.on_joy(*joy(G.deadman_button), 0.1)
    assert mm.output(0.1) == AUTO_CMD
    mm.on_joy(*joy(), 0.2)                  # deadman released
    assert mm.mode == mmc.AUTONOMOUS        # stays armed...
    assert is_stop(mm.output(0.2), mm)      # ...but brakes


def test_autonomous_ignores_the_gamepad_sticks(mm):
    autonomous(mm)
    mm.on_manual(MANUAL_CMD, 0.1)
    mm.on_joy(*joy(G.deadman_button), 0.1)
    assert mm.output(0.1) == AUTO_CMD


def test_autonomous_stops_when_pure_pursuit_goes_quiet(mm):
    autonomous(mm, now=0.0)
    now = mm.input_timeout_s + 0.01
    mm.on_joy(*joy(G.deadman_button), now)
    assert is_stop(mm.output(now), mm)


def test_autonomous_stops_when_gamepad_goes_quiet(mm):
    autonomous(mm, now=0.0)
    mm.on_joy(*joy(G.deadman_button), 0.0)   # last joy message: deadman held
    now = mm.input_timeout_s + 0.01
    mm.on_auto(AUTO_CMD, now)
    assert is_stop(mm.output(now), mm)       # held-at-disconnect must not keep driving


def test_brake_trigger_takes_over_to_manual(mm):
    autonomous(mm)
    mm.on_joy(*joy(G.deadman_button, brake_raw=TRIGGER_PRESSED), 0.1)
    assert mm.mode == mmc.MANUAL
    assert 'takeover' in mm.reason


def test_autonomous_refused_without_pure_pursuit(mm):
    press(mm, 0.0, G.autonomous_button)
    assert mm.mode == mmc.DISABLED
    assert mm.reason.startswith(mmc.REFUSED)


def test_autonomous_refused_while_braking(mm):
    mm.on_auto(AUTO_CMD, 0.0)
    press(mm, 0.0, G.autonomous_button, brake_raw=TRIGGER_PRESSED)
    assert mm.mode == mmc.DISABLED
    assert 'brake' in mm.reason


def test_autonomous_refused_with_stale_pure_pursuit(mm):
    mm.on_auto(AUTO_CMD, 0.0)
    press(mm, mm.input_timeout_s + 0.01, G.autonomous_button)
    assert mm.mode == mmc.DISABLED


# --- buttons ---------------------------------------------------------------

def test_disable_wins_over_everything_pressed_with_it(mm):
    autonomous(mm)
    mm.on_joy(*joy(G.disable_button, G.manual_button, G.autonomous_button), 0.1)
    assert mm.mode == mmc.DISABLED


def test_holding_a_button_is_one_press(mm):
    mm.on_joy(*joy(G.manual_button), 0.0)
    mm.on_joy(*joy(G.disable_button, G.manual_button), 0.1)   # manual still held
    mm.on_joy(*joy(G.manual_button), 0.2)                     # still held: no new edge
    assert mm.mode == mmc.DISABLED


def test_record_button_emits_one_event_per_press(mm):
    assert press(mm, 0.0, G.record_button) == [mmc.EVENT_RECORD_TOGGLE]
    assert mm.on_joy(*joy(G.record_button), 0.1) == [mmc.EVENT_RECORD_TOGGLE]
    assert mm.on_joy(*joy(G.record_button), 0.2) == []        # held, not re-pressed


def test_short_button_array_is_safe(mm):
    autonomous(mm)
    mm.on_joy([0, 0], [0.0, 0.0, TRIGGER_RELEASED], 0.1)      # pad changed mode mid-drive
    assert not mm.deadman_held
    assert is_stop(mm.output(0.1), mm)


# --- brake release easing ------------------------------------------------

def easing():
    return mmc.ModeManager(brake_release_s=0.4)


def test_brake_comes_off_gradually_and_throttle_waits_for_it():
    mm = easing()
    assert is_stop(mm.output(0.0), mm)
    mm.on_manual(MANUAL_CMD, 0.05)
    press(mm, 0.05, G.manual_button)
    first = mm.output(0.1)                   # 0.1 s after full brake: 1.0 - 0.1 / 0.4
    assert first.brake == pytest.approx(0.75)
    assert first.throttle == 0.0             # no throttle against a dragging brake
    assert first.steer == MANUAL_CMD.steer   # steering is never held back
    assert mm.output(0.3).brake == pytest.approx(0.25)
    assert mm.output(0.5) == MANUAL_CMD      # fully off: the gamepad passes through again


def test_braking_is_still_instant():
    mm = easing()
    mm.output(0.0)
    mm.on_manual(MANUAL_CMD, 0.0)
    press(mm, 0.0, G.manual_button)
    for t in (0.1, 0.2, 0.3, 0.4, 0.5):
        mm.on_manual(MANUAL_CMD, t)
        mm.output(t)
    assert mm.output(0.5) == MANUAL_CMD
    press(mm, 0.55, G.disable_button)
    assert is_stop(mm.output(0.55), mm)      # full brake on the very next command


def test_rapid_a_b_presses_keep_the_brake_mostly_on():
    mm = easing()
    mm.output(-0.05)                         # starts DISABLED: full brake
    brakes = []
    t = 0.0
    for i in range(20):                      # A, B, A, B ... every 0.1 s, 20 Hz output
        mm.on_manual(MANUAL_CMD, t)
        press(mm, t, G.manual_button if i % 2 == 0 else G.disable_button)
        for dt in (0.0, 0.05):
            brakes.append(mm.output(t + dt).brake)
        t += 0.1
    assert min(brakes) >= 0.7                # never gets far from full brake...
    steps = [b - a for a, b in zip(brakes, brakes[1:]) if b > a]
    assert max(steps) <= 0.3                 # ...so each B is a small servo move, not a full slam


def test_missing_brake_axis_reads_as_released(mm):
    mm.on_joy([0] * N_BUTTONS, [0.0], 0.0)
    mm.on_auto(AUTO_CMD, 0.0)
    assert mm.request_autonomous(0.0)
