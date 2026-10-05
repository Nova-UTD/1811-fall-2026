"""
Mode manager logic -- DISABLED / MANUAL / AUTONOMOUS, with zero ROS imports.

Kept free of rclpy/message types (same split as control.pure_pursuit_core) so
the safety-relevant decisions are unit-tested in test/test_mode_manager_core.py
without booting ROS. mode_manager_node.py is the thin rclpy wrapper.

The one rule this module exists to enforce: exactly ONE command source reaches
/vehicle_command at a time.

  DISABLED    -> all zeros (coast). Same thing the Arduino does on its own when
                 the link goes quiet, and lets you push the car by hand for a
                 dry run while watching /cmd/auto.
  MANUAL      -> gamepad (/cmd/manual) passes through.
  AUTONOMOUS  -> pure pursuit (/cmd/auto) passes through, but ONLY while the
                 deadman button is held. Released deadman, stale input, or a
                 lost gamepad all output a brake-and-stop command instead.

Squeezing the brake trigger while AUTONOMOUS is a takeover: it drops straight to
MANUAL, so the brake the driver is already pressing reaches the car at once.
"""
from dataclasses import dataclass
from typing import List, Optional, Sequence

DISABLED = 'DISABLED'
MANUAL = 'MANUAL'
AUTONOMOUS = 'AUTONOMOUS'

# Events handed back to the caller for anything that isn't a pure mode switch.
EVENT_RECORD_TOGGLE = 'record_toggle'
# Prefix of ModeManager.reason when a switch to AUTONOMOUS was turned down.
REFUSED = 'AUTONOMOUS refused'


@dataclass
class Command:
    throttle: float = 0.0
    steer: float = 0.0
    brake: float = 0.0


@dataclass
class GamepadMap:
    """
    Button/axis indices into sensor_msgs/Joy. Defaults are the Linux xpad layout.

    The axes (brake on axes[2], resting at +1.0) match what gamepad_node.py
    measured on the 8BitDo SN30 Pro in wired mode. The BUTTON indices are the
    standard xpad ones and have NOT been checked on that pad yet -- verify with
    `ros2 topic echo /joy` before trusting them.
    """

    manual_button: int = 0        # A
    disable_button: int = 1       # B
    record_button: int = 3        # Y
    deadman_button: int = 5       # RB -- hold to let AUTONOMOUS drive
    autonomous_button: int = 7    # Start
    brake_axis: int = 2           # left trigger
    trigger_rests_at_plus_one: bool = True


def trigger_to_brake(raw: float, rests_at_plus_one: bool) -> float:
    """Normalize a trigger axis to 0.0 (released) .. 1.0 (fully pressed)."""
    brake = (1.0 - raw) / 2.0 if rests_at_plus_one else raw
    return max(0.0, min(1.0, brake))


def _pressed(buttons: Sequence[int], index: int) -> bool:
    # A pad that enumerates in another mode can report fewer buttons. Missing
    # means "not pressed" -- which for the deadman is the safe answer.
    return 0 <= index < len(buttons) and bool(buttons[index])


class ModeManager:
    """
    Owns the current mode and decides what reaches /vehicle_command.

    All times are plain float seconds supplied by the caller, so tests can
    drive the clock by hand.
    """

    def __init__(self, gamepad: Optional[GamepadMap] = None, input_timeout_s: float = 0.5,
                 stop_brake: float = 1.0, takeover_brake: float = 0.2):
        self.gamepad = gamepad or GamepadMap()
        self.input_timeout_s = input_timeout_s
        self.stop_brake = stop_brake
        self.takeover_brake = takeover_brake

        self.mode = DISABLED
        # Why the last mode change happened, for the node to log.
        self.reason = 'startup'

        self._prev_buttons: List[int] = []
        self._deadman = False
        self._brake = 0.0
        self._joy_t: Optional[float] = None
        self._manual: Optional[Command] = None
        self._manual_t: Optional[float] = None
        self._auto: Optional[Command] = None
        self._auto_t: Optional[float] = None

    # --- inputs -----------------------------------------------------------

    def on_manual(self, cmd: Command, now: float) -> None:
        self._manual, self._manual_t = cmd, now

    def on_auto(self, cmd: Command, now: float) -> None:
        self._auto, self._auto_t = cmd, now

    def on_joy(self, buttons: Sequence[int], axes: Sequence[float], now: float) -> List[str]:
        """Process one gamepad message. Returns events (e.g. EVENT_RECORD_TOGGLE)."""
        g = self.gamepad
        self._joy_t = now
        self._deadman = _pressed(buttons, g.deadman_button)
        if 0 <= g.brake_axis < len(axes):
            self._brake = trigger_to_brake(axes[g.brake_axis], g.trigger_rests_at_plus_one)
        else:
            self._brake = 0.0

        def rising(index: int) -> bool:
            return _pressed(buttons, index) and not _pressed(self._prev_buttons, index)

        events: List[str] = []
        # Priority order: stopping always wins over anything else pressed at once.
        if rising(g.disable_button):
            self._set(DISABLED, 'disable button')
        elif rising(g.manual_button):
            self._set(MANUAL, 'manual button')
        elif rising(g.autonomous_button):
            self.request_autonomous(now)
        elif self.mode == AUTONOMOUS and self._brake > self.takeover_brake:
            self._set(MANUAL, f'brake takeover ({self._brake:.2f})')

        if rising(g.record_button):
            events.append(EVENT_RECORD_TOGGLE)

        self._prev_buttons = list(buttons)
        return events

    def request_autonomous(self, now: float) -> bool:
        """Enter AUTONOMOUS if it's safe to; otherwise stay put and say why."""
        if self.mode == AUTONOMOUS:
            return True
        if not self._fresh(self._auto_t, now):
            self.reason = f'{REFUSED}: no recent /cmd/auto (is pure_pursuit running?)'
            return False
        if not self._fresh(self._joy_t, now):
            self.reason = f'{REFUSED}: gamepad not publishing'
            return False
        if self._brake > self.takeover_brake:
            self.reason = f'{REFUSED}: brake trigger is pressed'
            return False
        self._set(AUTONOMOUS, 'autonomous button')
        return True

    # --- output -----------------------------------------------------------

    def output(self, now: float) -> Command:
        """Return the one command that should go to /vehicle_command right now."""
        if self.mode == DISABLED:
            return Command()
        if self.mode == MANUAL:
            if self._fresh(self._manual_t, now):
                return self._manual
            return self._stop()
        # AUTONOMOUS: every condition must hold, or stop.
        if not self._fresh(self._joy_t, now) or not self._deadman:
            return self._stop()
        if not self._fresh(self._auto_t, now):
            return self._stop()
        return self._auto

    @property
    def deadman_held(self) -> bool:
        return self._deadman

    # --- internals ----------------------------------------------------------

    def _set(self, mode: str, reason: str) -> None:
        self.mode = mode
        self.reason = reason

    def _fresh(self, stamp: Optional[float], now: float) -> bool:
        return stamp is not None and (now - stamp) <= self.input_timeout_s

    def _stop(self) -> Command:
        return Command(throttle=0.0, steer=0.0, brake=self.stop_brake)
