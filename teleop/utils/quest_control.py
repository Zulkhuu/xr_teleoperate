"""Hardware-independent Quest events and teleoperation state transitions."""
from dataclasses import dataclass
from enum import Enum, auto
import logging

START_HOLD_SEC = 0.5
EXIT_HOLD_SEC = 1.5
BUTTON_TAP_MAX_SEC = 0.7
ANCHOR_SETTLE_SEC = 0.15
ANCHOR_POSITION_TOL = 0.01
ANCHOR_ROTATION_TOL = 0.05
ACTIVATION_RAMP_SEC = 1.0
ARM_TARGET_RATE = 0.8  # rad/s, also applies in simulation
ROBOT_STALE_SEC = 0.5


class TeleopState(Enum):
    WAITING_FOR_XR = auto()
    WAITING_FOR_PREPARATION = auto()
    WAITING_FOR_ANCHOR = auto()
    ALIGNED = auto()
    PREPARING = auto()
    ACTIVE = auto()
    PAUSED = auto()
    DAMPED = auto()
    EXITING = auto()


class Button:
    def __init__(self):
        self.is_down = self.pressed = self.released = self.tap = False
        self.held_duration = 0.0
        self._since = None
        self._fired = set()
        self._armed = False

    def update(self, down, now):
        self.pressed = bool(down and not self.is_down)
        self.released = bool(self.is_down and not down)
        self.tap = False
        if self.pressed:
            self._since = now
            self._fired.clear()
        self.held_duration = now - self._since if self._since is not None else 0.0
        if self.released:
            self.tap = self._armed and self.held_duration <= BUTTON_TAP_MAX_SEC
            self._since = None
        if not down:
            self._armed = True
        self.is_down = bool(down)

    def long_pressed(self, seconds):
        if (self._armed and self.is_down and self.held_duration >= seconds
                and seconds not in self._fired):
            self._fired.add(seconds)
            return True
        return False

    def require_release(self):
        self._armed = False


class QuestControllerInput:
    # TeleVuer uses controller-relative A/B names on both controllers.
    FIELDS = dict(x='left_ctrl_aButton', y='left_ctrl_bButton',
                  a='right_ctrl_aButton', b='right_ctrl_bButton',
                  l3='left_ctrl_thumbstick', r3='right_ctrl_thumbstick')

    def __init__(self):
        for key in self.FIELDS:
            setattr(self, key, Button())

    def update(self, data, now, fresh=True):
        if not fresh:
            # Disconnect must not synthesize releases or preserve a held start.
            self.__init__()
            return
        for key, field in self.FIELDS.items():
            getattr(self, key).update(bool(getattr(data, field, False)), now)

    def events(self):
        return Events(pause=self.a.pressed,
                      anchor=self.b.tap, record=self.y.tap,
                      start=self.x.long_pressed(START_HOLD_SEC),
                      exit=self.a.long_pressed(EXIT_HOLD_SEC),
                      damp=self.l3.is_down and self.r3.is_down,
                      pause_down=self.a.is_down)


@dataclass
class Events:
    pause: bool = False
    anchor: bool = False
    record: bool = False
    start: bool = False
    exit: bool = False
    damp: bool = False
    pause_down: bool = False


class TeleopStateMachine:
    def __init__(self):
        self.state = TeleopState.WAITING_FOR_XR
        self.anchor = False
        self.anchor_requested = False
        self.record_requested = False
        self.recording = False
        self.reason = ''

    def transition(self, state, reason=''):
        if state != self.state or reason != self.reason:
            logging.getLogger(__name__).info('[Teleop] %s %s', state.name, reason)
        self.state, self.reason = state, reason

    def pause(self, reason):
        self.anchor = self.anchor_requested = False
        if self.state not in (TeleopState.DAMPED, TeleopState.EXITING):
            self.transition(TeleopState.PAUSED, reason)

    def anchored(self):
        self.anchor_requested = False
        self.anchor = True
        self.transition(TeleopState.ALIGNED, 'Anchor captured')

    def update(self, event, tracking, robot_ready, prepared=True):
        self.record_requested = False
        if self.state == TeleopState.EXITING:
            return
        if event.damp:
            self.anchor = self.anchor_requested = False
            self.transition(TeleopState.DAMPED, 'Check robot before re-aligning')
            return
        if event.pause:
            self.pause('Fresh anchor required')
            return
        if event.exit and self.state in (TeleopState.PAUSED, TeleopState.DAMPED):
            self.transition(TeleopState.EXITING, 'Graceful exit')
            return
        if not tracking or not robot_ready:
            self.anchor = self.anchor_requested = False
            if self.state in (TeleopState.ACTIVE, TeleopState.ALIGNED, TeleopState.PREPARING):
                self.pause('Tracking lost' if not tracking else 'Robot state unavailable')
            return
        if self.state == TeleopState.WAITING_FOR_XR:
            self.transition(TeleopState.WAITING_FOR_ANCHOR if prepared else
                            TeleopState.WAITING_FOR_PREPARATION, 'XR tracking ready')
        if not prepared and self.state == TeleopState.WAITING_FOR_ANCHOR:
            self.transition(TeleopState.WAITING_FOR_PREPARATION)
        if event.anchor and not event.pause_down:
            if not prepared:
                # After checking a damped robot, B acknowledges recovery only.
                # Initial preparation must still finish before capturing poses.
                if self.state == TeleopState.DAMPED:
                    self.transition(TeleopState.WAITING_FOR_PREPARATION,
                                    'Recovery acknowledged; hold X to prepare')
                return
            self.anchor = False
            self.anchor_requested = True
            if self.state != TeleopState.DAMPED:
                self.transition(TeleopState.PAUSED, 'Hold controllers still for anchor')
        # The runtime validates IK before committing activation.
        self.record_requested = event.record and self.state == TeleopState.ACTIVE

    def activate(self, ik_valid):
        if self.state == TeleopState.ALIGNED and self.anchor and ik_valid:
            self.transition(TeleopState.ACTIVE)
            return True
        return False
