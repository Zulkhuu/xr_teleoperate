"""Adapt XR input and recording to the device controllers' shared buffers."""
import time
from dataclasses import dataclass
from importlib import import_module
from multiprocessing import Array, Lock, Value

import logging_mp

logger = logging_mp.getLogger(__name__)

# module, controller class, joints per side
DEVICES = {
    'dex1': ('robot_hand_unitree', 'Dex1_1_Gripper_Controller', 1),
    'dex3': ('robot_hand_unitree', 'Dex3_1_Controller', 7),
    'inspire_ftp': ('robot_hand_inspire', 'Inspire_Controller_FTP', 6),
    'inspire_dfx': ('robot_hand_inspire', 'Inspire_Controller_DFX', 6),
    'brainco': ('robot_hand_brainco', 'Brainco_Controller', 6),
}


@dataclass
class EndEffectorSnapshot:
    left_state: list
    right_state: list
    left_action: list
    right_action: list


class EndEffectorSession:
    """Own device construction, input mapping, and coherent recording snapshots.

    Binary hand controllers start during the arm safety sequence. Other
    controllers start during setup, matching the existing teleoperation flow.
    """

    def __init__(self, args):
        self.device = args.ee
        self.input_mode = args.input_mode
        self.simulation = args.sim
        self.binary = self.device in ('dex3', 'inspire_ftp', 'inspire_dfx') and self.input_mode == 'controller'
        self.deferred = self.binary and not args.disable_hand_safety_sequence
        self.controller = None
        self._last_grasp = None
        self.label = 'Dex3' if self.device == 'dex3' else 'Inspire'
        self.joints = 0
        if self.device is None:
            return
        module, name, self.joints = DEVICES[self.device]
        self._controller_type = getattr(import_module(f'teleop.robot_control.{module}'), name)
        self._lock = Lock()
        self._state = Array('d', self.joints * 2, lock=False)
        self._action = Array('d', self.joints * 2, lock=False)
        if self.binary or self.device == 'dex1':
            self._left = Value('d', 0.0, lock=True)
            self._right = Value('d', 0.0, lock=True)
        else:
            self._left = Array('d', 75, lock=True)
            self._right = Array('d', 75, lock=True)

    def start(self):
        """Start at most once; caller chooses the lifecycle phase."""
        if self.device is not None and self.controller is None:
            options = {'simulation_mode': self.simulation}
            if self.binary:
                options['control_mode'] = 'binary'
            self.controller = self._controller_type(
                self._left, self._right, self._lock, self._state, self._action, **options)
        return self.controller

    def set_grasp(self, close, settle_time=0.6):
        """Apply lifecycle open/close only to binary hands, as before."""
        if not self.binary:
            return False
        target = 1.0 if close else 0.0
        with self._left.get_lock():
            self._left.value = target
        with self._right.get_lock():
            self._right.value = target
        logger.info(f"[{self.label} safety] {'close' if close else 'open'} both hands.")
        if settle_time > 0:
            time.sleep(settle_time)
        return True

    def update(self, tele_data):
        if self.device is None:
            return
        if self.binary:
            grasp = (tele_data.left_ctrl_triggerValue < 5.0,
                     tele_data.right_ctrl_triggerValue < 5.0)
            if grasp != self._last_grasp:
                logger.info(f'[{self.label} controller] trigger L={tele_data.left_ctrl_triggerValue:.2f} '
                            f"({'close' if grasp[0] else 'open'}), "
                            f'R={tele_data.right_ctrl_triggerValue:.2f} '
                            f"({'close' if grasp[1] else 'open'})")
                self._last_grasp = grasp
            left, right = map(float, grasp)
        elif self.device == 'dex1':
            if self.input_mode == 'controller':
                left, right = tele_data.left_ctrl_triggerValue, tele_data.right_ctrl_triggerValue
            else:
                left, right = tele_data.left_hand_pinchValue, tele_data.right_hand_pinchValue
        elif self.input_mode == 'hand':
            with self._left.get_lock():
                self._left[:] = tele_data.left_hand_pos.flatten()
            with self._right.get_lock():
                self._right[:] = tele_data.right_hand_pos.flatten()
            return
        else:
            # BrainCo controller input had no mapping in the original loop.
            return
        with self._left.get_lock():
            self._left.value = left
        with self._right.get_lock():
            self._right.value = right

    def snapshot(self):
        if self.device is None or (self.device == 'brainco' and self.input_mode == 'controller'):
            return EndEffectorSnapshot([], [], [], [])
        with self._lock:
            return EndEffectorSnapshot(self._state[:self.joints], self._state[self.joints:],
                                       self._action[:self.joints], self._action[self.joints:])


def create_end_effector(args):
    session = EndEffectorSession(args)
    if not session.deferred:
        session.start()
    return session
