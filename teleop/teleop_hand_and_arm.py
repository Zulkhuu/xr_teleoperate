import time
import argparse
from multiprocessing import Value, Array, Lock
import threading
import logging_mp
logging_mp.basicConfig(level=logging_mp.INFO)
logger_mp = logging_mp.getLogger(__name__)

import os 
import sys
import numpy as np
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize # dds 
from televuer import TeleVuerWrapper
from teleop.robot_control.robot_arm import G1_29_ArmController, G1_23_ArmController, H1_2_ArmController, H1_ArmController, H2_ArmController
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK, G1_23_ArmIK, H1_2_ArmIK, H1_ArmIK, H2_ArmIK
from teleimager.image_client import ImageClient
from teleop.utils.episode_writer import EpisodeWriter
from teleop.utils.ipc import IPC_Server
from teleop.utils.motion_switcher import MotionSwitcher, LocoClientWrapper
from sshkeyboard import listen_keyboard, stop_listening

# for simulation
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
def publish_reset_category(category: int, publisher): # Scene Reset signal
    msg = String_(data=str(category))
    publisher.Write(msg)
    logger_mp.info(f"published reset category: {category}")

# state transition
START          = False  # Enable to start robot following VR user motion
STOP           = False  # Enable to begin system exit procedure
READY          = False  # Ready to (1) enter START state, (2) enter RECORD_RUNNING state
RECORD_RUNNING = False  # True if [Recording]
RECORD_TOGGLE  = False  # Toggle recording state
#  -------        ---------                -----------                -----------            ---------
#   state          [Ready]      ==>        [Recording]     ==>         [AutoSave]     -->     [Ready]
#  -------        ---------      |         -----------      |         -----------      |     ---------
#   START           True         |manual      True          |manual      True          |        True
#   READY           True         |set         False         |set         False         |auto    True
#   RECORD_RUNNING  False        |to          True          |to          False         |        False
#                                ∨                          ∨                          ∨
#   RECORD_TOGGLE   False       True          False        True          False                  False
#  -------        ---------                -----------                 -----------            ---------
#  ==> manual: when READY is True, set RECORD_TOGGLE=True to transition.
#  --> auto  : Auto-transition after saving data.

def on_press(key):
    global STOP, START, RECORD_TOGGLE
    if key == 'r':
        START = True
    elif key == 'q':
        START = False
        STOP = True
    elif key == 's' and START == True:
        RECORD_TOGGLE = True
    else:
        logger_mp.warning(f"[on_press] {key} was pressed, but no action is defined for this key.")

def get_state() -> dict:
    """Return current heartbeat state"""
    global START, STOP, RECORD_RUNNING, READY
    return {
        "START": START,
        "STOP": STOP,
        "READY": READY,
        "RECORD_RUNNING": RECORD_RUNNING,
    }

def fast_pose_inv(pose):
    pose_inv = np.eye(4)
    pose_inv[:3, :3] = pose[:3, :3].T
    pose_inv[:3, 3] = -pose[:3, :3].T @ pose[:3, 3]
    return pose_inv

def align_wrist_pose_to_start(current_pose, xr_start_pose, robot_start_pose):
    return robot_start_pose @ fast_pose_inv(xr_start_pose) @ current_pose

def move_dual_arm_to_q_slow(
        arm_ctrl,
        q_target,
        tauff_target=None,
        velocity_limit=0.8,
        timeout=12.0,
        tolerance=0.08,
        restore_previous=True):
    previous_velocity_limit = getattr(arm_ctrl, "arm_velocity_limit", None)
    previous_speed_gradual_max = getattr(arm_ctrl, "_speed_gradual_max", None)
    if hasattr(arm_ctrl, "_speed_gradual_max"):
        arm_ctrl._speed_gradual_max = False
    if hasattr(arm_ctrl, "arm_velocity_limit"):
        arm_ctrl.arm_velocity_limit = velocity_limit

    if tauff_target is None:
        tauff_target = np.zeros_like(q_target)
    arm_ctrl.ctrl_dual_arm(q_target, tauff_target)

    reached = False
    deadline = time.time() + timeout
    while time.time() < deadline:
        current_q = arm_ctrl.get_current_dual_arm_q()
        if np.all(np.abs(current_q - q_target) < tolerance):
            reached = True
            break
        time.sleep(0.05)

    if restore_previous and previous_velocity_limit is not None:
        arm_ctrl.arm_velocity_limit = previous_velocity_limit
    if restore_previous and previous_speed_gradual_max is not None:
        arm_ctrl._speed_gradual_max = previous_speed_gradual_max
    return reached

def hold_current_arm_pose(arm_ctrl, hold_time=0.5, velocity_limit=0.2, label="current arm pose", restore_previous=True):
    previous_velocity_limit = getattr(arm_ctrl, "arm_velocity_limit", None)
    previous_speed_gradual_max = getattr(arm_ctrl, "_speed_gradual_max", None)
    if hasattr(arm_ctrl, "_speed_gradual_max"):
        arm_ctrl._speed_gradual_max = False
    if hasattr(arm_ctrl, "arm_velocity_limit"):
        arm_ctrl.arm_velocity_limit = velocity_limit

    current_q = arm_ctrl.get_current_dual_arm_q()
    tauff_target = np.zeros_like(current_q)
    logger_mp.info(f"Hold {label}.")
    deadline = time.time() + hold_time
    while time.time() < deadline:
        arm_ctrl.ctrl_dual_arm(current_q, tauff_target)
        time.sleep(0.02)

    if restore_previous and previous_velocity_limit is not None:
        arm_ctrl.arm_velocity_limit = previous_velocity_limit
    if restore_previous and previous_speed_gradual_max is not None:
        arm_ctrl._speed_gradual_max = previous_speed_gradual_max
    return current_q

def move_dual_arm_to_safety_pose(arm_ctrl, velocity_limit=0.8, timeout=30.0, hold_time=0.3, restore_previous=True):
    q_target = np.zeros_like(arm_ctrl.get_current_dual_arm_q())
    reached = move_dual_arm_to_q_slow(
        arm_ctrl,
        q_target,
        np.zeros_like(q_target),
        velocity_limit=velocity_limit,
        timeout=timeout,
        tolerance=0.08,
        restore_previous=restore_previous,
    )
    if reached:
        logger_mp.info("Arm safety pose reached.")
        time.sleep(hold_time)
    else:
        logger_mp.warning("Arm safety pose timed out; continuing with current arm pose.")
    return reached

def get_startup_dual_arm_q(arm_ctrl):
    startup_q = getattr(arm_ctrl, "startup_dual_arm_q", None)
    if startup_q is None:
        startup_q = getattr(arm_ctrl, "initial_dual_arm_q", None)
    if startup_q is None:
        return None
    startup_q = np.asarray(startup_q, dtype=float)
    current_q = arm_ctrl.get_current_dual_arm_q()
    if startup_q.shape != current_q.shape:
        logger_mp.warning("Startup arm pose shape does not match current arm pose; skip controlled lowering.")
        return None
    return startup_q.copy()

def move_dual_arm_to_startup_pose(arm_ctrl, velocity_limit=0.35, timeout=35.0, hold_time=0.5):
    startup_q = get_startup_dual_arm_q(arm_ctrl)
    if startup_q is None:
        logger_mp.warning("Controlled lowering skipped: startup arm pose is unavailable.")
        return False

    logger_mp.info("Exit safety sequence: slowly lower arms to startup standing pose under SDK control.")
    reached = move_dual_arm_to_q_slow(
        arm_ctrl,
        startup_q,
        np.zeros_like(startup_q),
        velocity_limit=velocity_limit,
        timeout=timeout,
        tolerance=0.10,
        restore_previous=False,
    )
    if reached:
        logger_mp.info("Startup standing arm pose reached.")
    else:
        logger_mp.warning("Startup standing arm pose timed out; holding current arm pose before release.")
    hold_current_arm_pose(
        arm_ctrl,
        hold_time=hold_time,
        velocity_limit=0.2,
        label="exit release pose",
        restore_previous=False,
    )
    return reached

def set_binary_grasp_values(left_value, right_value, close, label="hand", settle_time=0.6):
    if left_value is None or right_value is None:
        return False

    target = 1.0 if close else 0.0
    with left_value.get_lock():
        left_value.value = target
    with right_value.get_lock():
        right_value.value = target
    logger_mp.info(f"[{label} safety] {'close' if close else 'open'} both hands.")
    if settle_time > 0:
        time.sleep(settle_time)
    return True

def make_exit_retreat_waypoints(left_pose, right_pose, lateral_clearance=0.30, forward_clearance=0.25, min_height=0.12, lift=0.05):
    left_lateral = left_pose.copy()
    right_lateral = right_pose.copy()
    left_lateral[1, 3] = max(left_lateral[1, 3], lateral_clearance)
    right_lateral[1, 3] = min(right_lateral[1, 3], -lateral_clearance)
    left_lateral[2, 3] = max(left_lateral[2, 3] + lift, min_height)
    right_lateral[2, 3] = max(right_lateral[2, 3] + lift, min_height)

    left_forward = left_lateral.copy()
    right_forward = right_lateral.copy()
    left_forward[0, 3] = max(left_forward[0, 3], forward_clearance)
    right_forward[0, 3] = max(right_forward[0, 3], forward_clearance)

    return [(left_lateral, right_lateral), (left_forward, right_forward)]

def retreat_arms_before_motion_release(arm_ctrl, arm_ik, velocity_limit=0.8, waypoint_timeout=12.0):
    if not hasattr(arm_ik, "get_current_ee_poses"):
        logger_mp.warning("Exit retreat skipped: arm IK does not expose current EE poses.")
        return False

    logger_mp.info("Exit retreat: moving wrists outward before releasing arm SDK.")
    current_q = arm_ctrl.get_current_dual_arm_q()
    left_pose, right_pose = arm_ik.get_current_ee_poses(current_q)
    waypoints = make_exit_retreat_waypoints(left_pose, right_pose)

    completed_any = False
    for idx, (left_target, right_target) in enumerate(waypoints, start=1):
        current_q = arm_ctrl.get_current_dual_arm_q()
        current_dq = arm_ctrl.get_current_dual_arm_dq()
        sol_q, sol_tauff = arm_ik.solve_ik(left_target, right_target, current_q, current_dq)
        if sol_q is None or not np.all(np.isfinite(sol_q)):
            logger_mp.warning(f"Exit retreat waypoint {idx} skipped: IK target is invalid.")
            continue
        reached = move_dual_arm_to_q_slow(arm_ctrl, sol_q, sol_tauff, velocity_limit=velocity_limit, timeout=waypoint_timeout)
        completed_any = completed_any or reached
        if reached:
            logger_mp.info(f"Exit retreat waypoint {idx} reached.")
        else:
            logger_mp.warning(f"Exit retreat waypoint {idx} timed out; continuing with current arm pose.")
            break

    current_q = arm_ctrl.get_current_dual_arm_q()
    arm_ctrl.ctrl_dual_arm(current_q, np.zeros_like(current_q))
    time.sleep(0.3)
    return completed_any

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # basic control parameters
    parser.add_argument('--frequency', type = float, default = 30.0, help = 'control and record \'s frequency')
    parser.add_argument('--input-mode', type=str, choices=['hand', 'controller'], default='hand', help='Select XR device input tracking source')
    parser.add_argument('--display-mode', type=str, choices=['immersive', 'ego', 'pass-through'], default='immersive', help='Select XR device display mode')
    parser.add_argument('--arm', type=str, choices=['G1_29', 'G1_23', 'H1_2', 'H1', 'H2'], default='G1_29', help='Select arm controller')
    parser.add_argument('--ee', type=str, choices=['dex1', 'dex3', 'inspire_ftp', 'inspire_dfx', 'brainco'], help='Select end effector controller')
    parser.add_argument('--img-server-ip', type=str, default='192.168.123.164', help='IP address of image server, used by teleimager and televuer')
    parser.add_argument('--network-interface', type=str, default=None, help='Network interface for dds communication, e.g., eth0, wlan0. If None, use default interface.')
    # mode flags
    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')
    parser.add_argument('--upper-body-only', action='store_true',
                        help='In motion mode, do not send loco commands; keep lower body under the robot motion controller.')
    parser.add_argument('--absolute-wrist-pose', action='store_true',
                        help='Use raw absolute XR wrist poses instead of anchoring teleop to the robot pose at start.')
    parser.add_argument('--disable-hand-safety-sequence', action='store_true',
                        help='Disable the hand protection sequence: defer hand control until pre-teleop open, then exit close.')
    parser.add_argument('--hand-safety-settle', type=float, default=0.8,
                        help='Seconds to wait after hand open/close safety commands.')
    parser.add_argument('--arm-safety-velocity', type=float, default=0.8,
                        help='Arm joint velocity limit used by the pre-teleop and exit safety pose.')
    parser.add_argument('--arm-safety-timeout', type=float, default=30.0,
                        help='Seconds to wait for the arm safety pose.')
    parser.add_argument('--exit-arm-velocity', type=float, default=0.8,
                        help='Arm joint velocity limit used by the slow exit/retreat sequence.')
    parser.add_argument('--exit-initial-hold', type=float, default=0.8,
                        help='Seconds to hold the current arm pose immediately after exit is requested.')
    parser.add_argument('--exit-safety-hold', type=float, default=1.0,
                        help='Seconds to hold the arm safety pose before closing hands.')
    parser.add_argument('--exit-lower-velocity', type=float, default=0.35,
                        help='Arm joint velocity limit used when lowering arms before releasing SDK control.')
    parser.add_argument('--exit-lower-timeout', type=float, default=40.0,
                        help='Seconds to wait when lowering arms before releasing SDK control.')
    parser.add_argument('--exit-hold-before-release', type=float, default=1.0,
                        help='Seconds to hold the final exit arm pose before releasing SDK control.')
    parser.add_argument('--exit-release-duration', type=float, default=12.0,
                        help='Seconds used to release arm SDK control back to the robot motion controller.')
    parser.add_argument('--disable-exit-lower', action='store_true',
                        help='Disable controlled arm lowering before releasing SDK control in motion mode.')
    parser.add_argument('--disable-exit-retreat', action='store_true',
                        help='Disable the outward wrist retreat before releasing arm SDK in motion mode.')
    parser.add_argument('--headless', action='store_true', help='Enable headless mode (no display)')
    parser.add_argument('--sim', action = 'store_true', help = 'Enable isaac simulation mode')
    parser.add_argument('--ipc', action = 'store_true', help = 'Enable IPC server to handle input; otherwise enable sshkeyboard')
    parser.add_argument('--affinity', action = 'store_true', help = 'Enable high priority and set CPU affinity mode')
    # record mode and task info
    parser.add_argument('--record', action = 'store_true', help = 'Enable data recording mode')
    parser.add_argument('--task-dir', type = str, default = './utils/data/', help = 'path to save data')
    parser.add_argument('--task-name', type = str, default = 'pick cube', help = 'task file name for recording')
    parser.add_argument('--task-goal', type = str, default = 'pick up cube.', help = 'task goal for recording at json file')
    parser.add_argument('--task-desc', type = str, default = 'task description', help = 'task description for recording at json file')
    parser.add_argument('--task-steps', type = str, default = 'step1: do this; step2: do that;', help = 'task steps for recording at json file')

    args = parser.parse_args()
    logger_mp.debug(f"args: {args}")

    try:
        # setup dds communication domains id
        if args.sim:
            ChannelFactoryInitialize(1, networkInterface=args.network_interface)
        else:
            ChannelFactoryInitialize(0, networkInterface=args.network_interface)

        # ipc communication mode. client usage: see utils/ipc.py
        if args.ipc:
            ipc_server = IPC_Server(on_press=on_press,get_state=get_state)
            ipc_server.start()
        # sshkeyboard communication mode
        else:
            listen_keyboard_thread = threading.Thread(target=listen_keyboard, 
                                                      kwargs={"on_press": on_press, "until": None, "sequential": False,}, 
                                                      daemon=True)
            listen_keyboard_thread.start()

        # image client
        img_client = ImageClient(host=args.img_server_ip, request_bgr=True)
        camera_config = img_client.get_cam_config()
        logger_mp.debug(f"Camera config: {camera_config}")
        xr_need_local_img = not (args.display_mode == 'pass-through' or camera_config['head_camera']['enable_webrtc'])

        # televuer_wrapper: obtain hand pose data from the XR device and transmit the robot's head camera image to the XR device.
        tv_wrapper = TeleVuerWrapper(use_hand_tracking=args.input_mode == "hand", 
                                     binocular=camera_config['head_camera']['binocular'],
                                     img_shape=camera_config['head_camera']['image_shape'],
                                     # maybe should decrease fps for better performance?
                                     # https://github.com/unitreerobotics/xr_teleoperate/issues/172
                                     # display_fps=camera_config['head_camera']['fps'] ? args.frequency? 30.0?
                                     display_mode=args.display_mode,
                                     zmq=camera_config['head_camera']['enable_zmq'],
                                     webrtc=camera_config['head_camera']['enable_webrtc'],
                                     webrtc_url=f"https://{args.img_server_ip}:{camera_config['head_camera']['webrtc_port']}/offer",
                                     )
        
        # motion mode (G1: Regular mode R1+X, not Running mode R2+A)
        if args.motion:
            if args.input_mode == "controller" and not args.upper_body_only:
                loco_wrapper = LocoClientWrapper()
        else:
            motion_switcher = MotionSwitcher()
            status, result = motion_switcher.Enter_Debug_Mode()
            logger_mp.info(f"Enter debug mode: {'Success' if status == 0 else 'Failed'}")

        # arm
        if args.arm == "G1_29":
            arm_ik = G1_29_ArmIK()
            arm_ctrl = G1_29_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "G1_23":
            arm_ik = G1_23_ArmIK()
            arm_ctrl = G1_23_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "H1_2":
            arm_ik = H1_2_ArmIK()
            arm_ctrl = H1_2_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "H1":
            arm_ik = H1_ArmIK()
            arm_ctrl = H1_ArmController(simulation_mode=args.sim)
        elif args.arm == "H2":
            arm_ik = H2_ArmIK()
            arm_ctrl = H2_ArmController(motion_mode=args.motion, simulation_mode=args.sim)

        # end-effector
        left_binary_grasp_value = None
        right_binary_grasp_value = None
        binary_grasp_label = None
        deferred_hand_controller = None
        if args.ee == "dex3":
            from teleop.robot_control.robot_hand_unitree import Dex3_1_Controller
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 14, lock = False)   # [output] current left, right hand state(14) data.
            dual_hand_action_array = Array('d', 14, lock = False)  # [output] current left, right hand action(14) data.
            if args.input_mode == "controller":
                left_dex3_grasp_value = Value('d', 0.0, lock=True)   # [input] 0=open, 1=close
                right_dex3_grasp_value = Value('d', 0.0, lock=True)  # [input] 0=open, 1=close
                left_binary_grasp_value = left_dex3_grasp_value
                right_binary_grasp_value = right_dex3_grasp_value
                binary_grasp_label = "Dex3"
                if args.disable_hand_safety_sequence:
                    hand_ctrl = Dex3_1_Controller(left_dex3_grasp_value, right_dex3_grasp_value, dual_hand_data_lock,
                                                  dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, control_mode="binary")
                else:
                    deferred_hand_controller = lambda: Dex3_1_Controller(
                        left_dex3_grasp_value, right_dex3_grasp_value, dual_hand_data_lock,
                        dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, control_mode="binary")
            else:
                left_hand_pos_array = Array('d', 75, lock = True)      # [input]
                right_hand_pos_array = Array('d', 75, lock = True)     # [input]
                hand_ctrl = Dex3_1_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock,
                                              dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        elif args.ee == "dex1":
            from teleop.robot_control.robot_hand_unitree import Dex1_1_Gripper_Controller
            left_gripper_value = Value('d', 0.0, lock=True)        # [input]
            right_gripper_value = Value('d', 0.0, lock=True)       # [input]
            dual_gripper_data_lock = Lock()
            dual_gripper_state_array = Array('d', 2, lock=False)   # current left, right gripper state(2) data.
            dual_gripper_action_array = Array('d', 2, lock=False)  # current left, right gripper action(2) data.
            gripper_ctrl = Dex1_1_Gripper_Controller(left_gripper_value, right_gripper_value, dual_gripper_data_lock, 
                                                     dual_gripper_state_array, dual_gripper_action_array, simulation_mode=args.sim)
        elif args.ee == "inspire_dfx":
            from teleop.robot_control.robot_hand_inspire import Inspire_Controller_DFX
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            if args.input_mode == "controller":
                left_inspire_grasp_value = Value('d', 0.0, lock=True)   # [input] 0=open, 1=close
                right_inspire_grasp_value = Value('d', 0.0, lock=True)  # [input] 0=open, 1=close
                left_binary_grasp_value = left_inspire_grasp_value
                right_binary_grasp_value = right_inspire_grasp_value
                binary_grasp_label = "Inspire"
                if args.disable_hand_safety_sequence:
                    hand_ctrl = Inspire_Controller_DFX(left_inspire_grasp_value, right_inspire_grasp_value,
                                                       dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array,
                                                       simulation_mode=args.sim, control_mode="binary")
                else:
                    deferred_hand_controller = lambda: Inspire_Controller_DFX(
                        left_inspire_grasp_value, right_inspire_grasp_value,
                        dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array,
                        simulation_mode=args.sim, control_mode="binary")
            else:
                left_hand_pos_array = Array('d', 75, lock = True)      # [input]
                right_hand_pos_array = Array('d', 75, lock = True)     # [input]
                hand_ctrl = Inspire_Controller_DFX(left_hand_pos_array, right_hand_pos_array,
                                                   dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array,
                                                   simulation_mode=args.sim)
        elif args.ee == "inspire_ftp":
            from teleop.robot_control.robot_hand_inspire import Inspire_Controller_FTP
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            if args.input_mode == "controller":
                left_inspire_grasp_value = Value('d', 0.0, lock=True)   # [input] 0=open, 1=close
                right_inspire_grasp_value = Value('d', 0.0, lock=True)  # [input] 0=open, 1=close
                left_binary_grasp_value = left_inspire_grasp_value
                right_binary_grasp_value = right_inspire_grasp_value
                binary_grasp_label = "Inspire"
                if args.disable_hand_safety_sequence:
                    hand_ctrl = Inspire_Controller_FTP(left_inspire_grasp_value, right_inspire_grasp_value,
                                                       dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array,
                                                       simulation_mode=args.sim, control_mode="binary")
                else:
                    deferred_hand_controller = lambda: Inspire_Controller_FTP(
                        left_inspire_grasp_value, right_inspire_grasp_value,
                        dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array,
                        simulation_mode=args.sim, control_mode="binary")
            else:
                left_hand_pos_array = Array('d', 75, lock = True)      # [input]
                right_hand_pos_array = Array('d', 75, lock = True)     # [input]
                hand_ctrl = Inspire_Controller_FTP(left_hand_pos_array, right_hand_pos_array,
                                                   dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array,
                                                   simulation_mode=args.sim)
        elif args.ee == "brainco":
            from teleop.robot_control.robot_hand_brainco import Brainco_Controller
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Brainco_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, 
                                           dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        else:
            pass
        
        # affinity mode (if you dont know what it is, then you probably don't need it)
        if args.affinity:
            import psutil
            p = psutil.Process(os.getpid())
            p.cpu_affinity([0,1,2,3]) # Set CPU affinity to cores 0-3
            try:
                p.nice(-20)           # Set highest priority
                logger_mp.info("Set high priority successfully.")
            except psutil.AccessDenied:
                logger_mp.warning("Failed to set high priority. Please run as root.")
                
            for child in p.children(recursive=True):
                try:
                    logger_mp.info(f"Child process {child.pid} name: {child.name()}")
                    child.cpu_affinity([5,6])
                    child.nice(-20)
                except psutil.AccessDenied:
                    pass

        # simulation mode
        if args.sim:
            reset_pose_publisher = ChannelPublisher("rt/reset_pose/cmd", String_)
            reset_pose_publisher.Init()
            from teleop.utils.sim_state_topic import start_sim_state_subscribe
            sim_state_subscriber = start_sim_state_subscribe()

        # record + headless / non-headless mode
        if args.record:
            recorder = EpisodeWriter(task_dir = os.path.join(args.task_dir, args.task_name),
                                     task_goal = args.task_goal,
                                     task_desc = args.task_desc,
                                     task_steps = args.task_steps,
                                     frequency = args.frequency, 
                                     rerun_log = not args.headless)

        logger_mp.info("----------------------------------------------------------------")
        logger_mp.info("🟢  Press [r] to start syncing the robot with your movements.")
        if args.record:
            logger_mp.info("🟡  Press [s] to START or SAVE recording (toggle cycle).")
        else:
            logger_mp.info("🔵  Recording is DISABLED (run with --record to enable).")
        logger_mp.info("🔴  Press [q] to stop and exit the program.")
        logger_mp.info("⚠️  IMPORTANT: Please keep your distance and stay safe.")
        READY = True                  # now ready to (1) enter START state
        while not START and not STOP: # wait for start or stop signal.
            time.sleep(0.033)
            if camera_config['head_camera']['enable_zmq'] and xr_need_local_img:
                head_img = img_client.get_head_frame()
                if head_img is not None and head_img.bgr is not None:
                    tv_wrapper.render_to_xr(head_img.bgr)

        if STOP:
            logger_mp.info("Stop requested before tracking start.")
            raise KeyboardInterrupt

        if not args.disable_hand_safety_sequence:
            logger_mp.info("Pre-teleop safety sequence: raise arms to safety pose before opening hands.")
            move_dual_arm_to_safety_pose(
                arm_ctrl,
                velocity_limit=args.arm_safety_velocity,
                timeout=args.arm_safety_timeout,
            )
            if deferred_hand_controller is not None:
                logger_mp.info("Starting hand controller after arms reached the safety pose.")
                hand_ctrl = deferred_hand_controller()
            set_binary_grasp_values(
                left_binary_grasp_value,
                right_binary_grasp_value,
                close=False,
                label=binary_grasp_label or "hand",
                settle_time=args.hand_safety_settle,
            )

        logger_mp.info("---------------------🚀start Tracking🚀-------------------------")
        arm_ctrl.speed_gradual_max()

        head_img = None
        left_wrist_img = None
        right_wrist_img = None
        use_relative_controller_wrist = args.input_mode == "controller" and not args.absolute_wrist_pose
        controller_anchor_ready = not use_relative_controller_wrist
        controller_anchor_samples = 0
        controller_anchor_sample_target = 5
        controller_anchor_warn_time = 0.0
        left_xr_start_pose = None
        right_xr_start_pose = None
        left_robot_start_pose = None
        right_robot_start_pose = None
        last_binary_left_trigger_close = None
        last_binary_right_trigger_close = None

        # main loop. robot start to follow VR user's motion
        while not STOP:
            start_time = time.time()
            loco_action = []
            # get image
            if camera_config['head_camera']['enable_zmq']:
                if args.record or xr_need_local_img:
                    head_img = img_client.get_head_frame()
                if xr_need_local_img and head_img is not None and head_img.bgr is not None:
                    tv_wrapper.render_to_xr(head_img.bgr)
            if camera_config['left_wrist_camera']['enable_zmq']:
                if args.record:
                    left_wrist_img = img_client.get_left_wrist_frame()
            if camera_config['right_wrist_camera']['enable_zmq']:
                if args.record:
                    right_wrist_img = img_client.get_right_wrist_frame()

            # record mode
            if args.record and RECORD_TOGGLE:
                RECORD_TOGGLE = False
                if not RECORD_RUNNING:
                    if recorder.create_episode():
                        RECORD_RUNNING = True
                    else:
                        logger_mp.error("Failed to create episode. Recording not started.")
                else:
                    RECORD_RUNNING = False
                    recorder.save_episode()
                    if args.sim:
                        publish_reset_category(1, reset_pose_publisher)

            # get xr's tele data
            tele_data = tv_wrapper.get_tele_data()
            if (args.ee == "dex3" or args.ee == "inspire_dfx" or args.ee == "inspire_ftp" or args.ee == "brainco") and args.input_mode == "hand":
                with left_hand_pos_array.get_lock():
                    left_hand_pos_array[:] = tele_data.left_hand_pos.flatten()
                with right_hand_pos_array.get_lock():
                    right_hand_pos_array[:] = tele_data.right_hand_pos.flatten()
            elif args.ee in ("dex3", "inspire_dfx", "inspire_ftp") and args.input_mode == "controller":
                # TeleVuerWrapper maps controller triggerValue to 10.0=open, 0.0=fully pressed.
                left_trigger_close = tele_data.left_ctrl_triggerValue < 5.0
                right_trigger_close = tele_data.right_ctrl_triggerValue < 5.0
                if (left_trigger_close != last_binary_left_trigger_close or
                        right_trigger_close != last_binary_right_trigger_close):
                    controller_label = "Dex3" if args.ee == "dex3" else "Inspire"
                    logger_mp.info(
                        f"[{controller_label} controller] trigger L={tele_data.left_ctrl_triggerValue:.2f} "
                        f"({'close' if left_trigger_close else 'open'}), "
                        f"R={tele_data.right_ctrl_triggerValue:.2f} "
                        f"({'close' if right_trigger_close else 'open'})"
                    )
                    last_binary_left_trigger_close = left_trigger_close
                    last_binary_right_trigger_close = right_trigger_close
                if args.ee == "dex3":
                    with left_dex3_grasp_value.get_lock():
                        left_dex3_grasp_value.value = 1.0 if left_trigger_close else 0.0
                    with right_dex3_grasp_value.get_lock():
                        right_dex3_grasp_value.value = 1.0 if right_trigger_close else 0.0
                else:
                    with left_inspire_grasp_value.get_lock():
                        left_inspire_grasp_value.value = 1.0 if left_trigger_close else 0.0
                    with right_inspire_grasp_value.get_lock():
                        right_inspire_grasp_value.value = 1.0 if right_trigger_close else 0.0
            elif args.ee == "dex1" and args.input_mode == "controller":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_ctrl_triggerValue
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_ctrl_triggerValue
            elif args.ee == "dex1" and args.input_mode == "hand":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_hand_pinchValue
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_hand_pinchValue
            else:
                pass
            
            # high level control
            if args.input_mode == "controller" and args.motion and not args.upper_body_only:
                # quit teleoperate
                if tele_data.right_ctrl_aButton:
                    START = False
                    STOP = True
                # command robot to enter damping mode. soft emergency stop function
                if tele_data.left_ctrl_thumbstick and tele_data.right_ctrl_thumbstick:
                    loco_wrapper.Damp()
                # https://github.com/unitreerobotics/xr_teleoperate/issues/135, control, limit velocity to within 0.3
                loco_action = [
                    -tele_data.left_ctrl_thumbstickValue[1] * 0.3,
                    -tele_data.left_ctrl_thumbstickValue[0] * 0.3,
                    -tele_data.right_ctrl_thumbstickValue[0] * 0.3,
                ]
                loco_wrapper.Move(*loco_action)

            # get current robot state data.
            current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
            current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()

            left_wrist_pose = tele_data.left_wrist_pose
            right_wrist_pose = tele_data.right_wrist_pose
            if use_relative_controller_wrist:
                controller_pose_valid = tele_data.left_wrist_valid and tele_data.right_wrist_valid
                if not controller_pose_valid:
                    if time.time() - controller_anchor_warn_time > 1.0:
                        logger_mp.warning("Waiting for valid controller wrist poses before arm IK...")
                        controller_anchor_warn_time = time.time()
                    arm_ctrl.ctrl_dual_arm(current_lr_arm_q, np.zeros_like(current_lr_arm_q))
                    continue

                if not controller_anchor_ready:
                    controller_anchor_samples += 1
                    arm_ctrl.ctrl_dual_arm(current_lr_arm_q, np.zeros_like(current_lr_arm_q))
                    if controller_anchor_samples < controller_anchor_sample_target:
                        continue

                    if hasattr(arm_ik, "get_current_ee_poses"):
                        left_robot_start_pose, right_robot_start_pose = arm_ik.get_current_ee_poses(current_lr_arm_q)
                        left_xr_start_pose = tele_data.left_wrist_pose.copy()
                        right_xr_start_pose = tele_data.right_wrist_pose.copy()
                        controller_anchor_ready = True
                        logger_mp.info("Controller teleop start pose anchored to current robot wrists.")
                    else:
                        controller_anchor_ready = True
                        logger_mp.warning("Arm IK does not expose current EE poses; using absolute XR wrist poses.")

                if left_xr_start_pose is not None:
                    left_wrist_pose = align_wrist_pose_to_start(tele_data.left_wrist_pose, left_xr_start_pose, left_robot_start_pose)
                    right_wrist_pose = align_wrist_pose_to_start(tele_data.right_wrist_pose, right_xr_start_pose, right_robot_start_pose)

            # solve ik using motor data and wrist pose, then use ik results to control arms.
            time_ik_start = time.time()
            sol_q, sol_tauff  = arm_ik.solve_ik(left_wrist_pose, right_wrist_pose, current_lr_arm_q, current_lr_arm_dq)
            time_ik_end = time.time()
            logger_mp.debug(f"ik:\t{round(time_ik_end - time_ik_start, 6)}")
            arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)

            # record data
            if args.record:
                READY = recorder.is_ready() # now ready to (2) enter RECORD_RUNNING state
                # dex hand or gripper
                if args.ee == "dex3" and args.input_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:7]
                        right_ee_state = dual_hand_state_array[-7:]
                        left_hand_action = dual_hand_action_array[:7]
                        right_hand_action = dual_hand_action_array[-7:]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex3" and args.input_mode == "controller":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:7]
                        right_ee_state = dual_hand_state_array[-7:]
                        left_hand_action = dual_hand_action_array[:7]
                        right_hand_action = dual_hand_action_array[-7:]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex1" and args.input_mode == "hand":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[1]]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex1" and args.input_mode == "controller":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[1]]
                        current_body_state = arm_ctrl.get_current_motor_q().tolist()
                        current_body_action = [-tele_data.left_ctrl_thumbstickValue[1]  * 0.3,
                                               -tele_data.left_ctrl_thumbstickValue[0]  * 0.3,
                                               -tele_data.right_ctrl_thumbstickValue[0] * 0.3]
                elif (args.ee == "inspire_dfx" or args.ee == "inspire_ftp" or args.ee == "brainco") and args.input_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:6]
                        right_ee_state = dual_hand_state_array[-6:]
                        left_hand_action = dual_hand_action_array[:6]
                        right_hand_action = dual_hand_action_array[-6:]
                        current_body_state = []
                        current_body_action = []
                elif (args.ee == "inspire_dfx" or args.ee == "inspire_ftp") and args.input_mode == "controller":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:6]
                        right_ee_state = dual_hand_state_array[-6:]
                        left_hand_action = dual_hand_action_array[:6]
                        right_hand_action = dual_hand_action_array[-6:]
                        current_body_state = []
                        current_body_action = []
                else:
                    left_ee_state = []
                    right_ee_state = []
                    left_hand_action = []
                    right_hand_action = []
                    current_body_state = []
                    current_body_action = []

                if args.input_mode == "controller" and args.motion and not args.upper_body_only:
                    current_body_state = arm_ctrl.get_current_motor_q().tolist()
                    current_body_action = loco_action

                # arm state and action
                left_arm_state  = current_lr_arm_q[:7]
                right_arm_state = current_lr_arm_q[-7:]
                left_arm_action = sol_q[:7]
                right_arm_action = sol_q[-7:]
                if RECORD_RUNNING:
                    colors = {}
                    depths = {}
                    if camera_config['head_camera']['binocular']:
                        if head_img is not None:
                            colors[f"color_{0}"] = head_img.bgr[:, :camera_config['head_camera']['image_shape'][1]//2]
                            colors[f"color_{1}"] = head_img.bgr[:, camera_config['head_camera']['image_shape'][1]//2:]
                        else:
                            logger_mp.warning("Head image is None!")
                        if camera_config['left_wrist_camera']['enable_zmq']:
                            if left_wrist_img is not None:
                                colors[f"color_{2}"] = left_wrist_img.bgr
                            else:
                                logger_mp.warning("Left wrist image is None!")
                        if camera_config['right_wrist_camera']['enable_zmq']:
                            if right_wrist_img is not None:
                                colors[f"color_{3}"] = right_wrist_img.bgr
                            else:
                                logger_mp.warning("Right wrist image is None!")
                    else:
                        if head_img is not None:
                            colors[f"color_{0}"] = head_img.bgr
                        else:
                            logger_mp.warning("Head image is None!")
                        if camera_config['left_wrist_camera']['enable_zmq']:
                            if left_wrist_img is not None:
                                colors[f"color_{1}"] = left_wrist_img.bgr
                            else:
                                logger_mp.warning("Left wrist image is None!")
                        if camera_config['right_wrist_camera']['enable_zmq']:
                            if right_wrist_img is not None:
                                colors[f"color_{2}"] = right_wrist_img.bgr
                            else:
                                logger_mp.warning("Right wrist image is None!")
                    states = {
                        "left_arm": {                                                                    
                            "qpos":   left_arm_state.tolist(),    # numpy.array -> list
                            "qvel":   [],                          
                            "torque": [],                        
                        }, 
                        "right_arm": {                                                                    
                            "qpos":   right_arm_state.tolist(),       
                            "qvel":   [],                          
                            "torque": [],                         
                        },                        
                        "left_ee": {                                                                    
                            "qpos":   left_ee_state,           
                            "qvel":   [],                           
                            "torque": [],                          
                        }, 
                        "right_ee": {                                                                    
                            "qpos":   right_ee_state,       
                            "qvel":   [],                           
                            "torque": [],  
                        }, 
                        "body": {
                            "qpos": current_body_state,
                        }, 
                    }
                    actions = {
                        "left_arm": {                                   
                            "qpos":   left_arm_action.tolist(),       
                            "qvel":   [],       
                            "torque": [],      
                        }, 
                        "right_arm": {                                   
                            "qpos":   right_arm_action.tolist(),       
                            "qvel":   [],       
                            "torque": [],       
                        },                         
                        "left_ee": {                                   
                            "qpos":   left_hand_action,       
                            "qvel":   [],       
                            "torque": [],       
                        }, 
                        "right_ee": {                                   
                            "qpos":   right_hand_action,       
                            "qvel":   [],       
                            "torque": [], 
                        }, 
                        "body": {
                            "qpos": current_body_action,
                        }, 
                    }
                    if args.sim:
                        sim_state = sim_state_subscriber.read_data()            
                        recorder.add_item(colors=colors, depths=depths, states=states, actions=actions, sim_state=sim_state)
                    else:
                        recorder.add_item(colors=colors, depths=depths, states=states, actions=actions)

            current_time = time.time()
            time_elapsed = current_time - start_time
            sleep_time = max(0, (1 / args.frequency) - time_elapsed)
            time.sleep(sleep_time)
            logger_mp.debug(f"main process sleep: {sleep_time}")

    except KeyboardInterrupt:
        logger_mp.info("⛔ KeyboardInterrupt, exiting program...")
    except Exception:
        import traceback
        logger_mp.error(traceback.format_exc())
    finally:
        exit_lower_completed = False
        try:
            if "arm_ctrl" in locals():
                logger_mp.info("Exit safety sequence: hold current pose before any return motion.")
                hold_current_arm_pose(
                    arm_ctrl,
                    hold_time=args.exit_initial_hold,
                    velocity_limit=0.2,
                    label="current teleop arm pose",
                    restore_previous=False,
                )
                logger_mp.info("Exit safety sequence: move arms to safety pose before closing hands.")
                move_dual_arm_to_safety_pose(
                    arm_ctrl,
                    velocity_limit=args.arm_safety_velocity,
                    timeout=args.arm_safety_timeout,
                    hold_time=args.exit_safety_hold,
                    restore_previous=False,
                )
            if not args.disable_hand_safety_sequence:
                set_binary_grasp_values(
                    left_binary_grasp_value if "left_binary_grasp_value" in locals() else None,
                    right_binary_grasp_value if "right_binary_grasp_value" in locals() else None,
                    close=True,
                    label=(binary_grasp_label if "binary_grasp_label" in locals() else None) or "hand",
                    settle_time=args.hand_safety_settle,
                )
            if "arm_ctrl" in locals():
                if args.motion and not args.disable_exit_lower:
                    exit_lower_completed = move_dual_arm_to_startup_pose(
                        arm_ctrl,
                        velocity_limit=args.exit_lower_velocity,
                        timeout=args.exit_lower_timeout,
                        hold_time=args.exit_hold_before_release,
                    )
                else:
                    hold_current_arm_pose(
                        arm_ctrl,
                        hold_time=args.exit_hold_before_release,
                        velocity_limit=0.2,
                        label="exit pose",
                        restore_previous=False,
                    )
        except TypeError:
            if "arm_ctrl" in locals():
                arm_ctrl.ctrl_dual_arm_go_home()
        except Exception as e:
            logger_mp.error(f"Failed to run arm exit motion: {e}")

        try:
            if args.motion and "arm_ctrl" in locals() and hasattr(arm_ctrl, "release_arm_sdk_mode"):
                if not args.disable_exit_lower and not exit_lower_completed:
                    logger_mp.warning("Releasing arm SDK after controlled lowering did not fully complete; release will stay slow.")
                arm_ctrl.release_arm_sdk_mode(duration=args.exit_release_duration)
        except Exception as e:
            logger_mp.error(f"Failed to release arm sdk mode: {e}")

        try:
            if not args.sim:
                if "motion_switcher" not in locals():
                    motion_switcher = MotionSwitcher()
                status, result = motion_switcher.Exit_Debug_Mode()
                logger_mp.info(f"Switch to AI/remote mode: {'Success' if status == 0 else 'Failed'}")
        except Exception as e:
            logger_mp.error(f"Failed to switch to AI/remote mode: {e}")
        
        try:
            if args.ipc:
                ipc_server.stop()
            else:
                stop_listening()
                listen_keyboard_thread.join()
        except Exception as e:
            logger_mp.error(f"Failed to stop keyboard listener or ipc server: {e}")
        
        try:
            if img_client is not None:
                img_client.close()
        except Exception as e:
            logger_mp.error(f"Failed to close image client: {e}")

        try:
            tv_wrapper.close()
        except Exception as e:
            logger_mp.error(f"Failed to close televuer wrapper: {e}")

        try:
            if args.sim:
                sim_state_subscriber.stop_subscribe()
        except Exception as e:
            logger_mp.error(f"Failed to stop sim state subscriber: {e}")
        
        try:
            if args.record:
                recorder.close()
        except Exception as e:
            logger_mp.error(f"Failed to close recorder: {e}")
        logger_mp.info("✅ Finally, exiting program.")
        exit(0)
