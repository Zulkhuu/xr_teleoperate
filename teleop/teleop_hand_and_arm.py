import time
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
from teleop.utils.teleop_recording import build_recording_payload
from teleop.teleop_config import parse_args
from teleop.robot_control.end_effector import create_end_effector
from teleop.xr_connection import XRSession
from teleop.teleop_lifecycle import run_startup_sequence, run_shutdown_sequence
from teleop.utils.pose_utils import align_wrist_pose_to_start
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

def main(argv=None):
    global START, STOP, READY, RECORD_RUNNING, RECORD_TOGGLE
    START = STOP = READY = RECORD_RUNNING = RECORD_TOGGLE = False
    args = parse_args(argv)
    arm_ctrl = motion_switcher = None
    img_client = recorder = sim_state_subscriber = None
    ipc_server = listen_keyboard_thread = None
    xr = XRSession(args)
    loco_wrapper = None
    robot_initialized = False
    end_effector = None
    logger_mp.debug(f"args: {args}")

    try:
        xr.start()

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

        # All Vuer transports receive camera configuration on the host.
        img_client = ImageClient(host=args.img_server_ip, request_bgr=True)
        camera_config = img_client.get_cam_config()
        xr_need_local_img = xr.configure_display(camera_config)

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

        robot_initialized = True
        end_effector = create_end_effector(args)

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
            xr.check()
            time.sleep(0.033)
            if camera_config['head_camera']['enable_zmq'] and xr_need_local_img:
                head_img = img_client.get_head_frame()
                if head_img is not None and head_img.bgr is not None:
                    xr.render_to_xr(head_img.bgr)

        if STOP:
            logger_mp.info("Stop requested before tracking start.")
            raise KeyboardInterrupt

        def check_startup():
            if STOP:
                raise KeyboardInterrupt
            xr.read()

        check_startup()
        run_startup_sequence(args, arm_ctrl, end_effector=end_effector,
                             check_cancel=check_startup)

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

        # main loop. robot start to follow VR user's motion
        while not STOP:
            start_time = time.time()
            xr.check()
            loco_action = []
            # get image
            if camera_config['head_camera']['enable_zmq']:
                if args.record or xr_need_local_img:
                    head_img = img_client.get_head_frame()
                if xr_need_local_img and head_img is not None and head_img.bgr is not None:
                    xr.render_to_xr(head_img.bgr)
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
            tele_data = xr.read()
            end_effector.update(tele_data)

            # high level control
            if args.input_mode == "controller" and args.motion and not args.upper_body_only:
                # quit teleoperate
                if tele_data.right_ctrl_aButton:
                    START = False
                    STOP = True
                    break
                # command robot to enter damping mode. soft emergency stop function
                if tele_data.left_ctrl_thumbstick and tele_data.right_ctrl_thumbstick:
                    loco_wrapper.Enter_Damp_Mode()
                    START = False
                    STOP = True
                    break
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
                    time.sleep(1 / args.frequency)
                    continue

                if not controller_anchor_ready:
                    controller_anchor_samples += 1
                    arm_ctrl.ctrl_dual_arm(current_lr_arm_q, np.zeros_like(current_lr_arm_q))
                    if controller_anchor_samples < controller_anchor_sample_target:
                        time.sleep(1 / args.frequency)
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
                ee_snapshot = end_effector.snapshot()
                current_body_state = []
                current_body_action = []
                # Preserve Dex1 controller recording even without locomotion enabled.
                if args.ee == 'dex1' and args.input_mode == 'controller':
                    current_body_state = arm_ctrl.get_current_motor_q().tolist()
                    current_body_action = [-tele_data.left_ctrl_thumbstickValue[1] * 0.3,
                                           -tele_data.left_ctrl_thumbstickValue[0] * 0.3,
                                           -tele_data.right_ctrl_thumbstickValue[0] * 0.3]

                if args.input_mode == "controller" and args.motion and not args.upper_body_only:
                    current_body_state = arm_ctrl.get_current_motor_q().tolist()
                    current_body_action = loco_action

                # arm state and action
                left_arm_state  = current_lr_arm_q[:7]
                right_arm_state = current_lr_arm_q[-7:]
                left_arm_action = sol_q[:7]
                right_arm_action = sol_q[-7:]
                if RECORD_RUNNING:
                    colors, depths, states, actions = build_recording_payload(
                        camera_config,
                        head_img,
                        left_wrist_img,
                        right_wrist_img,
                        left_arm_state,
                        right_arm_state,
                        left_arm_action,
                        right_arm_action,
                        ee_snapshot.left_state,
                        ee_snapshot.right_state,
                        ee_snapshot.left_action,
                        ee_snapshot.right_action,
                        current_body_state,
                        current_body_action,
                    )
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
        raise
    finally:
        if loco_wrapper is not None:
            try:
                loco_wrapper.Move(0.0, 0.0, 0.0)
            except Exception as exc:
                logger_mp.error(f'Failed to stop locomotion: {exc}')
        try:
            run_shutdown_sequence(
                args, arm_ctrl, motion_switcher,
                end_effector=end_effector, robot_initialized=robot_initialized,
            )
        except Exception as exc:
            logger_mp.error(f'Failed to complete robot shutdown: {exc}')

        try:
            if args.ipc and ipc_server is not None:
                ipc_server.stop()
            elif listen_keyboard_thread is not None:
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
            xr.close()
        except Exception as e:
            logger_mp.error(f"Failed to close televuer wrapper: {e}")

        try:
            if sim_state_subscriber is not None:
                sim_state_subscriber.stop_subscribe()
        except Exception as e:
            logger_mp.error(f"Failed to stop sim state subscriber: {e}")
        
        try:
            if recorder is not None:
                recorder.close()
        except Exception as e:
            logger_mp.error(f"Failed to close recorder: {e}")
        logger_mp.info("✅ Finally, exiting program.")



if __name__ == "__main__":
    main()
