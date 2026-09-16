import threading
import logging_mp
logging_mp.basicConfig(level=logging_mp.INFO)
logger_mp = logging_mp.getLogger(__name__)

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize # dds 
from teleop.teleop_config import parse_args
from teleop.robot_control.end_effector import create_end_effector
from teleop.xr_connection import XRSession
from teleop.teleop_lifecycle import run_startup_sequence, run_shutdown_sequence
from teleop.robot_control.robot_arm import G1_29_ArmController, G1_23_ArmController, H1_2_ArmController, H1_ArmController, H2_ArmController
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK, G1_23_ArmIK, H1_2_ArmIK, H1_ArmIK, H2_ArmIK
from teleimager.image_client import ImageClient
from teleop.utils.episode_writer import EpisodeWriter
from teleop.utils.ipc import IPC_Server
from teleop.utils.motion_switcher import MotionSwitcher, LocoClientWrapper
from sshkeyboard import listen_keyboard, stop_listening

# Keyboard/IPC commands use the same runtime actions as Quest input.
import queue

operator_commands = queue.SimpleQueue()
_runtime = None


def on_press(key):
    if key in ('b', 'r', 's', 'p', 'q'):
        operator_commands.put(key)


def get_state() -> dict:
    from teleop.utils.quest_control import TeleopState
    machine = _runtime.machine if _runtime is not None else None
    return {
        "START": machine is not None and machine.state == TeleopState.ACTIVE,
        "STOP": machine is not None and machine.state == TeleopState.EXITING,
        "READY": machine is not None and machine.state == TeleopState.ALIGNED,
        "RECORD_RUNNING": machine is not None and machine.recording,
        "teleop_state": machine.state.name if machine is not None else "INITIALIZING",
    }


def main(argv=None):
    global _runtime, operator_commands
    import queue
    operator_commands = queue.SimpleQueue()
    _runtime = None
    args = parse_args(argv)
    arm_ctrl = motion_switcher = None
    img_client = recorder = sim_state_subscriber = None
    ipc_server = listen_keyboard_thread = None
    xr = XRSession(args)
    loco_wrapper = damping_wrapper = None
    robot_initialized = False
    end_effector = None
    logger_mp.debug(f"args: {args}")

    try:
        if args.record and args.recording_backend == 'neuracore':
            from teleop.utils.neuracore_recorder import NeuracoreRecorder
            recorder = NeuracoreRecorder(args)
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
        if args.record and args.recording_backend == 'neuracore' and not camera_config['head_camera']['enable_zmq']:
            raise ValueError('Neuracore RGB recording requires head_camera.enable_zmq=true')
        xr_need_local_img = xr.configure_display(camera_config, hud=True)

        # motion mode (G1: Regular mode R1+X, not Running mode R2+A)
        if args.motion:
            if args.input_mode == "controller" and not args.upper_body_only:
                loco_wrapper = LocoClientWrapper()
        else:
            motion_switcher = MotionSwitcher()
            status, result = motion_switcher.Enter_Debug_Mode()
            logger_mp.info(f"Enter debug mode: {'Success' if status == 0 else 'Failed'}")
            if status != 0:
                raise RuntimeError('Cannot enable teleoperation: entering debug mode failed')

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
            from teleop.utils.sim_state_topic import start_sim_state_subscribe
            sim_state_subscriber = start_sim_state_subscribe()

        # record + headless / non-headless mode
        if args.record and args.recording_backend == 'episode':
            recorder = EpisodeWriter(task_dir = os.path.join(args.task_dir, args.task_name),
                                     task_goal = args.task_goal,
                                     task_desc = args.task_desc,
                                     task_steps = args.task_steps,
                                     frequency = args.frequency, 
                                     rerun_log = not args.headless)

        from teleop.teleop_runtime import TeleopRuntime
        logger_mp.info("Starting automatic arm/hand safety preparation; no Quest button is required.")
        run_startup_sequence(args, arm_ctrl, end_effector=end_effector)
        logger_mp.info("Initial arm/hand safety preparation complete.")
        logger_mp.info("Waiting for XR tracking, then release B to set the teleoperation anchor.")
        logger_mp.info("Quest: B release anchor; hold X (keep pressed 0.5 s) start; Y release record; A pause; hold A exit; L3+R3 damping.")
        logger_mp.info("Keyboard debugging: r prepare, b anchor, r start, s record, p pause, q exit.")
        if args.display_mode == 'pass-through':
            logger_mp.warning("Pass-through mode has no camera HUD.")
        _runtime = TeleopRuntime(
            args, xr, arm_ctrl, arm_ik, end_effector, img_client, camera_config,
            xr_need_local_img, recorder=recorder, loco=loco_wrapper,
            damping=damping_wrapper, keyboard=operator_commands,
            sim_state=sim_state_subscriber,
            prepared=True,
        )
        _runtime.ever_activated = True  # preserve the established graceful exit path after startup preparation
        _runtime.run()

    except KeyboardInterrupt:
        logger_mp.info("⛔ KeyboardInterrupt, exiting program...")
    except Exception:
        import traceback
        logger_mp.error(traceback.format_exc())
        raise
    finally:
        if loco_wrapper is not None and not (_runtime and _runtime.damping_latched):
            try:
                loco_wrapper.Move(0.0, 0.0, 0.0)
            except Exception as exc:
                logger_mp.error(f'Failed to stop locomotion: {exc}')
        try:
            # Never send a home/release trajectory after emergency damping or
            # before the operator deliberately requested the preparation motion.
            safe_shutdown = (_runtime is not None and _runtime.ever_activated
                             and _runtime.machine.state.name == 'EXITING'
                             and not _runtime.damping_latched)
            run_shutdown_sequence(
                args, arm_ctrl if safe_shutdown else None,
                motion_switcher if safe_shutdown else None,
                end_effector=end_effector if safe_shutdown else None,
                robot_initialized=robot_initialized if safe_shutdown else False,
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
