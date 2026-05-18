from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize # dds
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_                           # idl
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
import numpy as np
from enum import IntEnum
import threading
import time
from multiprocessing import Process, Array

import logging_mp
logger_mp = logging_mp.getLogger(__name__)

Inspire_Num_Motors = 6
kTopicInspireDFXCommand = "rt/inspire/cmd"
kTopicInspireDFXState = "rt/inspire/state"
INSPIRE_OPEN_TARGET = np.ones(Inspire_Num_Motors)
INSPIRE_CLOSE_TARGET = np.zeros(Inspire_Num_Motors)

class Inspire_Controller_DFX:
    def __init__(self, left_hand_array, right_hand_array, dual_hand_data_lock = None, dual_hand_state_array = None,
                       dual_hand_action_array = None, fps = 100.0, Unit_Test = False, simulation_mode = False,
                       control_mode = "hand", state_wait_timeout = 3.0):
        logger_mp.info("Initialize Inspire_Controller_DFX...")
        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        self.control_mode = control_mode
        self.hand_retargeting = None
        if self.control_mode == "hand":
            if not self.Unit_Test:
                self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)
            else:
                self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND_Unit_Test)


        # initialize handcmd publisher and handstate subscriber
        self.HandCmb_publisher = ChannelPublisher(kTopicInspireDFXCommand, MotorCmds_)
        self.HandCmb_publisher.Init()

        self.HandState_subscriber = ChannelSubscriber(kTopicInspireDFXState, MotorStates_)
        self.HandState_subscriber.Init()

        # Shared Arrays for hand states
        self.left_hand_state_array  = Array('d', Inspire_Num_Motors, lock=True)  
        self.right_hand_state_array = Array('d', Inspire_Num_Motors, lock=True)

        # initialize subscribe thread
        self.subscribe_state_thread = threading.Thread(target=self._subscribe_hand_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        wait_start_time = time.time()
        while True:
            if any(self.right_hand_state_array): # any(self.left_hand_state_array) and
                break
            time.sleep(0.01)
            logger_mp.warning("[Inspire_Controller_DFX] Waiting to subscribe dds...")
            if self.control_mode == "binary" and time.time() - wait_start_time > state_wait_timeout:
                logger_mp.warning("[Inspire_Controller_DFX] Inspire state DDS not ready; continuing in binary command-only mode.")
                break
        logger_mp.info("[Inspire_Controller_DFX] Subscribe dds ok.")

        hand_control_process = Process(target=self.control_process, args=(left_hand_array, right_hand_array,  self.left_hand_state_array, self.right_hand_state_array,
                                                                          dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array,
                                                                          self.control_mode))
        hand_control_process.daemon = True
        hand_control_process.start()

        logger_mp.info("Initialize Inspire_Controller_DFX OK!")

    def _subscribe_hand_state(self):
        while True:
            hand_msg  = self.HandState_subscriber.Read()
            if hand_msg is not None:
                for idx, id in enumerate(Inspire_Left_Hand_JointIndex):
                    self.left_hand_state_array[idx] = hand_msg.states[id].q
                for idx, id in enumerate(Inspire_Right_Hand_JointIndex):
                    self.right_hand_state_array[idx] = hand_msg.states[id].q
            time.sleep(0.002)

    def ctrl_dual_hand(self, left_q_target, right_q_target):
        """
        Set current left, right hand motor state target q
        """
        for idx, id in enumerate(Inspire_Left_Hand_JointIndex):             
            self.hand_msg.cmds[id].q = left_q_target[idx]         
        for idx, id in enumerate(Inspire_Right_Hand_JointIndex):             
            self.hand_msg.cmds[id].q = right_q_target[idx] 

        self.HandCmb_publisher.Write(self.hand_msg)
        # logger_mp.debug("hand ctrl publish ok.")
    
    def control_process(self, left_hand_array, right_hand_array, left_hand_state_array, right_hand_state_array,
                              dual_hand_data_lock = None, dual_hand_state_array = None, dual_hand_action_array = None,
                              control_mode = "hand"):
        self.running = True

        left_q_target  = np.full(Inspire_Num_Motors, 1.0)
        right_q_target = np.full(Inspire_Num_Motors, 1.0)

        # initialize inspire hand's cmd msg
        self.hand_msg  = MotorCmds_()
        self.hand_msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(len(Inspire_Right_Hand_JointIndex) + len(Inspire_Left_Hand_JointIndex))]

        for cmd in self.hand_msg.cmds:
            cmd.mode = 1
            cmd.q = 1.0

        try:
            while self.running:
                start_time = time.time()
                # Read left and right q_state from shared arrays
                state_data = np.concatenate((np.array(left_hand_state_array[:]), np.array(right_hand_state_array[:])))

                if control_mode == "binary":
                    with left_hand_array.get_lock():
                        left_close_value = np.clip(left_hand_array.value, 0.0, 1.0)
                    with right_hand_array.get_lock():
                        right_close_value = np.clip(right_hand_array.value, 0.0, 1.0)
                    left_q_target = INSPIRE_OPEN_TARGET * (1.0 - left_close_value) + INSPIRE_CLOSE_TARGET * left_close_value
                    right_q_target = INSPIRE_OPEN_TARGET * (1.0 - right_close_value) + INSPIRE_CLOSE_TARGET * right_close_value
                else:
                    # get dual hand state
                    with left_hand_array.get_lock():
                        left_hand_data  = np.array(left_hand_array[:]).reshape(25, 3).copy()
                    with right_hand_array.get_lock():
                        right_hand_data = np.array(right_hand_array[:]).reshape(25, 3).copy()

                    if not np.all(right_hand_data == 0.0) and not np.all(left_hand_data[4] == np.array([-1.13, 0.3, 0.15])): # if hand data has been initialized.
                        ref_left_value = left_hand_data[self.hand_retargeting.left_indices[1,:]] - left_hand_data[self.hand_retargeting.left_indices[0,:]]
                        ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]

                        left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                        right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]

                        # Official DFX angles are normalized: 0.0 is closed, 1.0 is open.
                        def normalize(val, min_val, max_val):
                            return np.clip((max_val - val) / (max_val - min_val), 0.0, 1.0)

                        for idx in range(Inspire_Num_Motors):
                            if idx <= 3:
                                left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 1.7)
                                right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.7)
                            elif idx == 4:
                                left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 0.5)
                                right_q_target[idx] = normalize(right_q_target[idx], 0.0, 0.5)
                            elif idx == 5:
                                left_q_target[idx]  = normalize(left_q_target[idx], -0.1, 1.3)
                                right_q_target[idx] = normalize(right_q_target[idx], -0.1, 1.3)

                # get dual hand action
                action_data = np.concatenate((left_q_target, right_q_target))    
                if dual_hand_state_array and dual_hand_action_array:
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state_data
                        dual_hand_action_array[:] = action_data

                self.ctrl_dual_hand(left_q_target, right_q_target)
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            logger_mp.info("Inspire_Controller_DFX has been closed.")



kTopicInspireFTPLeftCommand   = "rt/inspire_hand/ctrl/l"
kTopicInspireFTPRightCommand  = "rt/inspire_hand/ctrl/r"
kTopicInspireFTPLeftState  = "rt/inspire_hand/state/l"
kTopicInspireFTPRightState = "rt/inspire_hand/state/r"
INSPIRE_FTP_LEFT_IP = "192.168.123.210"
INSPIRE_FTP_RIGHT_IP = "192.168.123.211"
INSPIRE_FTP_OPEN_TARGET = np.ones(Inspire_Num_Motors)
INSPIRE_FTP_CLOSE_TARGET = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

class Inspire_Controller_FTP:
    def __init__(self, left_hand_array, right_hand_array, dual_hand_data_lock = None, dual_hand_state_array = None,
                       dual_hand_action_array = None, fps = 100.0, Unit_Test = False, simulation_mode = False,
                       control_mode = "hand", direct_modbus = True,
                       left_hand_ip = INSPIRE_FTP_LEFT_IP, right_hand_ip = INSPIRE_FTP_RIGHT_IP,
                       modbus_port = 6000, command_speed = 300):
        logger_mp.info("Initialize Inspire_Controller_FTP...")
        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        self.control_mode = control_mode
        self.direct_modbus = direct_modbus
        self.left_hand_ip = left_hand_ip
        self.right_hand_ip = right_hand_ip
        self.modbus_port = modbus_port
        self.command_speed = int(np.clip(command_speed, 1, 1000))
        self.inspire_hand_default = None
        self.hand_retargeting = None
        if self.control_mode == "hand":
            if not self.Unit_Test:
                self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)
            else:
                self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND_Unit_Test)

        # Shared Arrays for hand states ([0,1] normalized values)
        self.left_hand_state_array  = Array('d', Inspire_Num_Motors, lock=True)
        self.right_hand_state_array = Array('d', Inspire_Num_Motors, lock=True)

        if self.direct_modbus:
            logger_mp.info(
                "[Inspire_Controller_FTP] Using direct ModbusTCP control "
                f"(left={self.left_hand_ip}:{self.modbus_port}, right={self.right_hand_ip}:{self.modbus_port}, speed={self.command_speed})."
            )
        else:
            from inspire_sdkpy import inspire_dds  # lazy import
            import inspire_sdkpy.inspire_hand_defaut as inspire_hand_default
            self.inspire_hand_default = inspire_hand_default

            # Initialize hand command publishers
            self.LeftHandCmd_publisher = ChannelPublisher(kTopicInspireFTPLeftCommand, inspire_dds.inspire_hand_ctrl)
            self.LeftHandCmd_publisher.Init()
            self.RightHandCmd_publisher = ChannelPublisher(kTopicInspireFTPRightCommand, inspire_dds.inspire_hand_ctrl)
            self.RightHandCmd_publisher.Init()

            # Initialize hand state subscribers
            self.LeftHandState_subscriber = ChannelSubscriber(kTopicInspireFTPLeftState, inspire_dds.inspire_hand_state)
            self.LeftHandState_subscriber.Init() # Consider using callback if preferred: Init(callback_func, period_ms)
            self.RightHandState_subscriber = ChannelSubscriber(kTopicInspireFTPRightState, inspire_dds.inspire_hand_state)
            self.RightHandState_subscriber.Init()

            # Initialize subscribe thread
            self.subscribe_state_thread = threading.Thread(target=self._subscribe_hand_state)
            self.subscribe_state_thread.daemon = True
            self.subscribe_state_thread.start()

            # Wait for initial DDS messages (optional, but good for ensuring connection)
            wait_count = 0
            while not (any(self.left_hand_state_array) or any(self.right_hand_state_array)):
                if wait_count % 100 == 0: # Print every second
                    logger_mp.info(f"[Inspire_Controller_FTP] Waiting to subscribe to hand states from DDS (L: {any(self.left_hand_state_array)}, R: {any(self.right_hand_state_array)})...")
                time.sleep(0.01)
                wait_count += 1
                if wait_count > 500: # Timeout after 5 seconds
                    logger_mp.warning("[Inspire_Controller_FTP] Warning: Timeout waiting for initial hand states. Proceeding anyway.")
                    break
            logger_mp.info("[Inspire_Controller_FTP] Initial hand states received or timeout.")

        hand_control_process = Process(target=self.control_process, args=(left_hand_array, right_hand_array, self.left_hand_state_array, self.right_hand_state_array,
                                                                          dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array,
                                                                          self.control_mode, self.direct_modbus,
                                                                          self.left_hand_ip, self.right_hand_ip,
                                                                          self.modbus_port, self.command_speed))
        hand_control_process.daemon = True
        hand_control_process.start()

        logger_mp.info("Initialize Inspire_Controller_FTP OK!\n")

    def _subscribe_hand_state(self):
        logger_mp.info("[Inspire_Controller_FTP] Subscribe thread started.")
        while True:
            # Left Hand
            left_state_msg = self.LeftHandState_subscriber.Read()
            if left_state_msg is not None:
                if hasattr(left_state_msg, 'angle_act') and len(left_state_msg.angle_act) == Inspire_Num_Motors:
                    with self.left_hand_state_array.get_lock():
                        for i in range(Inspire_Num_Motors):
                            self.left_hand_state_array[i] = left_state_msg.angle_act[i] / 1000.0
                else:
                    logger_mp.warning(f"[Inspire_Controller_FTP] Received left_state_msg but attributes are missing or incorrect. Type: {type(left_state_msg)}, Content: {str(left_state_msg)[:100]}")
            # Right Hand
            right_state_msg = self.RightHandState_subscriber.Read()
            if right_state_msg is not None:
                if hasattr(right_state_msg, 'angle_act') and len(right_state_msg.angle_act) == Inspire_Num_Motors:
                    with self.right_hand_state_array.get_lock():
                        for i in range(Inspire_Num_Motors):
                            self.right_hand_state_array[i] = right_state_msg.angle_act[i] / 1000.0
                else:
                    logger_mp.warning(f"[Inspire_Controller_FTP] Received right_state_msg but attributes are missing or incorrect. Type: {type(right_state_msg)}, Content: {str(right_state_msg)[:100]}")

            time.sleep(0.002)

    def _send_hand_command(self, left_angle_cmd_scaled, right_angle_cmd_scaled):
        """
        Send scaled angle commands [0-1000] to both hands.
        """
        # Left Hand Command
        left_cmd_msg = self.inspire_hand_default.get_inspire_hand_ctrl()
        left_cmd_msg.angle_set = left_angle_cmd_scaled
        left_cmd_msg.mode = 0b0001 # Mode 1: Angle control
        self.LeftHandCmd_publisher.Write(left_cmd_msg)

        # Right Hand Command
        right_cmd_msg = self.inspire_hand_default.get_inspire_hand_ctrl()
        right_cmd_msg.angle_set = right_angle_cmd_scaled
        right_cmd_msg.mode = 0b0001 # Mode 1: Angle control
        self.RightHandCmd_publisher.Write(right_cmd_msg)

        # 临时打开前 N 次的 log
        if not hasattr(self, "_debug_count"):
            self._debug_count = 0
        if self._debug_count < 50:
            logger_mp.info(f"[Inspire_Controller_FTP] Publish cmd L={left_angle_cmd_scaled} R={right_angle_cmd_scaled} ")
            self._debug_count += 1


    def control_process(self, left_hand_array, right_hand_array, left_hand_state_array, right_hand_state_array,
                              dual_hand_data_lock = None, dual_hand_state_array = None, dual_hand_action_array = None,
                              control_mode = "hand", direct_modbus = True,
                              left_hand_ip = INSPIRE_FTP_LEFT_IP, right_hand_ip = INSPIRE_FTP_RIGHT_IP,
                              modbus_port = 6000, command_speed = 300):
        logger_mp.info("[Inspire_Controller_FTP] Control process started.")
        self.running = True

        left_q_target  = INSPIRE_FTP_OPEN_TARGET.copy()
        right_q_target = INSPIRE_FTP_OPEN_TARGET.copy()
        left_client = None
        right_client = None
        last_left_cmd = None
        last_right_cmd = None
        last_left_write_time = 0.0
        last_right_write_time = 0.0

        def connect_modbus_client(ip, label):
            from pymodbus.client import ModbusTcpClient

            client = ModbusTcpClient(ip, port=modbus_port, timeout=1.0)
            if not client.connect():
                logger_mp.error(f"[Inspire_Controller_FTP] Failed to connect {label} FTP hand at {ip}:{modbus_port}.")
                return client
            try:
                client.write_register(1004, 1, slave=1)  # clear hand error state
                client.write_registers(1522, [command_speed] * Inspire_Num_Motors, slave=1)
            except Exception as exc:
                logger_mp.warning(f"[Inspire_Controller_FTP] Failed to initialize {label} FTP hand speed/error state: {exc}")
            logger_mp.info(f"[Inspire_Controller_FTP] Connected {label} FTP hand at {ip}:{modbus_port}.")
            return client

        def write_modbus_command(client, ip, label, cmd):
            if client is None or not client.connected:
                client = connect_modbus_client(ip, label)
            if client is None or not client.connected:
                return client, False
            try:
                response = client.write_registers(1486, cmd, slave=1)
                if response.isError():
                    logger_mp.warning(f"[Inspire_Controller_FTP] {label} FTP hand rejected command: {response}")
                    return client, False
                return client, True
            except Exception as exc:
                logger_mp.warning(f"[Inspire_Controller_FTP] {label} FTP hand write failed, will reconnect: {exc}")
                try:
                    client.close()
                except Exception:
                    pass
                return connect_modbus_client(ip, label), False

        if direct_modbus:
            command_speed = int(np.clip(command_speed, 1, 1000))
            left_client = connect_modbus_client(left_hand_ip, "left")
            right_client = connect_modbus_client(right_hand_ip, "right")

        try:
            while self.running:
                start_time = time.time()
                # Read left and right q_state from shared arrays
                state_data = np.concatenate((np.array(left_hand_state_array[:]), np.array(right_hand_state_array[:])))

                if control_mode == "binary":
                    with left_hand_array.get_lock():
                        left_close_value = np.clip(left_hand_array.value, 0.0, 1.0)
                    with right_hand_array.get_lock():
                        right_close_value = np.clip(right_hand_array.value, 0.0, 1.0)
                    left_q_target = INSPIRE_FTP_OPEN_TARGET * (1.0 - left_close_value) + INSPIRE_FTP_CLOSE_TARGET * left_close_value
                    right_q_target = INSPIRE_FTP_OPEN_TARGET * (1.0 - right_close_value) + INSPIRE_FTP_CLOSE_TARGET * right_close_value
                else:
                    # get dual hand state
                    with left_hand_array.get_lock():
                        left_hand_data  = np.array(left_hand_array[:]).reshape(25, 3).copy()
                    with right_hand_array.get_lock():
                        right_hand_data = np.array(right_hand_array[:]).reshape(25, 3).copy()

                    if not np.all(right_hand_data == 0.0) and not np.all(left_hand_data[4] == np.array([-1.13, 0.3, 0.15])): # if hand data has been initialized.
                        ref_left_value = left_hand_data[self.hand_retargeting.left_indices[1,:]] - left_hand_data[self.hand_retargeting.left_indices[0,:]]
                        ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]

                        left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                        right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]

                        def normalize(val, min_val, max_val):
                            return np.clip((max_val - val) / (max_val - min_val), 0.0, 1.0)

                        for idx in range(Inspire_Num_Motors):
                            if idx <= 3:
                                left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 1.7)
                                right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.7)
                            elif idx == 4:
                                left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 0.5)
                                right_q_target[idx] = normalize(right_q_target[idx], 0.0, 0.5)
                            elif idx == 5:
                                left_q_target[idx]  = normalize(left_q_target[idx], -0.1, 1.3)
                                right_q_target[idx] = normalize(right_q_target[idx], -0.1, 1.3)

                scaled_left_cmd = [int(np.clip(val * 1000, 0, 1000)) for val in left_q_target]
                scaled_right_cmd = [int(np.clip(val * 1000, 0, 1000)) for val in right_q_target]

                # get dual hand action
                action_data = np.concatenate((left_q_target, right_q_target))
                if dual_hand_state_array and dual_hand_action_array:
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state_data
                        dual_hand_action_array[:] = action_data

                if direct_modbus:
                    now = time.time()
                    if last_left_cmd != scaled_left_cmd or now - last_left_write_time > 0.25:
                        left_client, ok = write_modbus_command(left_client, left_hand_ip, "left", scaled_left_cmd)
                        if ok:
                            last_left_cmd = scaled_left_cmd
                            last_left_write_time = now
                    if last_right_cmd != scaled_right_cmd or now - last_right_write_time > 0.25:
                        right_client, ok = write_modbus_command(right_client, right_hand_ip, "right", scaled_right_cmd)
                        if ok:
                            last_right_cmd = scaled_right_cmd
                            last_right_write_time = now
                else:
                    self._send_hand_command(scaled_left_cmd, scaled_right_cmd)
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            for client in (left_client, right_client):
                if client is not None:
                    try:
                        client.close()
                    except Exception:
                        pass
            logger_mp.info("Inspire_Controller_FTP has been closed.")

# Update hand state, according to the official documentation:
# 1. https://support.unitree.com/home/en/G1_developer/inspire_dfx_dexterous_hand
# 2. https://support.unitree.com/home/en/G1_developer/inspire_ftp_dexterity_hand
# the state sequence is as shown in the table below
# ┌──────┬───────┬──────┬────────┬────────┬────────────┬────────────────┬───────┬──────┬────────┬────────┬────────────┬────────────────┐
# │ Id   │   0   │  1   │   2    │   3    │     4      │       5        │   6   │  7   │   8    │   9    │    10      │       11       │
# ├──────┼───────┼──────┼────────┼────────┼────────────┼────────────────┼───────┼──────┼────────┼────────┼────────────┼────────────────┤
# │      │                    Right Hand                                │                   Left Hand                                  │
# │Joint │ pinky │ ring │ middle │ index  │ thumb-bend │ thumb-rotation │ pinky │ ring │ middle │ index  │ thumb-bend │ thumb-rotation │
# └──────┴───────┴──────┴────────┴────────┴────────────┴────────────────┴───────┴──────┴────────┴────────┴────────────┴────────────────┘
class Inspire_Right_Hand_JointIndex(IntEnum):
    kRightHandPinky = 0
    kRightHandRing = 1
    kRightHandMiddle = 2
    kRightHandIndex = 3
    kRightHandThumbBend = 4
    kRightHandThumbRotation = 5

class Inspire_Left_Hand_JointIndex(IntEnum):
    kLeftHandPinky = 6
    kLeftHandRing = 7
    kLeftHandMiddle = 8
    kLeftHandIndex = 9
    kLeftHandThumbBend = 10
    kLeftHandThumbRotation = 11
