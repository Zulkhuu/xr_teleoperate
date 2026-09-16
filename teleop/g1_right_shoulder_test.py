#!/usr/bin/env python3

import sys
import time
import math
import threading

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__LowCmd_,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    LowCmd_,
    LowState_,
)
from unitree_sdk2py.utils.crc import CRC


# -----------------------------
# Configuration
# -----------------------------

DT = 0.02                    # 50 Hz, same as Unitree example
KP = 60.0
KD = 1.5

RIGHT_SHOULDER_PITCH = 22
ARM_SDK_WEIGHT = 29

MOVE_RAD = math.radians(10.0)

RAMP_WEIGHT_TIME = 2.0
MOVE_TIME = 2.0
HOLD_TIME = 2.0
RETURN_TIME = 2.0
RELEASE_TIME = 2.0


# 7 left arm + 7 right arm joints
ARM_JOINTS = [
    15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 26, 27, 28,
]


class ShoulderTest:
    def __init__(self):
        self.low_state = None
        self.lock = threading.Lock()

        self.publisher = ChannelPublisher(
            "rt/arm_sdk",
            LowCmd_,
        )
        self.publisher.Init()

        self.subscriber = ChannelSubscriber(
            "rt/lowstate",
            LowState_,
        )
        self.subscriber.Init(self.low_state_callback, 10)

        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC()

    def low_state_callback(self, msg):
        with self.lock:
            self.low_state = msg

    def wait_for_state(self):
        print("Waiting for rt/lowstate ...")

        while True:
            with self.lock:
                if self.low_state is not None:
                    return

            time.sleep(0.05)

    def get_arm_q(self):
        with self.lock:
            return {
                joint: self.low_state.motor_state[joint].q
                for joint in ARM_JOINTS
            }

    def publish(self, target_q, weight):
        # arm_sdk blending weight
        self.cmd.motor_cmd[ARM_SDK_WEIGHT].q = weight

        # Hold all arm joints at the requested positions
        for joint in ARM_JOINTS:
            self.cmd.motor_cmd[joint].tau = 0.0
            self.cmd.motor_cmd[joint].q = target_q[joint]
            self.cmd.motor_cmd[joint].dq = 0.0
            self.cmd.motor_cmd[joint].kp = KP
            self.cmd.motor_cmd[joint].kd = KD

        self.cmd.crc = self.crc.Crc(self.cmd)
        self.publisher.Write(self.cmd)

    def interpolate(self, start, end, duration, weight=1.0):
        steps = max(1, int(duration / DT))

        for i in range(steps + 1):
            r = i / steps

            q = start.copy()

            for joint in ARM_JOINTS:
                q[joint] = (
                    start[joint]
                    + r * (end[joint] - start[joint])
                )

            self.publish(q, weight)
            time.sleep(DT)

    def ramp_weight(self, q, start_weight, end_weight, duration):
        steps = max(1, int(duration / DT))

        for i in range(steps + 1):
            r = i / steps
            weight = start_weight + r * (end_weight - start_weight)

            self.publish(q, weight)
            time.sleep(DT)

    def hold(self, q, duration, weight=1.0):
        end_time = time.time() + duration

        while time.time() < end_time:
            self.publish(q, weight)
            time.sleep(DT)

    def run(self):
        self.wait_for_state()

        start_q = self.get_arm_q()

        print("\nCurrent right shoulder pitch:")
        print(
            f"  {start_q[RIGHT_SHOULDER_PITCH]:.4f} rad "
            f"({math.degrees(start_q[RIGHT_SHOULDER_PITCH]):.2f} deg)"
        )

        target_q = start_q.copy()
        target_q[RIGHT_SHOULDER_PITCH] += MOVE_RAD

        print("\nTarget:")
        print(
            f"  {target_q[RIGHT_SHOULDER_PITCH]:.4f} rad "
            f"({math.degrees(target_q[RIGHT_SHOULDER_PITCH]):.2f} deg)"
        )

        print("\n1. Taking control while holding current arm pose...")
        self.ramp_weight(
            start_q,
            0.0,
            1.0,
            RAMP_WEIGHT_TIME,
        )

        print("2. Moving RIGHT shoulder pitch +5 degrees...")
        self.interpolate(
            start_q,
            target_q,
            MOVE_TIME,
        )

        print("3. Holding...")
        self.hold(
            target_q,
            HOLD_TIME,
        )

        print("4. Returning to original position...")
        self.interpolate(
            target_q,
            start_q,
            RETURN_TIME,
        )

        print("5. Releasing arm_sdk...")
        self.ramp_weight(
            start_q,
            1.0,
            0.0,
            RELEASE_TIME,
        )

        # Explicit final disabled command
        self.publish(start_q, 0.0)

        print("Done.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(
            f"Usage: {sys.argv[0]} <network_interface>\n"
            f"Example: {sys.argv[0]} enx207bd2c81d8d"
        )
        sys.exit(1)

    interface = sys.argv[1]

    print("WARNING:")
    print("  G1 should be standing securely in Regular mode.")
    print("  Keep the right arm workspace clear.")
    print("  Ensure no other program is publishing to rt/arm_sdk.")
    print()

    input("Press Enter to begin the 5-degree shoulder test...")

    ChannelFactoryInitialize(0, interface)

    test = ShoulderTest()

    try:
        test.run()

    except KeyboardInterrupt:
        print("\nInterrupted.")

        # Best-effort release if we already have state
        if test.low_state is not None:
            q = test.get_arm_q()
            print("Releasing arm_sdk...")
            test.ramp_weight(q, 1.0, 0.0, 1.0)

        print("Stopped.")