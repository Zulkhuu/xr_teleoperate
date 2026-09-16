import sys
import time
import threading

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

ARM = [15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28]
SDK_WEIGHT = 29
DT = 0.02
KP = 60.0
KD = 1.5
MOVE_RAD = 0.1373  # 5 degrees


class ArmTest:
    def __init__(self):
        self.lock = threading.Lock()
        self.state = None
        self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.pub.Init()
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.on_state, 10)
        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC()

    def on_state(self, msg):
        with self.lock:
            self.state = msg

    def get_q(self):
        with self.lock:
            return [self.state.motor_state[i].q for i in ARM]

    def send(self, q, weight=1.0):
        self.cmd.motor_cmd[SDK_WEIGHT].q = weight
        for i, joint in enumerate(ARM):
            self.cmd.motor_cmd[joint].q = q[i]
            self.cmd.motor_cmd[joint].dq = 0.0
            self.cmd.motor_cmd[joint].tau = 0.0
            self.cmd.motor_cmd[joint].kp = KP
            self.cmd.motor_cmd[joint].kd = KD
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)

    def run(self):
        print("Waiting for rt/lowstate...")
        while self.state is None:
            time.sleep(0.1)

        start = self.get_q()
        target = start.copy()
        target[6] += MOVE_RAD  # right shoulder pitch: motor 22

        print(f"Initial right shoulder pitch: {start[6]:.4f} rad")
        print("Taking arm_sdk control and moving +5 degrees.")

        # Blend into SDK control while holding current pose.
        for n in range(100):
            self.send(start, n / 99.0)
            time.sleep(DT)

        # Move over two seconds.
        for n in range(101):
            r = n / 100.0
            q = [a + r * (b - a) for a, b in zip(start, target)]
            self.send(q)
            time.sleep(DT)

        time.sleep(2.0)

        print("Returning to initial pose.")
        for n in range(101):
            r = n / 100.0
            q = [a + r * (b - a) for a, b in zip(target, start)]
            self.send(q)
            time.sleep(DT)

        # Release arm_sdk control while holding current pose.
        for n in range(100, -1, -1):
            self.send(start, n / 100.0)
            time.sleep(DT)

        print("Done.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <network_interface>")
        raise SystemExit(1)

    ChannelFactoryInitialize(0, sys.argv[1])
    ArmTest().run()