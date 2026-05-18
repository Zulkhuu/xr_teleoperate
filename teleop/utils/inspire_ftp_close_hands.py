#!/usr/bin/env python3
import argparse
import time

from pymodbus.client import ModbusTcpClient


DEFAULT_LEFT_IP = "192.168.123.210"
DEFAULT_RIGHT_IP = "192.168.123.211"
DEFAULT_PORT = 6000
FTP_CLOSE_TARGET = [0, 0, 0, 0, 0, 1000]


def write_close(ip, port, speed, timeout):
    client = ModbusTcpClient(ip, port=port, timeout=timeout)
    if not client.connect():
        print(f"{ip}: connect failed")
        return False
    try:
        client.write_register(1004, 1, slave=1)
        client.write_registers(1522, [speed] * 6, slave=1)
        time.sleep(0.05)
        response = client.write_registers(1486, FTP_CLOSE_TARGET, slave=1)
        if response.isError():
            print(f"{ip}: close command failed: {response}")
            return False
        print(f"{ip}: close command sent")
        return True
    finally:
        client.close()


def main():
    parser = argparse.ArgumentParser(description="Close both Inspire FTP hands for startup protection.")
    parser.add_argument("--left-ip", default=DEFAULT_LEFT_IP)
    parser.add_argument("--right-ip", default=DEFAULT_RIGHT_IP)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--speed", type=int, default=300)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()

    speed = max(1, min(1000, args.speed))
    ok_left = write_close(args.left_ip, args.port, speed, args.timeout)
    ok_right = write_close(args.right_ip, args.port, speed, args.timeout)
    raise SystemExit(0 if ok_left and ok_right else 1)


if __name__ == "__main__":
    main()
