#!/usr/bin/env python3
"""Read and display G1 low-level state without publishing any commands."""

import argparse
import math
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


# SDK motor indices used by the 23-DoF G1 model in this project. The locked
# waist roll/pitch and wrist pitch/yaw slots remain present in the DDS array.
G1_23DOF_JOINTS = (
    (0, "left_hip_pitch"),
    (1, "left_hip_roll"),
    (2, "left_hip_yaw"),
    (3, "left_knee"),
    (4, "left_ankle_pitch"),
    (5, "left_ankle_roll"),
    (6, "right_hip_pitch"),
    (7, "right_hip_roll"),
    (8, "right_hip_yaw"),
    (9, "right_knee"),
    (10, "right_ankle_pitch"),
    (11, "right_ankle_roll"),
    (12, "waist_yaw"),
    (15, "left_shoulder_pitch"),
    (16, "left_shoulder_roll"),
    (17, "left_shoulder_yaw"),
    (18, "left_elbow"),
    (19, "left_wrist_roll"),
    (22, "right_shoulder_pitch"),
    (23, "right_shoulder_roll"),
    (24, "right_shoulder_yaw"),
    (25, "right_elbow"),
    (26, "right_wrist_roll"),
)


class LowStateMonitor:
    def __init__(self, print_period: float):
        self.print_period = print_period
        self.message_count = 0
        self.first_message = threading.Event()
        self.last_message_time = 0.0
        self.last_print_time = 0.0
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)

    def start(self) -> None:
        self.subscriber.Init(self._on_state, 10)

    def close(self) -> None:
        self.subscriber.Close()

    def _on_state(self, msg: LowState_) -> None:
        now = time.monotonic()
        self.message_count += 1
        self.last_message_time = now
        self.first_message.set()

        if now - self.last_print_time < self.print_period:
            return
        self.last_print_time = now

        rpy_deg = [math.degrees(value) for value in msg.imu_state.rpy]
        print(
            f"\nLowState #{self.message_count}  mode_machine={msg.mode_machine}  "
            f"IMU rpy=[{rpy_deg[0]:+.2f}, {rpy_deg[1]:+.2f}, {rpy_deg[2]:+.2f}] deg"
        )
        print(" idx  joint                         q [deg]    dq [rad/s]   tau_est [Nm]")
        for index, name in G1_23DOF_JOINTS:
            state = msg.motor_state[index]
            print(
                f" {index:>3}  {name:<27}"
                f" {math.degrees(state.q):>9.2f}"
                f" {state.dq:>12.3f}"
                f" {state.tau_est:>14.3f}"
            )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Subscribe to G1 rt/lowstate. This program never publishes LowCmd."
    )
    parser.add_argument("interface", help="wired interface connected to the G1")
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="monitor duration in seconds (default: 10)",
    )
    parser.add_argument(
        "--print-period",
        type=float,
        default=1.0,
        help="seconds between printed snapshots (default: 1)",
    )
    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.interface)
    monitor = LowStateMonitor(args.print_period)
    monitor.start()

    print(
        f"Waiting for rt/lowstate on {args.interface}. "
        "Read-only: no command publisher is created."
    )
    if not monitor.first_message.wait(timeout=5.0):
        monitor.close()
        print("ERROR: no LowState received within 5 seconds.")
        return 1

    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            time.sleep(0.1)
            if time.monotonic() - monitor.last_message_time > 1.0:
                print("WARNING: LowState stream has been silent for more than 1 second.")
    except KeyboardInterrupt:
        pass
    finally:
        count = monitor.message_count
        monitor.close()

    print(f"\nFinished after receiving {count} LowState messages.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
