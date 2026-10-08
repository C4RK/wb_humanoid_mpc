#!/usr/bin/env python3
"""Small, guarded G1 arm test through the high-level arm SDK topic.

Without --execute this program is read-only. With --execute it holds both arms
at their measured starting positions, moves one selected left-arm joint by at
most five degrees, returns it, then hands arm control back to the onboard
controller.
"""

import argparse
import math
import threading
import time

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


ARM_SDK_WEIGHT = 29
CONTROLLED_JOINTS = (12, 15, 16, 17, 18, 19, 22, 23, 24, 25, 26)
STATIONARY_JOINTS = (
    0, 1, 2, 3, 4, 5,
    6, 7, 8, 9, 10, 11,
    12, 15, 16, 17, 18, 19,
    22, 23, 24, 25, 26,
)
JOINT_SPECS = {
    "left_shoulder_pitch": (15, (-3.0892, 2.6704)),
    "left_shoulder_roll": (16, (-1.5882, 2.2515)),
    "left_shoulder_yaw": (17, (-2.6180, 2.6180)),
    "left_elbow": (18, (-1.0472, 2.0944)),
    "left_wrist_roll": (19, (-1.972222054, 1.972222054)),
}
MAX_DELTA_RAD = math.radians(5.0)
KP = 60.0
KD = 1.5
CONTROL_PERIOD = 0.02


class StateReceiver:
    def __init__(self):
        self._lock = threading.Lock()
        self._latest = None
        self._received_at = 0.0
        self.first_message = threading.Event()
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)

    def start(self) -> None:
        self.subscriber.Init(self._callback, 10)

    def close(self) -> None:
        self.subscriber.Close()

    def snapshot(self):
        with self._lock:
            return self._latest, self._received_at

    def _callback(self, message: LowState_) -> None:
        with self._lock:
            self._latest = message
            self._received_at = time.monotonic()
        self.first_message.set()


def smoothstep(progress: float) -> float:
    progress = min(max(progress, 0.0), 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * progress)


def validate_state(state: LowState_, joint_name: str, joint_index: int, limits) -> None:
    roll, pitch = state.imu_state.rpy[:2]
    max_tilt = max(abs(math.degrees(roll)), abs(math.degrees(pitch)))
    max_speed = max(abs(state.motor_state[index].dq) for index in STATIONARY_JOINTS)
    joint_position = state.motor_state[joint_index].q

    if max_tilt > 10.0:
        raise RuntimeError(f"IMU tilt is {max_tilt:.1f} deg; expected a stable standing pose")
    if max_speed > 0.3:
        raise RuntimeError(
            f"robot is moving (maximum measured joint speed {max_speed:.2f} rad/s)"
        )
    if not limits[0] <= joint_position <= limits[1]:
        raise RuntimeError(
            f"{joint_name} angle {math.degrees(joint_position):.1f} deg is invalid"
        )


def fill_command(command, desired, weight: float) -> None:
    for index in CONTROLLED_JOINTS:
        motor = command.motor_cmd[index]
        motor.tau = 0.0
        motor.q = desired[index]
        motor.dq = 0.0
        motor.kp = KP
        motor.kd = KD
    command.motor_cmd[ARM_SDK_WEIGHT].q = min(max(weight, 0.0), 1.0)


def send_phase(
    publisher,
    command,
    crc,
    receiver,
    duration: float,
    desired_fn,
    weight_fn,
    label: str,
    joint_index: int,
) -> None:
    started = time.monotonic()
    next_tick = started
    next_log = started

    while True:
        now = time.monotonic()
        elapsed = now - started
        if elapsed >= duration:
            break

        state, received_at = receiver.snapshot()
        if state is None or now - received_at > 0.2:
            raise RuntimeError("LowState timeout; stopping arm command stream")

        progress = elapsed / duration
        desired = desired_fn(progress)
        weight = weight_fn(progress)
        fill_command(command, desired, weight)
        command.crc = crc.Crc(command)
        publisher.Write(command)

        if now >= next_log:
            actual_deg = math.degrees(state.motor_state[joint_index].q)
            target_deg = math.degrees(desired[joint_index])
            print(
                f"{label:<9} target={target_deg:+7.2f} deg  "
                f"actual={actual_deg:+7.2f} deg  arm_sdk_weight={weight:.2f}"
            )
            next_log = now + 0.5

        next_tick += CONTROL_PERIOD
        time.sleep(max(0.0, next_tick - time.monotonic()))


def execute_test(
    receiver: StateReceiver,
    joint_name: str,
    joint_index: int,
    limits,
    delta_rad: float,
) -> None:
    state, received_at = receiver.snapshot()
    if state is None or time.monotonic() - received_at > 0.2:
        raise RuntimeError("LowState is unavailable")
    validate_state(state, joint_name, joint_index, limits)

    initial = {
        index: state.motor_state[index].q
        for index in CONTROLLED_JOINTS
    }
    target = dict(initial)
    target[joint_index] += delta_rad
    if not limits[0] <= target[joint_index] <= limits[1]:
        raise RuntimeError(f"requested {joint_name} target exceeds the G1 joint limit")

    print(
        f"Initial {joint_name}: {math.degrees(initial[joint_index]):.2f} deg; "
        f"target: {math.degrees(target[joint_index]):.2f} deg"
    )
    confirmation = input("Type MOVE and press Enter to start the physical motion: ")
    if confirmation != "MOVE":
        print("Motion cancelled. No command publisher was created.")
        return

    publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
    publisher.Init()
    command = unitree_hg_msg_dds__LowCmd_()
    crc = CRC()

    hold_initial = lambda _progress: initial
    move_out = lambda progress: {
        **initial,
        joint_index: initial[joint_index]
        + smoothstep(progress) * (target[joint_index] - initial[joint_index]),
    }
    hold_target = lambda _progress: target
    move_back = lambda progress: {
        **initial,
        joint_index: target[joint_index]
        + smoothstep(progress) * (initial[joint_index] - target[joint_index]),
    }

    try:
        send_phase(
            publisher, command, crc, receiver, 2.0,
            hold_initial, smoothstep, "takeover", joint_index,
        )
        send_phase(
            publisher, command, crc, receiver, 3.0,
            move_out, lambda _progress: 1.0, "move", joint_index,
        )
        send_phase(
            publisher, command, crc, receiver, 1.0,
            hold_target, lambda _progress: 1.0, "hold", joint_index,
        )
        send_phase(
            publisher, command, crc, receiver, 3.0,
            move_back, lambda _progress: 1.0, "return", joint_index,
        )
    finally:
        # Keep the last measured pose while blending arm control back to the
        # onboard controller. This path also runs after Ctrl+C or an exception.
        state, _ = receiver.snapshot()
        release_pose = dict(initial)
        if state is not None:
            release_pose = {
                index: state.motor_state[index].q
                for index in CONTROLLED_JOINTS
            }
        send_phase(
            publisher, command, crc, receiver, 2.0,
            lambda _progress: release_pose,
            lambda progress: 1.0 - smoothstep(progress),
            "release", joint_index,
        )
        fill_command(command, release_pose, 0.0)
        command.crc = crc.Crc(command)
        for _ in range(5):
            publisher.Write(command)
            time.sleep(CONTROL_PERIOD)

    print("Fixed arm test completed; arm_sdk weight returned to zero.")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("interface", help="wired interface connected to the G1")
    parser.add_argument(
        "--joint",
        choices=tuple(JOINT_SPECS),
        default="left_elbow",
        help="left-arm joint to test (default: left_elbow)",
    )
    parser.add_argument(
        "--delta-deg",
        type=float,
        default=5.0,
        help="selected joint offset, limited to +/-5 degrees",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="enable physical motion; without this flag the program is read-only",
    )
    args = parser.parse_args()

    joint_index, limits = JOINT_SPECS[args.joint]
    delta_rad = math.radians(args.delta_deg)
    if abs(delta_rad) > MAX_DELTA_RAD:
        print("ERROR: --delta-deg must be between -5 and +5.")
        return 2

    ChannelFactoryInitialize(0, args.interface)
    receiver = StateReceiver()
    receiver.start()
    print(f"Waiting for G1 LowState on {args.interface}...")
    if not receiver.first_message.wait(timeout=5.0):
        receiver.close()
        print("ERROR: no LowState received within 5 seconds.")
        return 1

    state, _ = receiver.snapshot()
    try:
        validate_state(state, args.joint, joint_index, limits)
        print(
            f"State OK: mode_machine={state.mode_machine}, "
            f"IMU roll={math.degrees(state.imu_state.rpy[0]):+.2f} deg, "
            f"pitch={math.degrees(state.imu_state.rpy[1]):+.2f} deg, "
            f"{args.joint}={math.degrees(state.motor_state[joint_index].q):+.2f} deg."
        )
        if not args.execute:
            print("DRY RUN ONLY: no command publisher was created.")
            print("Add --execute only after the robot is secured and the area is clear.")
            return 0
        execute_test(
            receiver, args.joint, joint_index, limits, delta_rad
        )
        return 0
    except (KeyboardInterrupt, RuntimeError) as error:
        print(f"STOPPED: {error}")
        return 1
    finally:
        receiver.close()


if __name__ == "__main__":
    raise SystemExit(main())
