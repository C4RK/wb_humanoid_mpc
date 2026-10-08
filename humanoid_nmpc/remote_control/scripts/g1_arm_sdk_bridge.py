#!/usr/bin/env python3
"""Safety bridge from localhost UDP arm targets to the G1 arm SDK.

The ROS 2 side sends five left-arm joint targets to UDP port 15000. This
process runs in the host's Unitree Python environment, rate-limits the targets,
checks freshness and joint bounds, and publishes them on rt/arm_sdk.

Without --execute no Unitree command publisher is created.
"""

import argparse
import math
import socket
import struct
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


PACKET = struct.Struct("!4sIQ5d")
PACKET_MAGIC = b"G1A1"
ARM_SDK_WEIGHT = 29
CONTROL_PERIOD = 0.02
KP = 60.0
KD = 1.5

LEFT_ARM = (
    (15, "pitch", (-3.0892, 2.6704)),
    (16, "roll", (-1.5882, 2.2515)),
    (17, "yaw", (-2.6180, 2.6180)),
    (18, "elbow", (-1.0472, 2.0944)),
    (19, "wrist", (-1.972222054, 1.972222054)),
)
CONTROLLED_JOINTS = (12, 15, 16, 17, 18, 19, 22, 23, 24, 25, 26)
STATIONARY_JOINTS = (
    0, 1, 2, 3, 4, 5,
    6, 7, 8, 9, 10, 11,
    *CONTROLLED_JOINTS,
)


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


class UdpTargetReceiver:
    def __init__(self, bind_host: str, port: int):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((bind_host, port))
        self.socket.setblocking(False)
        self.target = None
        self.sequence = None
        self.received_at = 0.0

    def close(self) -> None:
        self.socket.close()

    def update(self) -> None:
        while True:
            try:
                payload, _address = self.socket.recvfrom(2048)
            except BlockingIOError:
                return

            if len(payload) != PACKET.size:
                continue
            magic, sequence, _timestamp_ns, *target = PACKET.unpack(payload)
            if magic != PACKET_MAGIC or not all(math.isfinite(value) for value in target):
                continue
            self.target = tuple(target)
            self.sequence = sequence
            self.received_at = time.monotonic()

    def age(self) -> float:
        if self.target is None:
            return math.inf
        return time.monotonic() - self.received_at


def smoothstep(progress: float) -> float:
    progress = min(max(progress, 0.0), 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * progress)


def validate_robot_state(state: LowState_) -> None:
    roll, pitch = state.imu_state.rpy[:2]
    max_tilt_deg = max(abs(math.degrees(roll)), abs(math.degrees(pitch)))
    max_speed = max(abs(state.motor_state[index].dq) for index in STATIONARY_JOINTS)
    if max_tilt_deg > 10.0:
        raise RuntimeError(f"IMU tilt is {max_tilt_deg:.1f} deg")
    if max_speed > 0.3:
        raise RuntimeError(f"robot is moving at up to {max_speed:.2f} rad/s")


def validate_target(target) -> None:
    for value, (_index, name, limits) in zip(target, LEFT_ARM):
        if not limits[0] <= value <= limits[1]:
            raise RuntimeError(
                f"{name} target {math.degrees(value):.1f} deg exceeds URDF limits"
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


def publish_command(publisher, command, crc, desired, weight: float) -> None:
    fill_command(command, desired, weight)
    command.crc = crc.Crc(command)
    publisher.Write(command)


def wait_for_target(target_receiver: UdpTargetReceiver, timeout: float):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        target_receiver.update()
        if target_receiver.target is not None:
            validate_target(target_receiver.target)
            return target_receiver.target
        time.sleep(0.02)
    raise RuntimeError("no valid UDP arm target received")


def run_bridge(args, state_receiver, target_receiver) -> None:
    state, state_time = state_receiver.snapshot()
    if state is None or time.monotonic() - state_time > 0.2:
        raise RuntimeError("LowState is unavailable")
    validate_robot_state(state)

    initial = {
        index: state.motor_state[index].q
        for index in CONTROLLED_JOINTS
    }
    desired = dict(initial)
    first_target = wait_for_target(target_receiver, 5.0)

    max_excursion = math.radians(args.max_excursion_deg)
    for value, (index, name, _limits) in zip(first_target, LEFT_ARM):
        difference = abs(value - initial[index])
        if difference > max_excursion:
            raise RuntimeError(
                f"initial {name} target differs by {math.degrees(difference):.1f} deg; "
                f"limit is {args.max_excursion_deg:.1f} deg"
            )

    target_text = " ".join(
        f"{name}={math.degrees(value):+.1f}"
        for value, (_index, name, _limits) in zip(first_target, LEFT_ARM)
    )
    print(f"Fresh ROS arm target received: {target_text} deg")
    confirmation = input("Type TRACK and press Enter to enable physical arm tracking: ")
    if confirmation != "TRACK":
        print("Tracking cancelled. No command publisher was created.")
        return

    publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
    publisher.Init()
    command = unitree_hg_msg_dds__LowCmd_()
    crc = CRC()
    max_step = math.radians(args.max_rate_deg_s) * CONTROL_PERIOD

    try:
        started = time.monotonic()
        next_tick = started
        while time.monotonic() - started < 2.0:
            progress = (time.monotonic() - started) / 2.0
            publish_command(
                publisher, command, crc, desired, smoothstep(progress)
            )
            target_receiver.update()
            next_tick += CONTROL_PERIOD
            time.sleep(max(0.0, next_tick - time.monotonic()))

        print("arm_sdk enabled; tracking ROS targets.")
        started = time.monotonic()
        next_tick = started
        next_log = started
        while args.duration <= 0.0 or time.monotonic() - started < args.duration:
            now = time.monotonic()
            target_receiver.update()
            if target_receiver.age() > args.target_timeout:
                raise RuntimeError(
                    f"target stream stale for {target_receiver.age():.3f} s"
                )
            state, state_time = state_receiver.snapshot()
            if state is None or now - state_time > 0.2:
                raise RuntimeError("LowState stream timed out")

            validate_target(target_receiver.target)
            for value, (index, _name, limits) in zip(
                target_receiver.target, LEFT_ARM
            ):
                safe_lower = max(limits[0], initial[index] - max_excursion)
                safe_upper = min(limits[1], initial[index] + max_excursion)
                bounded = min(max(value, safe_lower), safe_upper)
                difference = bounded - desired[index]
                desired[index] += min(max(difference, -max_step), max_step)

            publish_command(publisher, command, crc, desired, 1.0)

            if now >= next_log:
                pairs = []
                for index, name, _limits in LEFT_ARM:
                    pairs.append(
                        f"{name} {math.degrees(desired[index]):+.1f}/"
                        f"{math.degrees(state.motor_state[index].q):+.1f}"
                    )
                print("desired/actual deg: " + "  ".join(pairs))
                next_log = now + 1.0

            next_tick += CONTROL_PERIOD
            time.sleep(max(0.0, next_tick - time.monotonic()))
    finally:
        print("Releasing arm_sdk control...")
        started = time.monotonic()
        next_tick = started
        while time.monotonic() - started < 2.0:
            progress = (time.monotonic() - started) / 2.0
            publish_command(
                publisher, command, crc, desired, 1.0 - smoothstep(progress)
            )
            next_tick += CONTROL_PERIOD
            time.sleep(max(0.0, next_tick - time.monotonic()))
        for _ in range(5):
            publish_command(publisher, command, crc, desired, 0.0)
            time.sleep(CONTROL_PERIOD)
        print("arm_sdk weight returned to zero.")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("interface", help="wired interface connected to the G1")
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=15000)
    parser.add_argument(
        "--duration",
        type=float,
        default=15.0,
        help="tracking duration in seconds; <=0 runs until Ctrl+C",
    )
    parser.add_argument("--target-timeout", type=float, default=0.25)
    parser.add_argument("--max-rate-deg-s", type=float, default=15.0)
    parser.add_argument("--max-excursion-deg", type=float, default=20.0)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="publish to rt/arm_sdk; without this flag the bridge is read-only",
    )
    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.interface)
    state_receiver = StateReceiver()
    target_receiver = UdpTargetReceiver(args.bind_host, args.port)
    state_receiver.start()
    print(
        f"Waiting for LowState and UDP targets on {args.bind_host}:{args.port}. "
        f"execute={args.execute}"
    )

    try:
        if not state_receiver.first_message.wait(timeout=5.0):
            raise RuntimeError("no LowState received within 5 seconds")
        state, _ = state_receiver.snapshot()
        validate_robot_state(state)

        if not args.execute:
            target = wait_for_target(target_receiver, 10.0)
            text = " ".join(
                f"{name}={math.degrees(value):+.1f}"
                for value, (_index, name, _limits) in zip(target, LEFT_ARM)
            )
            print(f"DRY RUN: received {text} deg; no command publisher was created.")
            return 0

        run_bridge(args, state_receiver, target_receiver)
        return 0
    except (KeyboardInterrupt, RuntimeError) as error:
        print(f"STOPPED: {error}")
        return 1
    finally:
        target_receiver.close()
        state_receiver.close()


if __name__ == "__main__":
    raise SystemExit(main())
