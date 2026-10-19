#!/usr/bin/env python3
"""Safety bridge from localhost UDP arm targets to the G1 arm SDK.

The ROS 2 side sends five left-arm joint targets to UDP port 15000. This
process runs in the host's Unitree Python environment, rate-limits the targets,
checks freshness and joint bounds, and publishes them on rt/arm_sdk.

Without --execute no Unitree command publisher is created.
"""

import argparse
import csv
from datetime import datetime, timezone
import math
import os
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
LEFT_ARM_JOINTS = tuple(index for index, _name, _limits in LEFT_ARM)
LEFT_ARM_NAMES = tuple(name for _index, name, _limits in LEFT_ARM)
BODY_AND_HELD_JOINTS = (
    0, 1, 2, 3, 4, 5,
    6, 7, 8, 9, 10, 11,
    12, 22, 23, 24, 25, 26,
)
STATIONARY_JOINTS = (
    0, 1, 2, 3, 4, 5,
    6, 7, 8, 9, 10, 11,
    *CONTROLLED_JOINTS,
)
JOINT_NAMES = {
    0: "left_hip_pitch",
    1: "left_hip_roll",
    2: "left_hip_yaw",
    3: "left_knee",
    4: "left_ankle_pitch",
    5: "left_ankle_roll",
    6: "right_hip_pitch",
    7: "right_hip_roll",
    8: "right_hip_yaw",
    9: "right_knee",
    10: "right_ankle_pitch",
    11: "right_ankle_roll",
    12: "waist_yaw",
    15: "left_shoulder_pitch",
    16: "left_shoulder_roll",
    17: "left_shoulder_yaw",
    18: "left_elbow",
    19: "left_wrist_roll",
    22: "right_shoulder_pitch",
    23: "right_shoulder_roll",
    24: "right_shoulder_yaw",
    25: "right_elbow",
    26: "right_wrist_roll",
}


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
        self.sent_monotonic_ns = None
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
            magic, sequence, timestamp_ns, *target = PACKET.unpack(payload)
            if magic != PACKET_MAGIC or not all(math.isfinite(value) for value in target):
                continue
            self.target = tuple(target)
            self.sequence = sequence
            self.sent_monotonic_ns = timestamp_ns
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
    fastest_index = max(
        STATIONARY_JOINTS,
        key=lambda index: abs(state.motor_state[index].dq),
    )
    max_speed = abs(state.motor_state[fastest_index].dq)
    if max_tilt_deg > 10.0:
        raise RuntimeError(f"IMU tilt is {max_tilt_deg:.1f} deg")
    if max_speed > 0.3:
        raise RuntimeError(
            f"{JOINT_NAMES[fastest_index]}[{fastest_index}] is moving at "
            f"{state.motor_state[fastest_index].dq:+.2f} rad/s; limit is 0.30 rad/s"
        )


def validate_tracking_state(state: LowState_, desired, args) -> None:
    """Abort active tracking when the physical robot leaves its safe envelope."""
    roll, pitch = state.imu_state.rpy[:2]
    if not all(math.isfinite(value) for value in (roll, pitch)):
        raise RuntimeError("IMU reported a non-finite tilt")

    max_tilt_deg = max(abs(math.degrees(roll)), abs(math.degrees(pitch)))
    if max_tilt_deg > args.max_tilt_deg:
        raise RuntimeError(
            f"IMU tilt reached {max_tilt_deg:.1f} deg; "
            f"limit is {args.max_tilt_deg:.1f} deg"
        )

    fastest_body_index = max(
        BODY_AND_HELD_JOINTS,
        key=lambda index: abs(state.motor_state[index].dq),
    )
    body_speed = abs(state.motor_state[fastest_body_index].dq)
    if not math.isfinite(body_speed) or body_speed > args.max_body_speed_rad_s:
        raise RuntimeError(
            f"{JOINT_NAMES[fastest_body_index]}[{fastest_body_index}] speed reached "
            f"{state.motor_state[fastest_body_index].dq:+.2f} rad/s; "
            f"limit is {args.max_body_speed_rad_s:.2f} rad/s"
        )

    fastest_arm_index = max(
        LEFT_ARM_JOINTS,
        key=lambda index: abs(state.motor_state[index].dq),
    )
    arm_speed = abs(state.motor_state[fastest_arm_index].dq)
    arm_speed_limit = math.radians(args.max_observed_arm_speed_deg_s)
    if not math.isfinite(arm_speed) or arm_speed > arm_speed_limit:
        raise RuntimeError(
            f"{JOINT_NAMES[fastest_arm_index]}[{fastest_arm_index}] speed reached "
            f"{math.degrees(state.motor_state[fastest_arm_index].dq):+.1f} deg/s; "
            f"limit is {args.max_observed_arm_speed_deg_s:.1f} deg/s"
        )

    worst_error = max(
        abs(desired[index] - state.motor_state[index].q)
        for index in CONTROLLED_JOINTS
    )
    tracking_error_limit = math.radians(args.max_tracking_error_deg)
    if not math.isfinite(worst_error) or worst_error > tracking_error_limit:
        raise RuntimeError(
            f"arm/waist tracking error reached {math.degrees(worst_error):.1f} deg; "
            f"limit is {args.max_tracking_error_deg:.1f} deg"
        )


def validate_target(target) -> None:
    for value, (_index, name, limits) in zip(target, LEFT_ARM):
        if not limits[0] <= value <= limits[1]:
            raise RuntimeError(
                f"{name} target {math.degrees(value):.1f} deg exceeds URDF limits"
            )


def map_target_to_robot(target, baseline_target, initial, relative_to_start,
                        active_joint_names):
    """Map ROS targets onto the current real-robot arm posture.

    The simulation target uses the model's absolute zero pose.  G1 FSM 500
    normally starts with a different arm posture, especially at the elbow.
    Relative mode preserves the robot posture captured at takeover and applies
    only changes from the first fresh ROS target.  Inactive joints remain at
    their measured takeover positions.
    """
    mapped = []
    for value, baseline, (index, name, _limits) in zip(
            target, baseline_target, LEFT_ARM):
        if name not in active_joint_names:
            mapped.append(float(initial[index]))
        elif relative_to_start:
            mapped.append(float(initial[index]) + float(value) - float(baseline))
        else:
            mapped.append(float(value))
    return tuple(mapped)


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


def tracking_log_header():
    columns = [
        "wall_time_ns",
        "monotonic_time_ns",
        "udp_sequence",
        "udp_sent_monotonic_ns",
        "udp_transport_latency_ms",
        "udp_target_age_ms",
        "imu_roll_rad",
        "imu_pitch_rad",
        "imu_yaw_rad",
    ]
    for _index, name, _limits in LEFT_ARM:
        columns.extend((
            f"source_target_{name}_rad",
            f"applied_target_{name}_rad",
            f"command_{name}_rad",
        ))
    for index in STATIONARY_JOINTS:
        name = JOINT_NAMES[index]
        columns.extend((f"actual_{name}_rad", f"velocity_{name}_rad_s"))
    return columns


def append_tracking_log(records, state, desired, target_receiver,
                        applied_target) -> None:
    """Record one synchronized bridge-loop sample without doing disk I/O."""
    monotonic_ns = time.monotonic_ns()
    sent_ns = target_receiver.sent_monotonic_ns
    transport_latency_ms = (
        (monotonic_ns - sent_ns) / 1e6 if sent_ns is not None else math.nan
    )
    row = [
        time.time_ns(),
        monotonic_ns,
        target_receiver.sequence,
        sent_ns,
        transport_latency_ms,
        target_receiver.age() * 1000.0,
        *[float(value) for value in state.imu_state.rpy[:3]],
    ]
    for source_value, applied_value, (index, _name, _limits) in zip(
            target_receiver.target, applied_target, LEFT_ARM):
        row.extend((
            float(source_value), float(applied_value), float(desired[index])))
    for index in STATIONARY_JOINTS:
        motor = state.motor_state[index]
        row.extend((float(motor.q), float(motor.dq)))
    records.append(row)


def save_tracking_log(records, log_dir: str):
    if not records:
        return None
    expanded_dir = os.path.abspath(os.path.expanduser(log_dir))
    os.makedirs(expanded_dir, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    path = os.path.join(expanded_dir, f"g1_arm_tracking_{timestamp}.csv")
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(tracking_log_header())
        writer.writerows(records)
    return path


def wait_for_target(target_receiver: UdpTargetReceiver, timeout: float):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        target_receiver.update()
        if target_receiver.target is not None:
            validate_target(target_receiver.target)
            return target_receiver.target
        time.sleep(0.02)
    raise RuntimeError("no valid UDP arm target received")


def run_bridge(args, state_receiver, target_receiver, tracking_records) -> None:
    state, state_time = state_receiver.snapshot()
    if state is None or time.monotonic() - state_time > 0.2:
        raise RuntimeError("LowState is unavailable")
    validate_robot_state(state)

    initial = {
        index: state.motor_state[index].q
        for index in CONTROLLED_JOINTS
    }
    desired = dict(initial)
    baseline_target = wait_for_target(target_receiver, 5.0)
    first_target = map_target_to_robot(
        baseline_target,
        baseline_target,
        initial,
        args.relative_to_start,
        args.active_joint_names,
    )
    validate_target(first_target)

    max_excursion = math.radians(args.max_excursion_deg)
    for value, (index, name, _limits) in zip(first_target, LEFT_ARM):
        difference = abs(value - initial[index])
        if difference > max_excursion:
            raise RuntimeError(
                f"initial {name} target differs by {math.degrees(difference):.1f} deg; "
                f"limit is {args.max_excursion_deg:.1f} deg"
            )

    source_text = " ".join(
        f"{name}={math.degrees(value):+.1f}"
        for value, (_index, name, _limits) in zip(baseline_target, LEFT_ARM)
    )
    applied_text = " ".join(
        f"{name}={math.degrees(value):+.1f}"
        for value, (_index, name, _limits) in zip(first_target, LEFT_ARM)
    )
    print(f"Fresh ROS arm target received: {source_text} deg")
    if args.relative_to_start:
        print(
            "Relative-to-start alignment is ENABLED; the first target maps to "
            f"the measured robot pose: {applied_text} deg"
        )
    print("Active tracked joints: " + ", ".join(sorted(args.active_joint_names)))
    confirmation = input("Type TRACK and press Enter to enable physical arm tracking: ")
    if confirmation != "TRACK":
        print("Tracking cancelled. No command publisher was created.")
        return

    publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
    publisher.Init()
    command = unitree_hg_msg_dds__LowCmd_()
    crc = CRC()
    max_step = math.radians(args.max_rate_deg_s) * CONTROL_PERIOD
    current_weight = 0.0

    try:
        started = time.monotonic()
        next_tick = started
        while time.monotonic() - started < 2.0:
            now = time.monotonic()
            progress = (now - started) / 2.0
            state, state_time = state_receiver.snapshot()
            if state is None or now - state_time > 0.2:
                raise RuntimeError("LowState stream timed out during arm_sdk takeover")
            validate_tracking_state(state, desired, args)
            current_weight = smoothstep(progress)
            publish_command(
                publisher, command, crc, desired, current_weight
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

            validate_tracking_state(state, desired, args)

            validate_target(target_receiver.target)
            applied_target = map_target_to_robot(
                target_receiver.target,
                baseline_target,
                initial,
                args.relative_to_start,
                args.active_joint_names,
            )
            validate_target(applied_target)
            for value, (index, _name, limits) in zip(applied_target, LEFT_ARM):
                safe_lower = max(limits[0], initial[index] - max_excursion)
                safe_upper = min(limits[1], initial[index] + max_excursion)
                bounded = min(max(value, safe_lower), safe_upper)
                difference = bounded - desired[index]
                desired[index] += min(max(difference, -max_step), max_step)

            publish_command(publisher, command, crc, desired, 1.0)
            current_weight = 1.0
            append_tracking_log(
                tracking_records, state, desired, target_receiver,
                applied_target,
            )

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
        state, _state_time = state_receiver.snapshot()
        release_pose = dict(desired)
        if state is not None:
            release_pose = {
                index: state.motor_state[index].q
                for index in CONTROLLED_JOINTS
            }
        release_start_weight = current_weight
        started = time.monotonic()
        next_tick = started
        while time.monotonic() - started < 2.0:
            progress = (time.monotonic() - started) / 2.0
            publish_command(
                publisher,
                command,
                crc,
                release_pose,
                release_start_weight * (1.0 - smoothstep(progress)),
            )
            next_tick += CONTROL_PERIOD
            time.sleep(max(0.0, next_tick - time.monotonic()))
        for _ in range(5):
            publish_command(publisher, command, crc, release_pose, 0.0)
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
    parser.add_argument("--max-tilt-deg", type=float, default=10.0)
    parser.add_argument("--max-body-speed-rad-s", type=float, default=0.5)
    parser.add_argument("--max-observed-arm-speed-deg-s", type=float, default=60.0)
    parser.add_argument("--max-tracking-error-deg", type=float, default=15.0)
    parser.add_argument(
        "--relative-to-start",
        action="store_true",
        help=(
            "map the first fresh ROS target to the measured robot posture and "
            "track only subsequent target changes"
        ),
    )
    parser.add_argument(
        "--active-joints",
        nargs="+",
        choices=LEFT_ARM_NAMES,
        default=list(LEFT_ARM_NAMES),
        metavar="JOINT",
        help=(
            "left-arm joints allowed to track; other left-arm joints remain "
            "at their measured takeover positions"
        ),
    )
    parser.add_argument(
        "--log-dir",
        default="~/humanoid_mpc_ws/calibration_logs",
        help="directory for the full-rate real-robot tracking CSV",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="publish to rt/arm_sdk; without this flag the bridge is read-only",
    )
    args = parser.parse_args()
    args.active_joint_names = frozenset(args.active_joints)

    positive_limits = {
        "--max-rate-deg-s": args.max_rate_deg_s,
        "--max-excursion-deg": args.max_excursion_deg,
        "--max-tilt-deg": args.max_tilt_deg,
        "--max-body-speed-rad-s": args.max_body_speed_rad_s,
        "--max-observed-arm-speed-deg-s": args.max_observed_arm_speed_deg_s,
        "--max-tracking-error-deg": args.max_tracking_error_deg,
    }
    for name, value in positive_limits.items():
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"{name} must be a positive finite value")

    ChannelFactoryInitialize(0, args.interface)
    state_receiver = StateReceiver()
    target_receiver = UdpTargetReceiver(args.bind_host, args.port)
    tracking_records = []
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
            source_target = wait_for_target(target_receiver, 10.0)
            state, state_time = state_receiver.snapshot()
            if state is None or time.monotonic() - state_time > 0.2:
                raise RuntimeError("LowState is unavailable")
            validate_robot_state(state)

            initial = {
                index: float(state.motor_state[index].q)
                for index in CONTROLLED_JOINTS
            }
            applied_target = map_target_to_robot(
                source_target,
                source_target,
                initial,
                args.relative_to_start,
                args.active_joint_names,
            )
            validate_target(applied_target)

            print("DRY RUN: fresh target received; no command publisher was created.")
            mode = "relative-to-start" if args.relative_to_start else "absolute"
            print(f"  mapping mode: {mode}")
            print("  active tracked joints: " + ", ".join(
                sorted(args.active_joint_names)))
            print(
                "  joint                 actual     source    applied   difference"
            )
            maximum_difference = 0.0
            maximum_name = ""
            for source_value, applied_value, (index, name, _limits) in zip(
                    source_target, applied_target, LEFT_ARM):
                actual = float(state.motor_state[index].q)
                difference = abs(float(applied_value) - actual)
                if difference > maximum_difference:
                    maximum_difference = difference
                    maximum_name = name
                print(
                    f"  {name:<20}"
                    f"{math.degrees(actual):+8.1f} deg"
                    f"{math.degrees(source_value):+9.1f} deg"
                    f"{math.degrees(applied_value):+9.1f} deg"
                    f"{math.degrees(difference):9.1f} deg"
                )

            maximum_difference_deg = math.degrees(maximum_difference)
            if maximum_difference_deg > args.max_excursion_deg:
                print(
                    "DRY-RUN RESULT: FAIL - "
                    f"{maximum_name} differs by {maximum_difference_deg:.1f} deg; "
                    f"limit is {args.max_excursion_deg:.1f} deg."
                )
                return 1
            print(
                "DRY-RUN RESULT: PASS - maximum initial difference is "
                f"{maximum_difference_deg:.1f} deg "
                f"(limit {args.max_excursion_deg:.1f} deg)."
            )
            return 0

        run_bridge(args, state_receiver, target_receiver, tracking_records)
        return 0
    except (KeyboardInterrupt, RuntimeError) as error:
        print(f"STOPPED: {error}")
        return 1
    finally:
        target_receiver.close()
        state_receiver.close()
        try:
            path = save_tracking_log(tracking_records, args.log_dir)
            if path is not None:
                print(
                    f"Saved {len(tracking_records)} full-rate real-robot "
                    f"tracking samples to {path}"
                )
        except OSError as error:
            print(f"WARNING: could not save real-robot tracking log: {error}")


if __name__ == "__main__":
    raise SystemExit(main())
