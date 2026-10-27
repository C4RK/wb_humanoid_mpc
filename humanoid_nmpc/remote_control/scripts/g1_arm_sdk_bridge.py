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
import statistics
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
MAX_PITCH_GRAVITY_FF_NM = 3.0
MAX_ROLL_GRAVITY_FF_NM = 3.0
TAKEOVER_DURATION = 20.0
RELEASE_SECONDS_PER_UNIT_WEIGHT = 20.0

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

    held_joint_indices = BODY_AND_HELD_JOINTS + tuple(
        index
        for index, name, _limits in LEFT_ARM
        if name not in args.active_joint_names
    )
    fastest_body_index = max(
        held_joint_indices,
        key=lambda index: abs(state.motor_state[index].dq),
    )
    body_speed = abs(state.motor_state[fastest_body_index].dq)
    if not math.isfinite(body_speed) or body_speed > args.max_body_speed_rad_s:
        raise RuntimeError(
            f"{JOINT_NAMES[fastest_body_index]}[{fastest_body_index}] speed reached "
            f"{state.motor_state[fastest_body_index].dq:+.2f} rad/s; "
            f"limit is {args.max_body_speed_rad_s:.2f} rad/s"
        )

    active_arm_indices = tuple(
        index
        for index, name, _limits in LEFT_ARM
        if name in args.active_joint_names
    )
    fastest_arm_index = max(
        active_arm_indices,
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


def bound_target_to_safe_range(applied_target, initial, max_excursion_by_name):
    """Intersect the per-run excursion envelope with absolute URDF limits."""
    bounded_target = []
    for value, (index, name, limits) in zip(applied_target, LEFT_ARM):
        max_excursion = max_excursion_by_name[name]
        safe_lower = max(limits[0], initial[index] - max_excursion)
        safe_upper = min(limits[1], initial[index] + max_excursion)
        bounded_target.append(min(max(value, safe_lower), safe_upper))
    bounded_target = tuple(bounded_target)
    validate_target(bounded_target)
    return bounded_target


def stable_target_from_samples(samples, active_joint_names,
                               tolerance_rad, minimum_samples=10):
    """Return a median target after verifying a quiet human pose.

    Only active joints participate in the stability gate.  Inactive joints are
    still included in the returned five-value target so the normal mapping
    code can keep them at the measured takeover posture.
    """
    if len(samples) < minimum_samples:
        raise RuntimeError(
            "too few fresh target samples during the alignment stability "
            f"window ({len(samples)} received; need {minimum_samples})"
        )

    representative = tuple(
        float(statistics.median(sample[column] for sample in samples))
        for column in range(len(LEFT_ARM))
    )
    for column, (_index, name, _limits) in enumerate(LEFT_ARM):
        if name not in active_joint_names:
            continue
        maximum_deviation = max(
            abs(float(sample[column]) - representative[column])
            for sample in samples
        )
        if maximum_deviation > tolerance_rad:
            raise RuntimeError(
                f"human {name} target was not stable during takeover "
                f"(maximum deviation {math.degrees(maximum_deviation):.1f} deg; "
                f"limit {math.degrees(tolerance_rad):.1f} deg)"
            )
    return representative


def maximum_active_target_difference(first, second, active_joint_names):
    """Return the largest named active-joint difference between two targets."""
    differences = [
        (abs(float(a) - float(b)), name)
        for a, b, (_index, name, _limits) in zip(first, second, LEFT_ARM)
        if name in active_joint_names
    ]
    return max(differences, default=(0.0, "none"))


def step_desired_arm_toward(desired, target, max_step):
    """Rate-limit one control-period step toward a five-joint arm target."""
    for value, (index, _name, _limits) in zip(target, LEFT_ARM):
        difference = float(value) - float(desired[index])
        desired[index] += min(max(difference, -max_step), max_step)


def step_desired_arm_toward_with_tracking_guard(
        desired, target, actual, active_joint_names, max_step,
        soft_tracking_error):
    """Rate-limit targets and pause motion before following error is unsafe.

    The hard tracking-error check remains authoritative.  This softer guard
    only removes the part of a step that would move an active command farther
    away from the measured joint once the remaining error headroom is used.
    Motion toward the measured position is always allowed, so the robot can
    catch up instead of accumulating error until the hard stop fires.

    Returns a mapping of joints whose outward step was limited to their
    pre-step absolute tracking error.
    """
    guarded = {}
    for value, (index, name, _limits) in zip(target, LEFT_ARM):
        difference = float(value) - float(desired[index])
        step = min(max(difference, -max_step), max_step)
        if name in active_joint_names and abs(step) > 0.0:
            error = float(desired[index]) - float(actual[index])
            proposed_error = error + step
            moving_farther_away = abs(proposed_error) > abs(error)
            if moving_farther_away and abs(proposed_error) > soft_tracking_error:
                remaining_headroom = max(
                    0.0, soft_tracking_error - abs(error))
                step = math.copysign(
                    min(abs(step), remaining_headroom), step)
                guarded[name] = abs(error)
        desired[index] += step
    return guarded


def pitch_gravity_feedforward_nm(pitch_rad: float, peak_torque_nm: float) -> float:
    """Return a conservative shoulder-pitch gravity feedforward torque.

    G1 pitch zero is the approximately vertical-down arm configuration.  The
    observed forward-raise direction is negative, so ``peak*sin(q)`` supplies
    negative lifting torque for a negative pitch target and naturally returns
    to zero near the down pose.  ``peak_torque_nm`` is the magnitude that would
    be applied at +/-90 degrees; the commissioning CLI limits it separately.
    """
    return float(peak_torque_nm) * math.sin(float(pitch_rad))


def roll_gravity_feedforward_nm(roll_rad: float, down_reference_rad: float,
                                peak_torque_nm: float) -> float:
    """Return conservative shoulder-roll gravity feedforward torque.

    Shoulder-roll joint zero is not the observed vertical-down posture on the
    real G1.  Use the measured roll angle captured at takeover as the zero-
    torque DOWN reference.  Positive roll raises the left arm outward in the
    commissioned mapping, so the sine model supplies positive lifting torque.
    """
    relative_roll = float(roll_rad) - float(down_reference_rad)
    return float(peak_torque_nm) * math.sin(relative_roll)


def fill_command(command, desired, weight: float,
                 pitch_gravity_ff_nm: float = 0.0,
                 roll_gravity_ff_nm: float = 0.0,
                 roll_gravity_zero_rad: float = 0.0) -> None:
    pitch_index = LEFT_ARM[0][0]
    roll_index = LEFT_ARM[1][0]
    for index in CONTROLLED_JOINTS:
        motor = command.motor_cmd[index]
        motor.tau = 0.0
        if index == pitch_index and pitch_gravity_ff_nm > 0.0:
            # Ramp the explicit feedforward together with arm_sdk ownership so
            # takeover and release cannot introduce a torque step.
            motor.tau = weight * pitch_gravity_feedforward_nm(
                desired[index], pitch_gravity_ff_nm)
        elif index == roll_index and roll_gravity_ff_nm > 0.0:
            motor.tau = weight * roll_gravity_feedforward_nm(
                desired[index], roll_gravity_zero_rad,
                roll_gravity_ff_nm)
        motor.q = desired[index]
        motor.dq = 0.0
        motor.kp = KP
        motor.kd = KD
    command.motor_cmd[ARM_SDK_WEIGHT].q = min(max(weight, 0.0), 1.0)


def publish_command(publisher, command, crc, desired, weight: float,
                    pitch_gravity_ff_nm: float = 0.0,
                    roll_gravity_ff_nm: float = 0.0,
                    roll_gravity_zero_rad: float = 0.0) -> None:
    fill_command(
        command, desired, weight, pitch_gravity_ff_nm,
        roll_gravity_ff_nm, roll_gravity_zero_rad)
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
        "command_pitch_gravity_ff_nm",
        "command_roll_gravity_ff_nm",
        "roll_gravity_zero_rad",
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
                        applied_target, pitch_gravity_ff_nm=0.0,
                        roll_gravity_ff_nm=0.0,
                        roll_gravity_zero_rad=0.0) -> None:
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
        pitch_gravity_feedforward_nm(
            desired[LEFT_ARM[0][0]], pitch_gravity_ff_nm),
        roll_gravity_feedforward_nm(
            desired[LEFT_ARM[1][0]], roll_gravity_zero_rad,
            roll_gravity_ff_nm),
        float(roll_gravity_zero_rad),
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
    roll_gravity_zero_rad = float(initial[LEFT_ARM[1][0]])
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

    for value, (index, name, _limits) in zip(first_target, LEFT_ARM):
        difference = abs(value - initial[index])
        max_excursion = args.max_excursion_by_name[name]
        if difference > max_excursion:
            raise RuntimeError(
                f"initial {name} target differs by {math.degrees(difference):.1f} deg; "
                f"limit is {math.degrees(max_excursion):.1f} deg"
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
    print(
        "Per-joint excursion limits: "
        + " ".join(
            f"{name}={math.degrees(args.max_excursion_by_name[name]):.1f}"
            for name in LEFT_ARM_NAMES
        )
        + " deg"
    )
    print(
        "Tracking lag guard: command slowdown begins at "
        f"{math.degrees(args.soft_tracking_error_rad):.1f} deg; hard stop "
        f"remains {args.max_tracking_error_deg:.1f} deg."
    )
    if args.pitch_gravity_ff_nm > 0.0:
        print(
            "Pitch gravity feedforward: ENABLED; peak="
            f"{args.pitch_gravity_ff_nm:.2f} N m at +/-90 deg, "
            "tau=peak*sin(commanded pitch)."
        )
    else:
        print("Pitch gravity feedforward: disabled (0.00 N m).")
    if args.roll_gravity_ff_nm > 0.0:
        print(
            "Roll gravity feedforward: ENABLED; peak="
            f"{args.roll_gravity_ff_nm:.2f} N m at 90 deg from DOWN, "
            f"DOWN zero={math.degrees(roll_gravity_zero_rad):+.2f} deg, "
            "tau=peak*sin(commanded roll - DOWN zero)."
        )
    else:
        print("Roll gravity feedforward: disabled (0.00 N m).")
    if args.absolute_alignment:
        print(
            "ABSOLUTE ALIGNMENT is ENABLED. Keep the current human pose still "
            "through takeover and robot alignment."
        )
        confirmation_word = "ALIGN"
    else:
        confirmation_word = "TRACK"
    confirmation = input(
        f"Type {confirmation_word} and press Enter to enable physical arm tracking: "
    )
    if confirmation != confirmation_word:
        print("Tracking cancelled. No command publisher was created.")
        return

    publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
    publisher.Init()
    command = unitree_hg_msg_dds__LowCmd_()
    crc = CRC()
    max_step = math.radians(args.max_rate_deg_s) * CONTROL_PERIOD
    alignment_max_step = (
        math.radians(args.alignment_rate_deg_s) * CONTROL_PERIOD
    )
    current_weight = 0.0
    alignment_samples = []
    last_alignment_sequence = None

    try:
        started = time.monotonic()
        next_tick = started
        next_log = started
        while time.monotonic() - started < TAKEOVER_DURATION:
            now = time.monotonic()
            progress = (now - started) / TAKEOVER_DURATION
            target_receiver.update()
            if target_receiver.age() > args.target_timeout:
                raise RuntimeError(
                    "target stream became stale during arm_sdk takeover "
                    f"({target_receiver.age():.3f} s)"
                )
            validate_target(target_receiver.target)
            if (args.absolute_alignment and
                    now - started >=
                    TAKEOVER_DURATION - args.alignment_stability_seconds and
                    target_receiver.sequence != last_alignment_sequence):
                alignment_samples.append(tuple(target_receiver.target))
                last_alignment_sequence = target_receiver.sequence
            state, state_time = state_receiver.snapshot()
            if state is None or now - state_time > 0.2:
                raise RuntimeError("LowState stream timed out during arm_sdk takeover")
            validate_tracking_state(state, desired, args)
            current_weight = smoothstep(progress)
            publish_command(
                publisher, command, crc, desired, current_weight,
                args.pitch_gravity_ff_nm,
                args.roll_gravity_ff_nm,
                roll_gravity_zero_rad,
            )
            if now >= next_log:
                print(
                    f"takeover weight={current_weight:.2f}  "
                    f"target_age={target_receiver.age() * 1000.0:.0f} ms"
                )
                next_log = now + 1.0
            next_tick += CONTROL_PERIOD
            time.sleep(max(0.0, next_tick - time.monotonic()))

        # Relative tracking should start from the human pose at the end of the
        # slow takeover.  Small posture changes while control authority is
        # being blended in must not become queued robot motion.
        target_receiver.update()
        if target_receiver.age() > args.target_timeout:
            raise RuntimeError(
                "target stream became stale at the end of arm_sdk takeover "
                f"({target_receiver.age():.3f} s)"
            )
        validate_target(target_receiver.target)
        if args.relative_to_start:
            baseline_target = tuple(target_receiver.target)
            print(
                "Relative target baseline refreshed at the end of takeover; "
                "tracking starts from the current human pose."
            )

        if args.absolute_alignment:
            frozen_source_target = stable_target_from_samples(
                alignment_samples,
                args.active_joint_names,
                math.radians(args.alignment_stability_deg),
            )
            frozen_alignment_target = map_target_to_robot(
                frozen_source_target,
                frozen_source_target,
                initial,
                False,
                args.active_joint_names,
            )
            validate_target(frozen_alignment_target)
            for value, (index, name, _limits) in zip(
                    frozen_alignment_target, LEFT_ARM):
                difference = abs(value - initial[index])
                limit = args.max_excursion_by_name[name]
                if difference > limit:
                    raise RuntimeError(
                        f"stable absolute {name} target differs from the "
                        f"takeover pose by {math.degrees(difference):.1f} deg; "
                        f"limit is {math.degrees(limit):.1f} deg"
                    )

            alignment_text = " ".join(
                f"{name}={math.degrees(value):+.1f}"
                for value, (_index, name, _limits) in zip(
                    frozen_alignment_target, LEFT_ARM)
                if name in args.active_joint_names
            )
            print(
                "Takeover complete. Aligning the robot to the frozen absolute "
                f"target at no more than {args.alignment_rate_deg_s:.1f} deg/s: "
                f"{alignment_text} deg"
            )
            print("Keep the human pose still until ABSOLUTE ALIGNMENT COMPLETE.")

            alignment_started = time.monotonic()
            aligned_since = None
            next_tick = alignment_started
            next_log = alignment_started
            while True:
                now = time.monotonic()
                if now - alignment_started > args.alignment_timeout:
                    raise RuntimeError(
                        "absolute alignment timed out after "
                        f"{args.alignment_timeout:.1f} s"
                    )

                target_receiver.update()
                if target_receiver.age() > args.target_timeout:
                    raise RuntimeError(
                        "target stream became stale during absolute alignment "
                        f"({target_receiver.age():.3f} s)"
                    )
                validate_target(target_receiver.target)
                source_motion, source_motion_name = (
                    maximum_active_target_difference(
                        target_receiver.target,
                        frozen_source_target,
                        args.active_joint_names,
                    )
                )
                if source_motion > math.radians(args.alignment_hold_deg):
                    raise RuntimeError(
                        f"human {source_motion_name} moved by "
                        f"{math.degrees(source_motion):.1f} deg during absolute "
                        f"alignment; limit is {args.alignment_hold_deg:.1f} deg"
                    )

                state, state_time = state_receiver.snapshot()
                if state is None or now - state_time > 0.2:
                    raise RuntimeError(
                        "LowState stream timed out during absolute alignment"
                    )
                validate_tracking_state(state, desired, args)

                step_desired_arm_toward(
                    desired, frozen_alignment_target, alignment_max_step)
                publish_command(
                    publisher, command, crc, desired, 1.0,
                    args.pitch_gravity_ff_nm,
                    args.roll_gravity_ff_nm,
                    roll_gravity_zero_rad,
                )
                current_weight = 1.0
                append_tracking_log(
                    tracking_records, state, desired, target_receiver,
                    frozen_alignment_target,
                    args.pitch_gravity_ff_nm,
                    args.roll_gravity_ff_nm,
                    roll_gravity_zero_rad,
                )

                actual_error, actual_error_name = max(
                    (
                        abs(state.motor_state[index].q - value),
                        name,
                    )
                    for value, (index, name, _limits) in zip(
                        frozen_alignment_target, LEFT_ARM)
                    if name in args.active_joint_names
                )
                command_error, _command_error_name = max(
                    (
                        abs(desired[index] - value),
                        name,
                    )
                    for value, (index, name, _limits) in zip(
                        frozen_alignment_target, LEFT_ARM)
                    if name in args.active_joint_names
                )
                within_tolerance = (
                    actual_error <= math.radians(args.alignment_tolerance_deg)
                    and command_error <= math.radians(0.1)
                )
                if within_tolerance:
                    if aligned_since is None:
                        aligned_since = now
                    elif now - aligned_since >= args.alignment_settle_seconds:
                        break
                else:
                    aligned_since = None

                if now >= next_log:
                    print(
                        "aligning absolute target: "
                        f"worst actual error={math.degrees(actual_error):.1f} deg "
                        f"({actual_error_name}); "
                        f"human hold error={math.degrees(source_motion):.1f} deg"
                    )
                    next_log = now + 1.0

                next_tick += CONTROL_PERIOD
                time.sleep(max(0.0, next_tick - time.monotonic()))

            print(
                "ABSOLUTE ALIGNMENT COMPLETE; the same retargeted pose now maps "
                "to the same robot joint target."
            )

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
            bounded_target = bound_target_to_safe_range(
                applied_target, initial, args.max_excursion_by_name)
            actual_positions = {
                index: state.motor_state[index].q
                for index in CONTROLLED_JOINTS
            }
            lag_guarded = step_desired_arm_toward_with_tracking_guard(
                desired,
                bounded_target,
                actual_positions,
                args.active_joint_names,
                max_step,
                args.soft_tracking_error_rad,
            )

            publish_command(
                publisher, command, crc, desired, 1.0,
                args.pitch_gravity_ff_nm,
                args.roll_gravity_ff_nm,
                roll_gravity_zero_rad,
            )
            current_weight = 1.0
            append_tracking_log(
                tracking_records, state, desired, target_receiver,
                applied_target,
                args.pitch_gravity_ff_nm,
                args.roll_gravity_ff_nm,
                roll_gravity_zero_rad,
            )

            if now >= next_log:
                pairs = []
                for source_value, applied_value, bounded_value, (
                        index, name, _limits) in zip(
                            target_receiver.target,
                            applied_target,
                            bounded_target,
                            LEFT_ARM):
                    if name in args.active_joint_names:
                        clipped = (
                            abs(float(applied_value) - float(bounded_value)) > 1e-9
                        )
                        pairs.append(
                            f"{name} "
                            f"{math.degrees(source_value):+.1f}/"
                            f"{math.degrees(applied_value):+.1f}/"
                            f"{math.degrees(bounded_value):+.1f}/"
                            f"{math.degrees(desired[index]):+.1f}/"
                            f"{math.degrees(state.motor_state[index].q):+.1f}"
                            f" err={math.degrees(abs(desired[index] - state.motor_state[index].q)):.1f}"
                            + (
                                f" ff={pitch_gravity_feedforward_nm(desired[index], args.pitch_gravity_ff_nm):+.2f}Nm"
                                if name == "pitch" and args.pitch_gravity_ff_nm > 0.0
                                else ""
                            )
                            + (
                                " ff="
                                f"{roll_gravity_feedforward_nm(desired[index], roll_gravity_zero_rad, args.roll_gravity_ff_nm):+.2f}Nm"
                                if name == "roll" and args.roll_gravity_ff_nm > 0.0
                                else ""
                            )
                            + (" LAG-GUARD" if name in lag_guarded else "")
                            + (" CLIPPED" if clipped else "")
                        )
                print(
                    "active source/applied/bounded/desired/actual deg: "
                    + "  ".join(pairs)
                )
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
        release_duration = max(
            3.0,
            RELEASE_SECONDS_PER_UNIT_WEIGHT * release_start_weight,
        )
        started = time.monotonic()
        next_tick = started
        next_log = started
        while time.monotonic() - started < release_duration:
            now = time.monotonic()
            progress = (now - started) / release_duration
            release_weight = release_start_weight * (1.0 - smoothstep(progress))
            publish_command(
                publisher,
                command,
                crc,
                release_pose,
                release_weight,
                args.pitch_gravity_ff_nm,
                args.roll_gravity_ff_nm,
                roll_gravity_zero_rad,
            )
            if now >= next_log:
                print(f"release weight={release_weight:.2f}")
                next_log = now + 1.0
            next_tick += CONTROL_PERIOD
            time.sleep(max(0.0, next_tick - time.monotonic()))
        for _ in range(5):
            publish_command(
                publisher, command, crc, release_pose, 0.0,
                args.pitch_gravity_ff_nm,
                args.roll_gravity_ff_nm,
                roll_gravity_zero_rad,
            )
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
    parser.add_argument("--target-timeout", type=float, default=0.50)
    parser.add_argument("--max-rate-deg-s", type=float, default=15.0)
    parser.add_argument(
        "--max-excursion-deg",
        type=float,
        default=20.0,
        help=(
            "fallback excursion limit in degrees for every active joint; "
            "a per-joint option overrides it"
        ),
    )
    for joint_name in LEFT_ARM_NAMES:
        parser.add_argument(
            f"--max-excursion-{joint_name}-deg",
            type=float,
            default=None,
            help=(
                f"excursion limit in degrees for {joint_name}; defaults to "
                "--max-excursion-deg"
            ),
        )
    parser.add_argument("--max-tilt-deg", type=float, default=10.0)
    parser.add_argument("--max-body-speed-rad-s", type=float, default=0.3)
    parser.add_argument("--max-observed-arm-speed-deg-s", type=float, default=60.0)
    parser.add_argument("--max-tracking-error-deg", type=float, default=15.0)
    parser.add_argument(
        "--tracking-error-slowdown-margin-deg",
        type=float,
        default=1.0,
        help=(
            "start pausing outward command motion this many degrees before "
            "the hard tracking-error stop"
        ),
    )
    parser.add_argument(
        "--pitch-gravity-ff-nm",
        type=float,
        default=0.0,
        help=(
            "experimental left-shoulder-pitch gravity feedforward amplitude "
            "in N m at +/-90 degrees; default 0 disables it and the initial "
            f"commissioning cap is {MAX_PITCH_GRAVITY_FF_NM:.1f} N m"
        ),
    )
    parser.add_argument(
        "--roll-gravity-ff-nm",
        type=float,
        default=0.0,
        help=(
            "experimental left-shoulder-roll gravity feedforward amplitude "
            "in N m at 90 degrees from the measured takeover/DOWN angle; "
            "default 0 disables it and the initial commissioning cap is "
            f"{MAX_ROLL_GRAVITY_FF_NM:.1f} N m"
        ),
    )
    mapping_group = parser.add_mutually_exclusive_group()
    mapping_group.add_argument(
        "--relative-to-start",
        action="store_true",
        help=(
            "map the first fresh ROS target to the measured robot posture and "
            "track only subsequent target changes"
        ),
    )
    mapping_group.add_argument(
        "--absolute-alignment",
        action="store_true",
        help=(
            "after takeover, slowly align active joints to a stable absolute "
            "ROS target before enabling live absolute tracking"
        ),
    )
    parser.add_argument(
        "--alignment-rate-deg-s",
        type=float,
        default=2.0,
        help="maximum active-joint speed during absolute startup alignment",
    )
    parser.add_argument(
        "--alignment-stability-seconds",
        type=float,
        default=2.0,
        help="quiet target window collected at the end of takeover",
    )
    parser.add_argument(
        "--alignment-stability-deg",
        type=float,
        default=2.0,
        help="maximum target deviation allowed in the quiet window",
    )
    parser.add_argument(
        "--alignment-hold-deg",
        type=float,
        default=4.0,
        help="maximum human target motion allowed while the robot aligns",
    )
    parser.add_argument(
        "--alignment-tolerance-deg",
        type=float,
        default=2.0,
        help="actual robot error required to finish absolute alignment",
    )
    parser.add_argument(
        "--alignment-settle-seconds",
        type=float,
        default=1.0,
        help="time the robot must remain within alignment tolerance",
    )
    parser.add_argument(
        "--alignment-timeout",
        type=float,
        default=90.0,
        help="maximum absolute-alignment time after takeover",
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
        default="~/humanoid_mpc_ws/src/wb_humanoid_mpc/calibration_logs",
        help="directory for the full-rate real-robot tracking CSV",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="publish to rt/arm_sdk; without this flag the bridge is read-only",
    )
    args = parser.parse_args()
    args.active_joint_names = frozenset(args.active_joints)

    if (not math.isfinite(args.pitch_gravity_ff_nm)
            or args.pitch_gravity_ff_nm < 0.0
            or args.pitch_gravity_ff_nm > MAX_PITCH_GRAVITY_FF_NM):
        parser.error(
            "--pitch-gravity-ff-nm must be finite and in [0, "
            f"{MAX_PITCH_GRAVITY_FF_NM:.1f}]"
        )
    if (args.pitch_gravity_ff_nm > 0.0
            and "pitch" not in args.active_joint_names):
        parser.error(
            "--pitch-gravity-ff-nm requires pitch in --active-joints"
        )
    if (not math.isfinite(args.roll_gravity_ff_nm)
            or args.roll_gravity_ff_nm < 0.0
            or args.roll_gravity_ff_nm > MAX_ROLL_GRAVITY_FF_NM):
        parser.error(
            "--roll-gravity-ff-nm must be finite and in [0, "
            f"{MAX_ROLL_GRAVITY_FF_NM:.1f}]"
        )
    if (args.roll_gravity_ff_nm > 0.0
            and "roll" not in args.active_joint_names):
        parser.error(
            "--roll-gravity-ff-nm requires roll in --active-joints"
        )

    if args.execute and not (args.relative_to_start or args.absolute_alignment):
        parser.error(
            "--execute requires an explicit mapping mode: use "
            "--relative-to-start for commissioning or --absolute-alignment "
            "for pose-following"
        )

    positive_limits = {
        "--max-rate-deg-s": args.max_rate_deg_s,
        "--max-excursion-deg": args.max_excursion_deg,
        "--max-tilt-deg": args.max_tilt_deg,
        "--max-body-speed-rad-s": args.max_body_speed_rad_s,
        "--max-observed-arm-speed-deg-s": args.max_observed_arm_speed_deg_s,
        "--max-tracking-error-deg": args.max_tracking_error_deg,
        "--tracking-error-slowdown-margin-deg": (
            args.tracking_error_slowdown_margin_deg),
        "--alignment-rate-deg-s": args.alignment_rate_deg_s,
        "--alignment-stability-seconds": args.alignment_stability_seconds,
        "--alignment-stability-deg": args.alignment_stability_deg,
        "--alignment-hold-deg": args.alignment_hold_deg,
        "--alignment-tolerance-deg": args.alignment_tolerance_deg,
        "--alignment-settle-seconds": args.alignment_settle_seconds,
        "--alignment-timeout": args.alignment_timeout,
    }
    for name, value in positive_limits.items():
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"{name} must be a positive finite value")
    if args.alignment_stability_seconds >= TAKEOVER_DURATION:
        parser.error(
            f"--alignment-stability-seconds must be less than the "
            f"{TAKEOVER_DURATION:.0f}-second takeover duration"
        )
    if (args.tracking_error_slowdown_margin_deg >=
            args.max_tracking_error_deg):
        parser.error(
            "--tracking-error-slowdown-margin-deg must be smaller than "
            "--max-tracking-error-deg"
        )
    args.soft_tracking_error_rad = math.radians(
        args.max_tracking_error_deg
        - args.tracking_error_slowdown_margin_deg)

    args.max_excursion_by_name = {}
    for joint_name in LEFT_ARM_NAMES:
        option_name = f"--max-excursion-{joint_name}-deg"
        override = getattr(args, f"max_excursion_{joint_name}_deg")
        if override is not None:
            if not math.isfinite(override) or override <= 0.0:
                parser.error(f"{option_name} must be a positive finite value")
            limit_deg = override
        else:
            limit_deg = args.max_excursion_deg
        args.max_excursion_by_name[joint_name] = math.radians(limit_deg)

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
            if args.relative_to_start:
                mode = "relative-to-start"
            elif args.absolute_alignment:
                mode = "absolute-alignment"
            else:
                mode = "absolute preview"
            print(f"  mapping mode: {mode}")
            print("  active tracked joints: " + ", ".join(
                sorted(args.active_joint_names)))
            print(
                "  joint                 actual     source    applied   difference     limit"
            )
            violations = []
            for source_value, applied_value, (index, name, _limits) in zip(
                    source_target, applied_target, LEFT_ARM):
                actual = float(state.motor_state[index].q)
                difference = abs(float(applied_value) - actual)
                limit = args.max_excursion_by_name[name]
                if difference > limit:
                    violations.append((name, difference, limit))
                print(
                    f"  {name:<20}"
                    f"{math.degrees(actual):+8.1f} deg"
                    f"{math.degrees(source_value):+9.1f} deg"
                    f"{math.degrees(applied_value):+9.1f} deg"
                    f"{math.degrees(difference):9.1f} deg"
                    f"{math.degrees(limit):9.1f} deg"
                )

            if violations:
                violation_text = ", ".join(
                    f"{name} {math.degrees(difference):.1f}>"
                    f"{math.degrees(limit):.1f} deg"
                    for name, difference, limit in violations
                )
                print(
                    "DRY-RUN RESULT: FAIL - "
                    f"initial differences exceed limits: {violation_text}."
                )
                return 1
            print(
                "DRY-RUN RESULT: PASS - all initial differences are within "
                "their per-joint excursion limits."
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
