#!/usr/bin/env python3

"""Retarget synchronized upper-arm and forearm WEMT poses to the G1 arm."""

from datetime import datetime, timezone
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from remote_control.dual_calibration_node import (
    CALIBRATION_LOG_DIR,
    load_dual_calibration,
)
from remote_control.dual_retargeting_kinematics import (
    ROTATION_AS_REPORTED,
    apply_rotation_convention,
    dual_joint_angles,
    dual_joint_angles_from_directions,
    dual_joint_angles_from_segment_vectors,
)
from remote_control.retargeting_kinematics import quaternion_to_matrix
from remote_control.retargeting_kinematics import shoulder_pitch_roll_continuous
from remote_control.retargeting_node import (
    DEFAULT_JOINT_STATE,
    JOINT_LIMITS,
    LEFT_ARM_INDICES,
)


ARM_JOINT_KEYS = [
    'left_shoulder_pitch_joint',
    'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint',
    'left_elbow_joint',
    'left_wrist_roll_joint',
]


def _stamp_ns(msg: PoseStamped) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def _message_pose(msg: PoseStamped):
    position = np.array(
        [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
    quaternion = np.array(
        [msg.pose.orientation.w, msg.pose.orientation.x,
         msg.pose.orientation.y, msg.pose.orientation.z], dtype=float)
    return position, quaternion_to_matrix(quaternion)


def _message_quaternion(msg: PoseStamped):
    return np.array(
        [msg.pose.orientation.w, msg.pose.orientation.x,
         msg.pose.orientation.y, msg.pose.orientation.z], dtype=float)


class DualRetargetingNode(Node):
    def __init__(self):
        super().__init__('dual_retargeting_node')
        calibration = load_dual_calibration()

        self.calibration_mode = calibration['mode']
        self.calibration_dry_run_only = bool(
            calibration.get('dry_run_only', False))
        self.forearm_mount = self._matrix(
            calibration, 'forearm_sensor_mount_rotation')
        self.use_hybrid = self.calibration_mode == 'dual_receiver_hybrid'
        if self.use_hybrid:
            self.upper_position_align = self._matrix(
                calibration, 'upper_position_rotation_tx_to_body')
            self.upper_shoulder_center = self._vector(
                calibration, 'upper_shoulder_center_tx_m')
            self.upper_receiver_radius = float(
                calibration['upper_receiver_radius_m'])
            if not 0.08 <= self.upper_receiver_radius <= 0.35:
                raise ValueError('Invalid upper receiver radius')
            self.forearm_align = self._matrix(
                calibration, 'forearm_rotation_tx_to_body')
            self.forearm_axis_local = self._vector(
                calibration, 'forearm_axis_local')
        else:
            self.R_align = self._matrix(calibration, 'rotation_tx_to_body')
            self.upper_mount = self._matrix(
                calibration, 'upper_sensor_mount_rotation')
        self.use_segment_directions = (
            self.calibration_mode == 'dual_receiver_direction')
        if self.use_segment_directions:
            self.upper_axis_local = self._vector(calibration, 'upper_axis_local')
            self.forearm_axis_local = self._vector(calibration, 'forearm_axis_local')
        self.use_functional_centres = (
            self.calibration_mode == 'dual_receiver_functional')
        if self.use_functional_centres:
            self.shoulder_center = self._vector(calibration, 'shoulder_center_tx_m')
            self.upper_to_shoulder = self._vector(calibration, 'upper_sensor_to_shoulder_m')
            self.upper_to_elbow = self._vector(calibration, 'upper_sensor_to_elbow_m')
            self.forearm_to_elbow = self._vector(calibration, 'forearm_sensor_to_elbow_m')
        self.rotation_convention = calibration.get(
            'rotation_convention', ROTATION_AS_REPORTED)

        self.declare_parameter('filter_time_constant_s', 0.06)
        self.declare_parameter('max_joint_speed_rad_s', 2.5)
        self.declare_parameter('max_shoulder_drift_m', 0.05)
        self.declare_parameter('max_elbow_mismatch_m', 0.05)
        self.declare_parameter('max_upper_radius_error_m', 0.05)
        self.declare_parameter('watchdog_timeout_s', 0.25)
        # Shoulder axial rotation and forearm twist become poorly conditioned
        # before the arm is completely straight.  Freeze both once the
        # perpendicular forearm component falls below 0.55 (about 57 degrees
        # in this elbow-angle convention) so measurement noise is not turned
        # into a yaw/wrist excursion during elbow extension.
        self.declare_parameter('min_yaw_observability', 0.55)
        # Pitch is undefined at shoulder roll +/-90 degrees.  Enter a protected
        # state before the direction becomes ill-conditioned, then require a
        # larger X/Z projection before reacquiring pitch.  This hysteresis keeps
        # position noise from selecting a different shoulder solution during a
        # side raise.
        self.declare_parameter('min_pitch_observability', 0.5)
        self.declare_parameter('pitch_reacquire_observability', 0.65)
        self.declare_parameter('publish_joint_targets', False)
        self.declare_parameter('record_diagnostics', True)
        self.declare_parameter('diagnostic_log_dir', CALIBRATION_LOG_DIR)
        self.filter_tau = max(0.0, float(self.get_parameter('filter_time_constant_s').value))
        self.max_speed = max(0.1, float(self.get_parameter('max_joint_speed_rad_s').value))
        self.max_shoulder_drift = max(0.0, float(self.get_parameter('max_shoulder_drift_m').value))
        self.max_elbow_mismatch = max(0.0, float(self.get_parameter('max_elbow_mismatch_m').value))
        self.max_upper_radius_error = max(
            0.01, float(self.get_parameter('max_upper_radius_error_m').value))
        self.watchdog_timeout = max(0.05, float(self.get_parameter('watchdog_timeout_s').value))
        self.min_yaw_observability = float(np.clip(
            self.get_parameter('min_yaw_observability').value, 0.05, 0.95))
        self.min_pitch_observability = float(np.clip(
            self.get_parameter('min_pitch_observability').value, 0.05, 0.85))
        self.pitch_reacquire_observability = float(np.clip(
            self.get_parameter('pitch_reacquire_observability').value,
            self.min_pitch_observability + 0.05, 0.99))
        self.publish_joint_targets = bool(
            self.get_parameter('publish_joint_targets').value)
        self.record_diagnostics = bool(
            self.get_parameter('record_diagnostics').value)
        self.diagnostic_log_dir = str(
            self.get_parameter('diagnostic_log_dir').value)
        if self.calibration_dry_run_only and self.publish_joint_targets:
            raise ValueError(
                'This calibration is marked dry-run-only because its forearm '
                'reference error exceeded 15 degrees. Recalibrate before '
                'publishing robot joint targets.')

        quality = calibration.get('quality', {})
        self.get_logger().info(
            'Loaded dual calibration: '
            f'mode={self.calibration_mode}, '
            f'rotation={self.rotation_convention}, '
            f"reference max={quality.get('reference_max_error_deg', float('nan')):.1f} deg, "
            f'dry_run_only={self.calibration_dry_run_only}')

        self._pending_upper = {}
        self._pending_forearm = {}
        self._joint_state = list(DEFAULT_JOINT_STATE)
        self._raw_angles = np.zeros(5)
        self._filtered_angles = np.zeros(5)
        self._pitch_singularity_locked = False
        self._pitch_observability = 1.0
        self._last_stamp_ns = None
        self._last_pair_wall = None
        self._diagnostic_records = []
        self._diagnostic_saved = False
        self._diagnostic_started_at = datetime.now(timezone.utc)

        self.create_subscription(
            PoseStamped, '/em/upper_arm_pose', self._on_upper_pose, 20)
        self.create_subscription(
            PoseStamped, '/em/forearm_pose', self._on_forearm_pose, 20)
        self._publisher = self.create_publisher(JointState, '/arm_joint_target', 10)
        self._watchdog_timer = self.create_timer(0.1, self._watchdog)
        self.get_logger().info(
            'Dual retargeting ready. Hold the calibration reference pose before '
            'starting MPC. Joint-target publishing is '
            f'{"ENABLED" if self.publish_joint_targets else "DISABLED (dry run)"}.')
        if self.record_diagnostics:
            self.get_logger().info(
                'Full-rate diagnostic recording is ENABLED. Press Ctrl+C to '
                f'save an NPZ file in {self.diagnostic_log_dir}.')

    def _resolve_continuous_pitch(self, upper_direction):
        was_locked = self._pitch_singularity_locked
        pitch, _, locked, observability = shoulder_pitch_roll_continuous(
            upper_direction,
            previous_pitch=float(self._raw_angles[0]),
            pitch_locked=was_locked,
            lock_below=self.min_pitch_observability,
            release_above=self.pitch_reacquire_observability)
        self._pitch_singularity_locked = locked
        self._pitch_observability = observability
        if locked != was_locked:
            action = 'holding' if locked else 'reacquiring'
            self.get_logger().info(
                f'Shoulder pitch is {action} a continuous solution '
                f'(observability={observability:.2f}).')
        return pitch

    @staticmethod
    def _vector(calibration, key):
        value = np.asarray(calibration[key], dtype=float)
        if value.shape != (3,) or not np.all(np.isfinite(value)):
            raise ValueError(f'Invalid calibration vector: {key}')
        return value

    @staticmethod
    def _matrix(calibration, key):
        value = np.asarray(calibration[key], dtype=float)
        if value.shape != (3, 3) or not np.all(np.isfinite(value)):
            raise ValueError(f'Invalid calibration matrix: {key}')
        orthogonality_error = float(np.linalg.norm(value.T @ value - np.eye(3)))
        if orthogonality_error > 1e-3 or np.linalg.det(value) < 0.99:
            raise ValueError(f'Calibration matrix {key} is not a proper rotation')
        return value

    def _on_upper_pose(self, msg: PoseStamped):
        key = _stamp_ns(msg)
        self._pending_upper[key] = msg
        self._try_pair(key)

    def _on_forearm_pose(self, msg: PoseStamped):
        key = _stamp_ns(msg)
        self._pending_forearm[key] = msg
        self._try_pair(key)

    def _try_pair(self, key: int):
        upper = self._pending_upper.get(key)
        forearm = self._pending_forearm.get(key)
        if upper is None or forearm is None:
            self._trim_pending()
            return
        self._pending_upper.pop(key, None)
        self._pending_forearm.pop(key, None)
        self._trim_pending()
        self._process_pair(upper, forearm, key)

    def _trim_pending(self):
        for pending in (self._pending_upper, self._pending_forearm):
            while len(pending) > 30:
                pending.pop(min(pending))

    def _process_pair(self, upper_msg: PoseStamped, forearm_msg: PoseStamped, stamp_ns: int):
        if upper_msg.header.frame_id != forearm_msg.header.frame_id:
            self.get_logger().warn('Receiver frame_id mismatch; pair dropped.')
            return
        p_u, R_u = _message_pose(upper_msg)
        p_f, R_f = _message_pose(forearm_msg)
        q_u_reported = _message_quaternion(upper_msg)
        q_f_reported = _message_quaternion(forearm_msg)
        R_u = apply_rotation_convention(R_u, self.rotation_convention)
        R_f = apply_rotation_convention(R_f, self.rotation_convention)

        shoulder_drift = None
        elbow_mismatch = None
        upper_radius_error = None
        upper_direction = np.full(3, np.nan)
        forearm_direction = np.full(3, np.nan)
        if self.use_functional_centres:
            shoulder_now = p_u + R_u @ self.upper_to_shoulder
            elbow_from_upper = p_u + R_u @ self.upper_to_elbow
            elbow_from_forearm = p_f + R_f @ self.forearm_to_elbow
            shoulder_drift = float(np.linalg.norm(shoulder_now - self.shoulder_center))
            elbow_mismatch = float(np.linalg.norm(elbow_from_upper - elbow_from_forearm))
            if shoulder_drift > self.max_shoulder_drift:
                self.get_logger().warn(
                    f'Shoulder consistency error {shoulder_drift*100:.1f} cm; '
                    'transmitter/upper receiver may have shifted. Frame dropped.',
                    throttle_duration_sec=1.0)
                return
            if elbow_mismatch > self.max_elbow_mismatch:
                self.get_logger().warn(
                    f'Elbow consistency error {elbow_mismatch*100:.1f} cm; '
                    'a receiver may have shifted. Frame dropped.',
                    throttle_duration_sec=1.0)
                return

        if self.use_hybrid:
            upper_offset = p_u - self.upper_shoulder_center
            upper_radius = float(np.linalg.norm(upper_offset))
            if upper_radius < 1e-6:
                self.get_logger().warn(
                    'Upper receiver is at the fitted shoulder centre; frame dropped.',
                    throttle_duration_sec=1.0)
                return
            upper_radius_error = abs(
                upper_radius - self.upper_receiver_radius)
            if upper_radius_error > self.max_upper_radius_error:
                self.get_logger().warn(
                    f'Upper position radius error {upper_radius_error*1000:.0f} mm; '
                    'transmitter or upper receiver may have shifted. Frame dropped.',
                    throttle_duration_sec=1.0)
                return
            upper_direction = self.upper_position_align @ upper_offset
            forearm_aligned = self.forearm_align @ R_f
            forearm_direction = forearm_aligned @ self.forearm_axis_local
            continuous_pitch = self._resolve_continuous_pitch(upper_direction)
            raw = dual_joint_angles_from_segment_vectors(
                upper_direction,
                forearm_direction,
                forearm_aligned,
                self.forearm_mount,
                previous_pitch=continuous_pitch,
                previous_yaw=float(self._raw_angles[2]),
                previous_wrist=float(self._raw_angles[4]),
                min_pitch_observability=self.min_pitch_observability,
                min_yaw_observability=self.min_yaw_observability,
                pitch_override=continuous_pitch)
        elif self.use_segment_directions:
            upper_aligned = self.R_align @ R_u
            upper_direction = upper_aligned @ self.upper_axis_local
            continuous_pitch = self._resolve_continuous_pitch(upper_direction)
            raw = dual_joint_angles_from_directions(
                R_u, R_f, self.R_align,
                self.upper_axis_local, self.forearm_axis_local,
                self.forearm_mount,
                previous_pitch=continuous_pitch,
                previous_yaw=float(self._raw_angles[2]),
                previous_wrist=float(self._raw_angles[4]),
                min_pitch_observability=self.min_pitch_observability,
                min_yaw_observability=self.min_yaw_observability,
                pitch_override=continuous_pitch)
        else:
            raw = dual_joint_angles(
                R_u, R_f, self.R_align, self.upper_mount, self.forearm_mount,
                previous_yaw=float(self._raw_angles[2]),
                previous_wrist=float(self._raw_angles[4]))
        self._raw_angles = raw

        bounded = np.array([
            np.clip(value, *JOINT_LIMITS[joint_name])
            for value, joint_name in zip(raw, ARM_JOINT_KEYS)
        ])

        if self._last_stamp_ns is None:
            dt = 0.02
        else:
            dt = float(np.clip((stamp_ns - self._last_stamp_ns) / 1e9, 0.001, 0.1))
        self._last_stamp_ns = stamp_ns

        if self.filter_tau > 0.0:
            alpha = 1.0 - math.exp(-dt / self.filter_tau)
            desired = self._filtered_angles + alpha * (bounded - self._filtered_angles)
        else:
            desired = bounded
        max_step = self.max_speed * dt
        delta = np.clip(desired - self._filtered_angles, -max_step, max_step)
        self._filtered_angles += delta

        commanded = []
        for value, joint_name in zip(self._filtered_angles, ARM_JOINT_KEYS):
            low, high = JOINT_LIMITS[joint_name]
            commanded.append(float(np.clip(value, low, high)))
        for value, index in zip(commanded, LEFT_ARM_INDICES):
            self._joint_state[index] = value

        if self.record_diagnostics:
            upper_norm = float(np.linalg.norm(upper_direction))
            forearm_norm = float(np.linalg.norm(forearm_direction))
            if upper_norm > 1e-9 and forearm_norm > 1e-9:
                upper_unit = upper_direction / upper_norm
                forearm_unit = forearm_direction / forearm_norm
                pitch_observability = math.hypot(
                    float(upper_unit[0]), float(upper_unit[2]))
                forearm_perpendicular = (
                    forearm_unit
                    - float(np.dot(forearm_unit, upper_unit)) * upper_unit)
                yaw_observability = float(np.linalg.norm(forearm_perpendicular))
            else:
                upper_unit = np.full(3, np.nan)
                forearm_unit = np.full(3, np.nan)
                pitch_observability = float('nan')
                yaw_observability = float('nan')
            self._diagnostic_records.append({
                'stamp_ns': int(stamp_ns),
                'upper_position_m': p_u.copy(),
                'upper_quaternion_wxyz': q_u_reported.copy(),
                'forearm_position_m': p_f.copy(),
                'forearm_quaternion_wxyz': q_f_reported.copy(),
                'upper_direction_body': upper_unit,
                'forearm_direction_body': forearm_unit,
                'raw_joint_angles_rad': np.asarray(raw, dtype=float).copy(),
                'commanded_joint_angles_rad': np.asarray(commanded, dtype=float),
                'pitch_observability': pitch_observability,
                'pitch_singularity_locked': bool(
                    self._pitch_singularity_locked),
                'yaw_observability': yaw_observability,
                'upper_radius_error_m': (
                    float(upper_radius_error)
                    if upper_radius_error is not None else float('nan')),
                'shoulder_drift_m': (
                    float(shoulder_drift)
                    if shoulder_drift is not None else float('nan')),
                'elbow_mismatch_m': (
                    float(elbow_mismatch)
                    if elbow_mismatch is not None else float('nan')),
            })

        output = JointState()
        output.header.stamp = upper_msg.header.stamp
        output.position = list(self._joint_state)
        if self.publish_joint_targets:
            self._publisher.publish(output)
        self._last_pair_wall = time.monotonic()

        degrees = np.degrees(commanded)
        message = (
            f'pitch={degrees[0]:+.1f} roll={degrees[1]:+.1f} '
            f'yaw={degrees[2]:+.1f} elbow={degrees[3]:+.1f} '
            f'wrist={degrees[4]:+.1f} deg')
        if self.use_functional_centres:
            message += (
                f'; joint-centre errors: shoulder={shoulder_drift*1000:.0f} mm, '
                f'elbow={elbow_mismatch*1000:.0f} mm')
        if self.use_hybrid:
            message += f'; upper radius error={upper_radius_error*1000:.0f} mm'
        self.get_logger().info(message, throttle_duration_sec=1.0)

    def _watchdog(self):
        if self._last_pair_wall is None:
            return
        age = time.monotonic() - self._last_pair_wall
        if age > self.watchdog_timeout:
            action = ('no new arm target is being published'
                      if self.publish_joint_targets else
                      'dry-run output is no longer updating')
            self.get_logger().warn(
                f'No valid receiver pair for {age:.2f} s; {action}.',
                throttle_duration_sec=1.0)

    def _save_diagnostics(self):
        if (self._diagnostic_saved or not self.record_diagnostics or
                not self._diagnostic_records):
            return None
        os.makedirs(self.diagnostic_log_dir, exist_ok=True)
        timestamp = self._diagnostic_started_at.strftime('%Y%m%dT%H%M%SZ')
        path = os.path.join(
            self.diagnostic_log_dir,
            f'dual_retargeting_{timestamp}.npz')
        keys = tuple(self._diagnostic_records[0].keys())
        arrays = {
            key: np.asarray([record[key] for record in self._diagnostic_records])
            for key in keys
        }
        arrays.update({
            'joint_names': np.asarray(ARM_JOINT_KEYS),
            'calibration_mode': np.asarray(self.calibration_mode),
            'rotation_convention': np.asarray(self.rotation_convention),
            'calibration_dry_run_only': np.asarray(
                self.calibration_dry_run_only),
            'min_pitch_observability': np.asarray(
                self.min_pitch_observability),
            'pitch_reacquire_observability': np.asarray(
                self.pitch_reacquire_observability),
            'min_yaw_observability': np.asarray(
                self.min_yaw_observability),
        })
        np.savez_compressed(path, **arrays)
        self._diagnostic_saved = True
        self.get_logger().info(
            f'Saved {len(self._diagnostic_records)} full-rate diagnostic '
            f'frames to {path}')
        return path

    def destroy_node(self):
        self._save_diagnostics()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DualRetargetingNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
