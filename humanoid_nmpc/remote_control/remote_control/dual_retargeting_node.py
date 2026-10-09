#!/usr/bin/env python3

"""Retarget synchronized upper-arm and forearm WEMT poses to the G1 arm."""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from remote_control.dual_calibration_node import load_dual_calibration
from remote_control.dual_retargeting_kinematics import dual_joint_angles
from remote_control.retargeting_kinematics import quaternion_to_matrix
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


class DualRetargetingNode(Node):
    def __init__(self):
        super().__init__('dual_retargeting_node')
        calibration = load_dual_calibration()

        self.R_align = self._matrix(calibration, 'rotation_tx_to_body')
        self.upper_mount = self._matrix(calibration, 'upper_sensor_mount_rotation')
        self.forearm_mount = self._matrix(calibration, 'forearm_sensor_mount_rotation')
        self.shoulder_center = self._vector(calibration, 'shoulder_center_tx_m')
        self.upper_to_shoulder = self._vector(calibration, 'upper_sensor_to_shoulder_m')
        self.upper_to_elbow = self._vector(calibration, 'upper_sensor_to_elbow_m')
        self.forearm_to_elbow = self._vector(calibration, 'forearm_sensor_to_elbow_m')

        self.declare_parameter('filter_time_constant_s', 0.06)
        self.declare_parameter('max_joint_speed_rad_s', 2.5)
        self.declare_parameter('max_shoulder_drift_m', 0.05)
        self.declare_parameter('max_elbow_mismatch_m', 0.05)
        self.declare_parameter('watchdog_timeout_s', 0.25)
        self.filter_tau = max(0.0, float(self.get_parameter('filter_time_constant_s').value))
        self.max_speed = max(0.1, float(self.get_parameter('max_joint_speed_rad_s').value))
        self.max_shoulder_drift = max(0.0, float(self.get_parameter('max_shoulder_drift_m').value))
        self.max_elbow_mismatch = max(0.0, float(self.get_parameter('max_elbow_mismatch_m').value))
        self.watchdog_timeout = max(0.05, float(self.get_parameter('watchdog_timeout_s').value))

        quality = calibration.get('quality', {})
        self.get_logger().info(
            'Loaded dual calibration: '
            f"upper={calibration['upper_arm_length_m']*100:.1f} cm, "
            f"elbow fit={quality.get('elbow_pivot_rms_m', float('nan'))*1000:.1f} mm, "
            f"reference max={quality.get('reference_max_error_deg', float('nan')):.1f} deg")

        self._pending_upper = {}
        self._pending_forearm = {}
        self._joint_state = list(DEFAULT_JOINT_STATE)
        self._raw_angles = np.zeros(5)
        self._filtered_angles = np.zeros(5)
        self._last_stamp_ns = None
        self._last_pair_wall = None

        self.create_subscription(
            PoseStamped, '/em/upper_arm_pose', self._on_upper_pose, 20)
        self.create_subscription(
            PoseStamped, '/em/forearm_pose', self._on_forearm_pose, 20)
        self._publisher = self.create_publisher(JointState, '/arm_joint_target', 10)
        self._watchdog_timer = self.create_timer(0.1, self._watchdog)
        self.get_logger().info(
            'Dual retargeting ready. Hold the calibration reference pose before starting MPC.')

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

        output = JointState()
        output.header.stamp = upper_msg.header.stamp
        output.position = list(self._joint_state)
        self._publisher.publish(output)
        self._last_pair_wall = time.monotonic()

        degrees = np.degrees(commanded)
        self.get_logger().info(
            f'pitch={degrees[0]:+.1f} roll={degrees[1]:+.1f} '
            f'yaw={degrees[2]:+.1f} elbow={degrees[3]:+.1f} wrist={degrees[4]:+.1f} deg; '
            f'joint-centre errors: shoulder={shoulder_drift*1000:.0f} mm, '
            f'elbow={elbow_mismatch*1000:.0f} mm',
            throttle_duration_sec=1.0)

    def _watchdog(self):
        if self._last_pair_wall is None:
            return
        age = time.monotonic() - self._last_pair_wall
        if age > self.watchdog_timeout:
            self.get_logger().warn(
                f'No valid receiver pair for {age:.2f} s; no new arm target is being published.',
                throttle_duration_sec=1.0)


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
