#!/usr/bin/env python3

"""
retargeting_node.py

Core thesis contribution: maps human forearm pose (from WMET EM tracker) to
G1 robot left arm joint angles and forwards them to the MPC.

Pipeline:
  /em/pose  (PoseStamped, 50 Hz)
    ↓  Step 1: extract wrist position W and forearm orientation R
    ↓  Step 2: compute elbow position  E = W - R * forearm_axis * L2
    ↓  Step 3: shoulder_pitch, shoulder_roll  from elbow direction
    ↓  Step 4: elbow_joint angle from law of cosines
    ↓  Step 5: shoulder_yaw from forearm orientation relative to upper arm
    ↓  Step 6: clamp to joint limits
    ↓  pack into full 21-joint MPC state vector
  /arm_joint_target  (JointState, 50 Hz)
    ↓  C++ subscriber in CentroidalMpcRobotSim.cpp
    ↓  setTargetJointState()
  MPC arm tracking

Coordinate frame:
  All positions are in the EM transmitter frame.
  The transmitter is at the user's shoulder (origin).
  The receiver is on the user's forearm near the wrist.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from remote_control.calibration_node import load_calibration


# ---------------------------------------------------------------------------
# MPC joint state layout  (21 joints total, wrist_roll joints are FIXED → excluded)
#
#  Index  Joint name
#  -----  -----------------------------------------------
#   0-5   left leg  (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
#   6-11  right leg (same order)
#   12    waist_yaw_joint
#   13    left_shoulder_pitch_joint   ← controlled by EM tracker
#   14    left_shoulder_roll_joint    ← controlled by EM tracker
#   15    left_shoulder_yaw_joint     ← controlled by EM tracker
#   16    left_elbow_joint            ← controlled by EM tracker
#   17    right_shoulder_pitch_joint
#   18    right_shoulder_roll_joint
#   19    right_shoulder_yaw_joint
#   20    right_elbow_joint
#
#  Note: left_wrist_roll (ref.info idx 17) and right_wrist_roll (ref.info idx 22)
#  are fixedJointNames → NOT in MPC state → indices shift: right arm is 17-20 here.
# ---------------------------------------------------------------------------

LEFT_ARM_INDICES = [13, 14, 15, 16]  # positions in the 21-element MPC joint vector

LEFT_ARM_JOINT_NAMES = [
    'left_shoulder_pitch_joint',
    'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint',
    'left_elbow_joint',
]

# Default MPC joint state (21 joints).
# Values from reference.info defaultJointState — legs in nominal stance, arms at rest.
DEFAULT_JOINT_STATE = [
    # left leg  (indices 0-5)
    -0.05, 0.0, 0.0, 0.1, -0.05, 0.0,
    # right leg (indices 6-11)
    -0.05, 0.0, 0.0, 0.1, -0.05, 0.0,
    # waist_yaw (index 12)
    0.0,
    # left arm  (indices 13-16) — overwritten every frame by EM tracker
    0.0, 0.0, 0.0, 0.0,
    # right arm (indices 17-20) — stays at default
    0.0, 0.0, 0.0, 0.0,
]  # 21 elements total

# Joint limits [min_rad, max_rad] for the left arm.
# Source: g1_23dof.urdf — use URDF values, not estimates.
JOINT_LIMITS = {
    'left_shoulder_pitch_joint': (-3.0892, 2.6704),
    'left_shoulder_roll_joint':  (-1.5882, 2.2515),
    'left_shoulder_yaw_joint':   (-2.618,  2.618),
    'left_elbow_joint':          (-1.0472, 2.0944),
}

# The forearm bone direction in the RECEIVER's local frame.
# When the forearm points "straight along its own axis", this local vector
# describes that direction. Default: local +Z axis.
# If the retargeting looks wrong (elbow goes opposite direction), try [0,0,-1].
FOREARM_LOCAL_AXIS = np.array([0.0, 1.0, 0.0])  # confirmed from WMET sensor mounting test


# ---------------------------------------------------------------------------

class RetargetingNode(Node):

    def __init__(self):
        super().__init__('retargeting_node')

        # Load arm lengths from calibration_node output
        try:
            cal = load_calibration()
            self.L1 = cal['upper_arm_length_m']  # shoulder to elbow
            self.L2 = cal['forearm_length_m']     # elbow to wrist (receiver)
            self.get_logger().info(
                f'Calibration: upper_arm={self.L1*100:.1f} cm, forearm={self.L2*100:.1f} cm'
            )
        except FileNotFoundError as e:
            self.get_logger().fatal(str(e))
            raise

        # Working copy of the full 21-joint state.
        # Arm joints (indices 13-16) are overwritten each callback; rest stays at default.
        self._joint_state = list(DEFAULT_JOINT_STATE)

        # Subscriber: EM tracker pose at 50 Hz
        self.create_subscription(PoseStamped, '/em/pose', self._on_pose_received, 10)

        # Publisher: full 21-joint state sent to MPC
        self._pub = self.create_publisher(JointState, '/arm_joint_target', 10)

        self.get_logger().info('Retargeting node ready. Listening on /em/pose ...')

    # ------------------------------------------------------------------
    # Main callback  (50 Hz)
    # ------------------------------------------------------------------

    def _on_pose_received(self, msg: PoseStamped):
        """
        Called every time a new EM pose arrives (50 Hz).
        Computes arm joint angles and publishes the full MPC joint state.
        """
        # Wrist position in shoulder frame (meters)
        W = np.array([msg.pose.position.x,
                      msg.pose.position.y,
                      msg.pose.position.z])

        # Forearm orientation as quaternion [w, x, y, z]
        q = np.array([msg.pose.orientation.w,
                      msg.pose.orientation.x,
                      msg.pose.orientation.y,
                      msg.pose.orientation.z])

        angles = self._compute_joint_angles(W, q)
        if angles is None:
            return  # unreachable configuration — skip this frame

        # Write the 4 left-arm angles into the full joint state
        for i, state_idx in enumerate(LEFT_ARM_INDICES):
            self._joint_state[state_idx] = angles[i]

        # Publish  — C++ subscriber picks this up and calls setTargetJointState()
        out = JointState()
        out.header.stamp = msg.header.stamp  # keep original timestamp
        out.position = list(self._joint_state)  # 21 floats
        self._pub.publish(out)

    # ------------------------------------------------------------------
    # Inverse kinematics
    # ------------------------------------------------------------------

    def _compute_joint_angles(self, W: np.ndarray, q: np.ndarray):
        """
        Maps human arm pose to 4 robot joint angles.

        Args:
            W: wrist position [x,y,z] in shoulder (transmitter) frame, meters
            q: forearm quaternion [w,x,y,z]

        Returns:
            [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_joint] radians
            or None if configuration is unreachable.
        """

        # ----------------------------------------------------------------
        # Step 1 — Forearm direction in shoulder frame
        # ----------------------------------------------------------------
        R = _quat_to_matrix(q)                     # 3x3 rotation: local → shoulder frame
        forearm_dir = R @ FOREARM_LOCAL_AXIS        # forearm direction in shoulder frame

        # ----------------------------------------------------------------
        # Step 2 — Elbow position
        #
        # The receiver sits at the wrist (W).
        # The forearm runs from elbow E to wrist W along forearm_dir.
        # Therefore: E = W - forearm_dir * L2
        # ----------------------------------------------------------------
        E = W - forearm_dir * self.L2
        elbow_dist = np.linalg.norm(E)

        # Guard: elbow should be approximately L1 from shoulder.
        # Allow 15% margin for calibration imperfection and measurement noise.
        if elbow_dist > self.L1 * 1.15:
            self.get_logger().warn(
                f'Elbow distance {elbow_dist*100:.1f} cm > upper arm {self.L1*100:.1f} cm'
                ' — unreachable, skipping.',
                throttle_duration_sec=1.0
            )
            return None

        E_hat = E / (elbow_dist + 1e-9)  # unit vector: shoulder → elbow direction

        # ----------------------------------------------------------------
        # Step 3 — Shoulder pitch and roll from elbow direction
        #
        # Robot zero configuration (from g1_23dof.urdf, all joints = 0):
        #   upper arm hangs straight DOWN, forearm points FORWARD.
        # This means shoulder_pitch = 0 when arm hangs down.
        #
        #   shoulder_pitch (Y-axis rotation):
        #     pitch = atan2(-E_hat.x, -E_hat.z)
        #     0      → arm hangs straight down
        #     -π/2   → arm raised forward horizontal
        #     +π/2   → arm swings backward
        #     Confirmed from g1_23dof.urdf URDF preview: positive pitch = arm backward.
        #
        #   shoulder_roll (X-axis rotation): arm swings outward/inward
        #     roll = asin(E_hat.y)
        #     0      → arm at side (no sideways movement)
        #     +π/2   → arm raised fully sideways (abduction)
        #
        # This is an approximation: the joints are sequential (pitch then roll),
        # so the decomposition is not exact for large angles. For accurate full-range
        # tracking, replace with Pinocchio numerical IK.
        # ----------------------------------------------------------------
        # Guard: when arm points nearly straight sideways (E_hat ≈ ±Y), both
        # E_hat[0] and E_hat[2] are ≈ 0.  IEEE 754 negative-zero makes
        # atan2(-0.0, -0.0) = -π instead of 0, so we must handle this explicitly.
        if abs(E_hat[0]) < 1e-6 and abs(E_hat[2]) < 1e-6:
            shoulder_pitch = 0.0
        else:
            shoulder_pitch = math.atan2(-E_hat[0], -E_hat[2])
        shoulder_roll  = math.asin(float(np.clip(E_hat[1], -1.0, 1.0)))

        # ----------------------------------------------------------------
        # Step 4 — Elbow joint angle from law of cosines
        #
        # Triangle: S (shoulder) — E (elbow) — W (wrist)
        # Sides: |SE| = L1, |EW| = L2, |SW| = d_sw
        #
        # Law of cosines gives cos_val = (L1²+L2²-d_sw²) / (2·L1·L2)
        #
        # Robot elbow joint (axis = Y, from g1_23dof.urdf):
        #   0°   = forearm pointing forward (+X)  — robot default
        #  +90°  = forearm pointing downward (-Z) — arm fully extended
        #  -60°  = forearm pointing upward  (+Z)  — arm folded (lower limit)
        #
        # Derived from URDF preview (confirmed visually):
        #   robot_elbow = acos(cos_val) - π/2
        #
        # Verification:
        #   arm straight (d_sw = L1+L2): cos_val=-1 → acos(π)-π/2 = π/2  ≈ +1.571 ✓
        #   arm 90° bent              : cos_val= 0 → acos(π/2)-π/2 = 0    ✓
        #   arm 150° bent             : cos_val=√3/2→ acos(π/6)-π/2 = -π/3 ≈ -1.047 ✓
        # ----------------------------------------------------------------
        d_sw = np.linalg.norm(W)
        d_sw_safe = float(np.clip(d_sw, abs(self.L1 - self.L2) + 1e-6, self.L1 + self.L2))
        cos_val      = (self.L1**2 + self.L2**2 - d_sw_safe**2) / (2 * self.L1 * self.L2)
        elbow_scaled = math.acos(float(np.clip(cos_val, -1.0, 1.0))) - math.pi / 2

        # ----------------------------------------------------------------
        # Step 5 — Shoulder yaw (upper arm axial twist)
        #
        # Shoulder yaw describes how much the upper arm is twisted around its
        # own axis. It is computed from the forearm's orientation relative to
        # the plane formed by the upper arm direction and the world vertical.
        #
        # Reference plane normal: cross(E_hat, world_down)
        # Forearm component perpendicular to upper arm: forearm_dir - (forearm_dir·E_hat)·E_hat
        # Shoulder yaw = signed angle between reference direction and forearm component.
        # ----------------------------------------------------------------
        world_down = np.array([0.0, 0.0, -1.0])

        # Reference direction: perpendicular to upper arm, in the gravity plane
        ref = world_down - np.dot(world_down, E_hat) * E_hat
        ref_norm = np.linalg.norm(ref)

        # Forearm component perpendicular to upper arm
        fp = forearm_dir - np.dot(forearm_dir, E_hat) * E_hat
        fp_norm = np.linalg.norm(fp)

        if ref_norm > 0.05 and fp_norm > 0.05:
            ref = ref / ref_norm
            fp  = fp  / fp_norm
            cross = np.cross(ref, fp)
            shoulder_yaw = math.atan2(float(np.dot(cross, E_hat)), float(np.dot(ref, fp)))
        else:
            # Singular: arm pointing straight down or forearm parallel to upper arm
            shoulder_yaw = 0.0

        # ----------------------------------------------------------------
        # Step 6 — Apply joint limits
        # ----------------------------------------------------------------
        result = [
            _clamp(shoulder_pitch, *JOINT_LIMITS['left_shoulder_pitch_joint']),
            _clamp(shoulder_roll,  *JOINT_LIMITS['left_shoulder_roll_joint']),
            _clamp(shoulder_yaw,   *JOINT_LIMITS['left_shoulder_yaw_joint']),
            _clamp(elbow_scaled,   *JOINT_LIMITS['left_elbow_joint']),
        ]
        return result


# ---------------------------------------------------------------------------
# Math utilities
# ---------------------------------------------------------------------------

def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """
    Quaternion [w, x, y, z] → 3x3 rotation matrix.
    Maps vectors from the local (receiver) frame to the world (transmitter) frame:
        v_shoulder = R @ v_local
    """
    q = q / np.linalg.norm(q)  # normalize to avoid drift errors
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w),   1 - 2*(x*x + z*z),  2*(y*z - x*w)],
        [    2*(x*z - y*w),   2*(y*z + x*w),      1 - 2*(x*x + y*y)],
    ])


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = RetargetingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
