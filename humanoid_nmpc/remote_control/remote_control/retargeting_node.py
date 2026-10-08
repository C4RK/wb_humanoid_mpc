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
    ↓  pack into full 22-joint MPC state vector
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
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from remote_control.calibration_node import load_calibration, CALIBRATION_FILE
from remote_control.retargeting_kinematics import (
    calibration_alignment_errors,
    quaternion_to_matrix,
    sensor_mount_from_reference,
    shoulder_pitch_roll,
    shoulder_yaw as solve_shoulder_yaw,
    wrist_roll as solve_wrist_roll,
)


# ---------------------------------------------------------------------------
# MPC joint state layout  (22 joints total, only right_wrist_roll is FIXED)
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
#   17    left_wrist_roll_joint       ← controlled by EM tracker (pronation/supination)
#   18    right_shoulder_pitch_joint
#   19    right_shoulder_roll_joint
#   20    right_shoulder_yaw_joint
#   21    right_elbow_joint
#
#  Note: right_wrist_roll (ref.info idx 22) is still fixedJointName → excluded.
#  left_wrist_roll is now tracked → at index 17, right arm shifted to 18-21.
# ---------------------------------------------------------------------------

LEFT_ARM_INDICES = [13, 14, 15, 16, 17]  # positions in the 22-element MPC joint vector

LEFT_ARM_JOINT_NAMES = [
    'left_shoulder_pitch_joint',
    'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint',
    'left_elbow_joint',
    'left_wrist_roll_joint',
]

# Default MPC joint state (22 joints).
# Values from reference.info defaultJointState — legs in nominal stance, arms at rest.
DEFAULT_JOINT_STATE = [
    # left leg  (indices 0-5)
    -0.05, 0.0, 0.0, 0.1, -0.05, 0.0,
    # right leg (indices 6-11)
    -0.05, 0.0, 0.0, 0.1, -0.05, 0.0,
    # waist_yaw (index 12)
    0.0,
    # left arm  (indices 13-17) — overwritten every frame by EM tracker
    0.0, 0.0, 0.0, 0.0, 0.0,
    # right arm (indices 18-21) — stays at default
    0.0, 0.0, 0.0, 0.0,
]  # 22 elements total

# Joint limits [min_rad, max_rad] for the left arm.
# Source: g1_23dof.urdf — use URDF values, not estimates.
JOINT_LIMITS = {
    'left_shoulder_pitch_joint': (-3.0892, 2.6704),
    'left_shoulder_roll_joint':  (-1.5882, 2.2515),
    'left_shoulder_yaw_joint':   (-2.618,  2.618),
    'left_elbow_joint':          (-1.0472, 2.0944),
    'left_wrist_roll_joint':     (-1.97222, 1.97222),
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

            # Shoulder offset: X and Y displacement of the wrist when the arm
            # hangs naturally at rest (caused by body width, hip geometry, and
            # the gap between the transmitter and the actual shoulder joint).
            # Subtracting this before IK ensures the natural-rest pose maps to
            # zero shoulder pitch/roll commands on the robot.
            # Falls back to zero for old calibration files without this field.
            # Shoulder joint position in transmitter frame, from sphere-fit
            # calibration. Subtracted from every wrist reading before IK so
            # that the IK always works in the shoulder-centred frame.
            # Falls back to zero for older calibration files.
            self.shoulder_offset = np.array([
                cal.get('shoulder_offset_x_m', 0.0),
                cal.get('shoulder_offset_y_m', 0.0),
                cal.get('shoulder_offset_z_m', 0.0),
            ])

            self.get_logger().info(
                f'Calibration: upper_arm={self.L1*100:.1f} cm, forearm={self.L2*100:.1f} cm'
            )
            self.get_logger().info(
                f'Shoulder joint offset: x={self.shoulder_offset[0]*100:.1f} cm, '
                f'y={self.shoulder_offset[1]*100:.1f} cm, '
                f'z={self.shoulder_offset[2]*100:.1f} cm'
            )

            # ── Build R_align from calibration reference directions ───────
            # Prefer 3-vector SVD (Wahba's problem) when arm_sideways_hat is
            # available (Step 4 of calibration).  This correctly handles the
            # non-orthogonality between arm_down and arm_forward that causes
            # a ~38° pitch error during lateral raises with the 2-step method.
            # Falls back to the original 2-step approach for old calibration files.
            arm_down_raw = np.array([
                cal.get('arm_down_hat_x', 0.0),
                cal.get('arm_down_hat_y', 0.0),
                cal.get('arm_down_hat_z', -1.0),
            ])
            if np.linalg.norm(arm_down_raw) < 0.1:
                arm_down_raw = np.array([0.0, 0.0, -1.0])
            arm_down = arm_down_raw / np.linalg.norm(arm_down_raw)

            arm_fwd_key = cal.get('arm_forward_hat_x')
            arm_sideways_key = cal.get('arm_sideways_hat_x')

            if arm_fwd_key is not None and arm_sideways_key is not None:
                # ── Gram-Schmidt: arm_sideways defines the lateral axis exactly ──
                #
                # SVD (Wahba's) averaged over all three directions, but arm_fwd
                # and arm_side are only ~46° apart in transmitter space (should be
                # 90°), so the SVD compromise left pitch = -38° in lateral raises.
                #
                # Instead: fix arm_side → robot [0,1,0] exactly, then orthogonalise
                # arm_down against it and build the forward axis via cross product.
                # This guarantees pitch=0° for the exact pose held at Step 4.
                arm_side = np.array([cal['arm_sideways_hat_x'],
                                     cal['arm_sideways_hat_y'],
                                     cal['arm_sideways_hat_z']])
                arm_side /= np.linalg.norm(arm_side)

                # e_side: the robot's lateral (+Y) direction in transmitter frame
                e_side = arm_side

                # Project arm_down perpendicular to e_side → pure "down" component
                down_orth = arm_down - np.dot(arm_down, e_side) * e_side
                e_down = down_orth / np.linalg.norm(down_orth)   # → robot [0,0,-1]
                e_up   = -e_down                                   # → robot [0,0,+1]

                # Right-hand forward: cross(side, up) → robot [1,0,0]
                e_fwd = np.cross(e_side, e_up)
                e_fwd /= np.linalg.norm(e_fwd)

                # R_align = source_frame.T  (orthonormal by construction)
                M_S = np.column_stack([e_fwd, e_side, e_up])
                self.R_align = M_S.T

                dot_ds = float(np.dot(arm_down, arm_side))
                angle_ds = math.degrees(math.acos(float(np.clip(dot_ds, -1.0, 1.0))))
                self.get_logger().info(
                    f'R_align: Gram-Schmidt (arm_side exact, arm_down approx). '
                    f'arm_down/arm_side angle={angle_ds:.1f}° '
                    f'(90° = perfect orthogonal)')
            elif arm_fwd_key is not None:
                # ── 2-step fallback ───────────────────────────────────────
                R1 = _rotation_between(arm_down, np.array([0.0, 0.0, -1.0]))
                arm_fwd_raw = np.array([cal['arm_forward_hat_x'],
                                        cal['arm_forward_hat_y'],
                                        cal['arm_forward_hat_z']])
                arm_fwd_raw /= np.linalg.norm(arm_fwd_raw)
                fwd_after_R1 = R1 @ arm_fwd_raw
                fxy = fwd_after_R1[:2]
                fxy_norm = float(np.linalg.norm(fxy))
                if fxy_norm > 0.1:
                    theta = math.atan2(float(fxy[1]), float(fxy[0]))
                    ct, st = math.cos(theta), math.sin(theta)
                    R2 = np.array([[ ct, st, 0.0],
                                   [-st, ct, 0.0],
                                   [0.0, 0.0, 1.0]])
                else:
                    R2 = np.eye(3)
                self.R_align = R2 @ R1
                self.get_logger().warn(
                    'R_align: 2-step fallback (no arm_sideways_hat). '
                    'Re-run calibration (Step 4) to fix lateral-raise pitch error.')
            else:
                R1 = _rotation_between(arm_down, np.array([0.0, 0.0, -1.0]))
                self.R_align = R1
                self.get_logger().warn(
                    'No arm_forward_hat in calibration file. '
                    'Re-run calibration to fix tracking errors.')

            alignment_errors = calibration_alignment_errors(cal, self.R_align)
            if alignment_errors:
                residual_text = ', '.join(
                    f'{name}={error:.1f}°' for name, error in alignment_errors.items())
                self.get_logger().info(f'Calibration reference residuals: {residual_text}')
                if max(alignment_errors.values()) > 15.0:
                    self.get_logger().warn(
                        'Calibration reference poses are inconsistent (>15° residual). '
                        'Simulation can run, but repeat calibration before accuracy experiments.')

            # Yaw offset: sensor mounting rotation around the forearm axis.
            # Computed during calibration from the Step 3 "arm forward" pose.
            # Subtract from every computed shoulder yaw so that yaw = 0 at
            # the calibration pose (upper arm forward, forearm up, neutral wrist).
            self.yaw_offset = cal.get('yaw_offset', 0.0)
            self.get_logger().info(
                f'Yaw offset (sensor mounting): {math.degrees(self.yaw_offset):+.1f}°'
            )

            # Wrist roll reference: sensor quaternion at Step 3 (arm forward, wrist neutral).
            # The retargeting measures forearm rotation (pronation/supination) as the
            # rotation of the sensor around its own Y-axis (forearm axis) relative to
            # this reference.  Requires re-calibration to take effect.
            wrist_roll_ref = cal.get('wrist_roll_ref_quat', None)
            if wrist_roll_ref:
                self.R_wrist_roll_ref = _quat_to_matrix(np.array(wrist_roll_ref))

                # Reconstruct the Step 3 zero using the final alignment matrix.
                # Older calibration files computed yaw_offset before the sideways
                # reference was available, then changed R_align afterwards.  That
                # made the saved zero inconsistent by tens of degrees.  Deriving
                # it again from the saved Step 3 direction and quaternion keeps
                # yaw and wrist zero tied to the same final frame.
                reference_angles = (-math.pi / 2.0, 0.0, 0.0, 0.0)
                if cal.get('arm_forward_hat_x') is not None:
                    upper_ref = self.R_align @ np.array([
                        cal['arm_forward_hat_x'],
                        cal['arm_forward_hat_y'],
                        cal['arm_forward_hat_z'],
                    ])
                    upper_ref /= np.linalg.norm(upper_ref)
                    sensor_ref_aligned = self.R_align @ self.R_wrist_roll_ref
                    forearm_ref = sensor_ref_aligned @ FOREARM_LOCAL_AXIS
                    forearm_ref /= np.linalg.norm(forearm_ref)
                    pitch_ref, roll_ref = shoulder_pitch_roll(upper_ref)
                    derived_yaw_offset, observable = solve_shoulder_yaw(
                        upper_ref, forearm_ref, pitch_ref, roll_ref,
                        yaw_offset=0.0, fallback=self.yaw_offset)
                    elbow_ref = math.asin(float(np.clip(
                        np.dot(upper_ref, forearm_ref), -1.0, 1.0)))
                    if observable:
                        saved_yaw_offset = self.yaw_offset
                        self.yaw_offset = derived_yaw_offset
                        if abs(self.yaw_offset - saved_yaw_offset) > math.radians(2.0):
                            self.get_logger().warn(
                                'Saved yaw zero used an older alignment; '
                                f'using Step 3 zero {math.degrees(self.yaw_offset):+.1f}° '
                                f'instead of {math.degrees(saved_yaw_offset):+.1f}°.')
                    reference_angles = (pitch_ref, roll_ref, 0.0, elbow_ref)

                self.R_sensor_mount = sensor_mount_from_reference(
                    self.R_align, self.R_wrist_roll_ref, reference_angles)
                self.has_wrist_roll = True
                self.get_logger().info(
                    'Wrist roll reference loaded — shoulder motion compensation active.')
            else:
                self.R_wrist_roll_ref = np.eye(3)
                self.R_sensor_mount = np.eye(3)
                self.has_wrist_roll = False
                self.get_logger().warn(
                    'No wrist_roll_ref_quat in calibration — re-run calibration to enable wrist tracking.'
                )
        except FileNotFoundError as e:
            self.get_logger().fatal(str(e))
            raise

        # Continuity values are also used through singular configurations.  With
        # an almost straight elbow, shoulder yaw is not observable from one
        # forearm receiver, so keeping the last valid value prevents jumps.
        self._last_shoulder_yaw = 0.0
        self._last_wrist_roll = 0.0

        # Working copy of the full 22-joint state.
        # Arm joints (indices 13-17) are overwritten each callback; rest stays at default.
        self._joint_state = list(DEFAULT_JOINT_STATE)

        # Subscriber: EM tracker pose at 50 Hz
        self.create_subscription(PoseStamped, '/em/pose', self._on_pose_received, 10)

        # Publisher: full 22-joint state sent to MPC
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
        # Wrist position in shoulder frame (meters).
        # Subtract the shoulder offset so that the natural-rest arm pose maps
        # to [0, 0, -L_total] — i.e. zero shoulder pitch/roll commands.
        W = np.array([msg.pose.position.x,
                      msg.pose.position.y,
                      msg.pose.position.z]) - self.shoulder_offset

        # Forearm orientation as quaternion [w, x, y, z]
        q = np.array([msg.pose.orientation.w,
                      msg.pose.orientation.x,
                      msg.pose.orientation.y,
                      msg.pose.orientation.z])

        angles = self._compute_joint_angles(W, q)
        if angles is None:
            return  # unreachable configuration — skip this frame

        # Write the 5 left-arm angles into the full joint state
        for i, state_idx in enumerate(LEFT_ARM_INDICES):
            self._joint_state[state_idx] = angles[i]

        # Diagnostic: log angles at 1 Hz so we can verify correctness
        self.get_logger().info(
            f'pitch={math.degrees(angles[0]):+.1f}° roll={math.degrees(angles[1]):+.1f}°'
            f' yaw={math.degrees(angles[2]):+.1f}° elbow={math.degrees(angles[3]):+.1f}°'
            f' wrist={math.degrees(angles[4]):+.1f}°',
            throttle_duration_sec=0.1
        )

        # Publish  — C++ subscriber picks this up and calls setTargetJointState()
        out = JointState()
        out.header.stamp = msg.header.stamp  # keep original timestamp
        out.position = list(self._joint_state)  # 22 floats
        self._pub.publish(out)

    # ------------------------------------------------------------------
    # Inverse kinematics
    # ------------------------------------------------------------------

    def _compute_joint_angles(self, W: np.ndarray, q: np.ndarray):
        """
        Maps human arm pose to 5 robot joint angles.

        Args:
            W: wrist position [x,y,z] in shoulder (transmitter) frame, meters
            q: forearm quaternion [w,x,y,z]

        Returns:
            [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_joint, wrist_roll] radians
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
                f' — unreachable, skipping.'
                f'  W=[{W[0]*100:.1f},{W[1]*100:.1f},{W[2]*100:.1f}] cm'
                f'  f=[{forearm_dir[0]:+.3f},{forearm_dir[1]:+.3f},{forearm_dir[2]:+.3f}]'
                f'  E=[{E[0]*100:.1f},{E[1]*100:.1f},{E[2]*100:.1f}] cm',
                throttle_duration_sec=1.0
            )
            return None

        # Rotate E and forearm_dir from transmitter frame into robot-aligned
        # frame (where "arm hanging down" = [0, 0, -1]).
        # Magnitudes are preserved (R_align is orthogonal), so elbow_dist is
        # the same before and after — only directions change.
        E          = self.R_align @ E
        forearm_dir = self.R_align @ forearm_dir
        R_sensor_aligned = self.R_align @ R

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
        # The forearm component perpendicular to the upper arm identifies the
        # elbow plane and therefore shoulder yaw.  Near full extension that
        # component vanishes; yaw is then unobservable with one receiver and
        # the last stable value is retained.
        # ----------------------------------------------------------------
        shoulder_yaw, yaw_observable = solve_shoulder_yaw(
            E_hat,
            forearm_dir,
            shoulder_pitch,
            shoulder_roll,
            yaw_offset=self.yaw_offset,
            fallback=self._last_shoulder_yaw,
        )
        if yaw_observable:
            self._last_shoulder_yaw = shoulder_yaw

        # ----------------------------------------------------------------
        # Step 6 — Wrist roll (forearm axial rotation = pronation / supination)
        #
        # Calibration Step 3 identifies the fixed sensor mounting rotation.
        # Remove the reconstructed shoulder and elbow rotations first, then
        # extract only the residual twist around the robot forearm (+X) axis.
        # This prevents ordinary arm raising from appearing as wrist rotation.
        # ----------------------------------------------------------------
        if self.has_wrist_roll:
            wrist_roll = solve_wrist_roll(
                R_sensor_aligned,
                self.R_sensor_mount,
                shoulder_pitch,
                shoulder_roll,
                shoulder_yaw,
                elbow_scaled,
                fallback=self._last_wrist_roll,
            )
            self._last_wrist_roll = wrist_roll
        else:
            wrist_roll = 0.0

        # ----------------------------------------------------------------
        # Step 7 — Apply joint limits
        # ----------------------------------------------------------------
        result = [
            _clamp(shoulder_pitch, *JOINT_LIMITS['left_shoulder_pitch_joint']),
            _clamp(shoulder_roll,  *JOINT_LIMITS['left_shoulder_roll_joint']),
            _clamp(shoulder_yaw,   *JOINT_LIMITS['left_shoulder_yaw_joint']),
            _clamp(elbow_scaled,   *JOINT_LIMITS['left_elbow_joint']),
            _clamp(wrist_roll,     *JOINT_LIMITS['left_wrist_roll_joint']),
        ]
        return result


# ---------------------------------------------------------------------------
# Math utilities
# ---------------------------------------------------------------------------

def _rotation_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """
    3×3 rotation matrix R such that R @ a = b, where a and b are unit vectors.
    Uses Rodrigues' rotation formula.

    Used to build the frame-alignment correction:
        R_align = _rotation_between(arm_down_hat, [0, 0, -1])
    """
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    s = float(np.linalg.norm(v))   # sin of angle
    c = float(np.dot(a, b))        # cos of angle
    if s < 1e-8:
        if c > 0:
            return np.eye(3)       # already aligned
        # Anti-parallel: 180° rotation around any perpendicular axis
        perp = np.array([1.0, 0.0, 0.0])
        if abs(float(np.dot(perp, a))) > 0.9:
            perp = np.array([0.0, 1.0, 0.0])
        ax = np.cross(a, perp)
        ax = ax / np.linalg.norm(ax)
        return 2.0 * np.outer(ax, ax) - np.eye(3)
    vx = np.array([[   0, -v[2],  v[1]],
                   [v[2],     0, -v[0]],
                   [-v[1], v[0],    0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / (s * s))


def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """
    Quaternion [w, x, y, z] → 3x3 rotation matrix.
    Maps vectors from the local (receiver) frame to the world (transmitter) frame:
        v_shoulder = R @ v_local
    """
    return quaternion_to_matrix(q)


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = RetargetingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
