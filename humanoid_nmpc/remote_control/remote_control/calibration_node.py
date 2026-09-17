#!/usr/bin/env python3

"""
calibration_node.py

Determines the user's arm lengths and shoulder joint position using the
WMET EM tracker. Results are saved to a YAML file and reused every session.

Calibration procedure:

  Step 0 — Shoulder sphere (8 s sweep, elbow locked straight):
    The wrist traces a sphere whose centre is the shoulder joint.
      → shoulder_joint  (3D position in transmitter frame)
      → sphere_radius   (≈ total arm length, used as cross-check)

  Step 1 — Free arm motion (15 s, varying elbow angle):
    Move the arm freely — bend and straighten the elbow while also
    moving the shoulder in different directions.  For each of the ~750
    recorded poses the sensor provides both the wrist position W and the
    forearm direction f (from the quaternion).  The elbow lies at:

        E = W − L2 · f

    and must always be exactly L1 from the shoulder joint S:

        |E − S|² = L1²

    Substituting V = W − S and expanding:

        2·(V·f)·L2 + (L1² − L2²) = |V|²

    This is LINEAR in the two unknowns [L2, u = L1²−L2²].
    Stacking one row per pose gives an overdetermined linear system
    that is solved with least squares.  Both L1 and L2 emerge directly
    — no subtraction, no dependency between them.

    Good conditioning requires V·f to vary widely across poses.
    Bending/straightening the elbow drives V·f from ≈L2 (bent)
    to ≈L_total (straight), which maximises the information content.

Coordinate frame: all positions are in the EM transmitter frame.
"""

import os
import math
import yaml
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from datetime import datetime, timezone


CALIBRATION_FILE = '/wb_humanoid_mpc_ws/src/wb_humanoid_mpc/humanoid_nmpc/remote_control/config/wmet_calibration.yaml'

# Step 0 sweep duration (seconds).
SWEEP_SECONDS_SHOULDER = 8.0

# Step 1 free-motion duration (seconds).
SWEEP_SECONDS_FREE = 15.0

# Forearm direction in the receiver's local frame (empirically confirmed).
FOREARM_LOCAL_AXIS = np.array([0.0, 1.0, 0.0])

# Acceptable sphere-fit RMS error.
SPHERE_FIT_TOLERANCE_M = 0.03

# Acceptable RMS elbow-distance residual from the linear solve (1 cm tight).
SOLVE_RMS_TOLERANCE_M = 0.02

# Acceptable deviation between (L1+L2) and the sphere radius (loose check).
CROSSCHECK_TOLERANCE_M = 0.05


class CalibrationNode(Node):

    def __init__(self):
        super().__init__('calibration_node')

        self.latest_pose = None
        self._pose_lock = threading.Lock()

        self.pose_subscriber = self.create_subscription(
            PoseStamped, '/em/pose', self._on_pose_received, 10)

        self.get_logger().info('Calibration node started. Waiting for /em/pose ...')

        self._calibration_thread = threading.Thread(
            target=self._run_calibration_sequence, daemon=True)
        self._calibration_thread.start()

    # ------------------------------------------------------------------
    # Subscriber callback
    # ------------------------------------------------------------------

    def _on_pose_received(self, msg: PoseStamped):
        with self._pose_lock:
            self.latest_pose = msg

    # ------------------------------------------------------------------
    # Calibration sequence
    # ------------------------------------------------------------------

    def _run_calibration_sequence(self):
        import time

        # ---- Check for existing calibration ----
        if os.path.exists(CALIBRATION_FILE):
            print(f'\n[Calibration] Existing calibration found at {CALIBRATION_FILE}')
            cal = self._load_calibration()
            print(f'  upper_arm_length = {cal["upper_arm_length_m"]*100:.1f} cm')
            print(f'  forearm_length   = {cal["forearm_length_m"]*100:.1f} cm')
            if 'shoulder_offset_x_m' in cal:
                ox, oy, oz = (cal['shoulder_offset_x_m'],
                              cal['shoulder_offset_y_m'],
                              cal.get('shoulder_offset_z_m', 0.0))
                print(f'  shoulder_joint   = x={ox*100:.1f}, y={oy*100:.1f}, '
                      f'z={oz*100:.1f} cm')
            answer = input('\nUse existing calibration? [Y/n]: ').strip().lower()
            if answer != 'n':
                print('[Calibration] Using existing calibration.')
                return

        # ---- Wait for first pose ----
        print('\n[Calibration] Waiting for EM tracker data on /em/pose ...')
        while rclpy.ok():
            with self._pose_lock:
                if self.latest_pose is not None:
                    break
            time.sleep(0.1)
        print('[Calibration] EM tracker data received.\n')

        # ================================================================
        # STEP 0 — Shoulder sphere
        # ================================================================
        print('=' * 60)
        print('STEP 0: Shoulder sphere — locate the shoulder joint')
        print()
        print('  1. LOCK your elbow completely straight.')
        print(f'  2. Press ENTER, then sweep your arm for '
              f'{SWEEP_SECONDS_SHOULDER:.0f} seconds')
        print('     in as many shoulder directions as possible:')
        print('       → forward, sideways, up, diagonal, down.')
        print('  3. Keep the elbow LOCKED the ENTIRE time.')
        print('  4. Move slowly and smoothly.')
        print()
        print('  WHY: the wrist traces a sphere; its centre is your')
        print('       shoulder joint — regardless of transmitter tilt.')
        print('=' * 60)
        input('Press ENTER, then start sweeping...')

        samples_0 = self._collect_samples_timed(SWEEP_SECONDS_SHOULDER)
        shoulder_joint, sphere_radius, rms_0 = self._fit_sphere(samples_0)

        print(f'\n  Shoulder joint (transmitter frame):')
        print(f'    x = {shoulder_joint[0]*100:+.1f} cm')
        print(f'    y = {shoulder_joint[1]*100:+.1f} cm')
        print(f'    z = {shoulder_joint[2]*100:+.1f} cm')
        print(f'  → Sphere radius (cross-check ≈ total arm): '
              f'{sphere_radius*100:.1f} cm')
        print(f'  → Fit error: {rms_0*100:.1f} cm RMS')

        if rms_0 > SPHERE_FIT_TOLERANCE_M:
            print(f'\n[WARNING] Fit error {rms_0*100:.1f} cm > '
                  f'{SPHERE_FIT_TOLERANCE_M*100:.0f} cm — elbow may not be locked.')
            answer = input('Retry? [Y/n]: ').strip().lower()
            if answer != 'n':
                print('[Calibration] Please redo Step 0 with elbow locked.')
                return
        else:
            print('[Calibration] Step 0 passed.\n')

        # ================================================================
        # STEP 1 — Free motion: solve L1 and L2 simultaneously
        # ================================================================
        print('=' * 60)
        print('STEP 1: Free arm motion — solve upper arm and forearm lengths')
        print()
        print('  Move your arm FREELY for 15 seconds.')
        print()
        print('  What to do:')
        print('    • Continuously BEND and STRAIGHTEN the elbow')
        print('      (vary from about 30° to almost fully extended).')
        print('    • At the same time, move the SHOULDER in different')
        print('      directions (forward, sideways, up, diagonally).')
        print('    • Move slowly and steadily — no fast flicks.')
        print()
        print('  What NOT to do:')
        print('    • Don\'t hold any fixed pose — keep moving the whole time.')
        print('    • Don\'t stay at one elbow angle — varying it is essential.')
        print()
        print('  WHY: each arm pose gives one linear equation in L1 and L2.')
        print('       ~750 poses together give a highly overdetermined system')
        print('       that is solved with least squares — no subtraction.')
        print('=' * 60)
        input('Press ENTER, then start moving...')

        samples_1 = self._collect_samples_timed(SWEEP_SECONDS_FREE)
        # ── Primary solve: constrained 1D using sphere radius ──────────
        # Enforcing L1 + L2 = sphere_radius eliminates the slope/intercept
        # correlation that causes L1 and L2 to trade off between sessions.
        L1, L2, rms_elbow = self._solve_arm_lengths_constrained(
            samples_1, shoulder_joint, sphere_radius)

        # ── Fallback: unconstrained 2D solve ────────────────────────────
        L1_2d, L2_2d, rms_2d = self._solve_arm_lengths_simultaneous(
            samples_1, shoulder_joint)

        if L1 is None and L1_2d is None:
            print('\n[ERROR] Both solvers failed — elbow angle may not have varied.')
            print('  Please try again and bend/straighten the elbow continuously.')
            return

        if L1 is None:
            print('\n[NOTE] Constrained solve failed; using unconstrained solve.')
            L1, L2, rms_elbow = L1_2d, L2_2d, rms_2d

        computed_total = L1 + L2
        crosscheck = abs(computed_total - sphere_radius)
        split_diff = abs((L1 - L1_2d)) if L1_2d is not None else float('inf')

        print(f'\n  → upper_arm_length (L1): {L1*100:.1f} cm')
        print(f'  → forearm_length   (L2): {L2*100:.1f} cm')
        print(f'  → L1 + L2:              {computed_total*100:.1f} cm  '
              f'(anchored to sphere radius)')
        print(f'  → Sphere radius (Step 0): {sphere_radius*100:.1f} cm')
        print(f'  → Elbow residual RMS:    {rms_elbow*100:.2f} cm')
        if L1_2d is not None:
            print(f'  → Cross-check (2D solve):  L1={L1_2d*100:.1f} cm, '
                  f'L2={L2_2d*100:.1f} cm  (split diff {split_diff*100:.1f} cm)')

        if rms_elbow > SOLVE_RMS_TOLERANCE_M:
            print(f'\n[WARNING] Elbow residual RMS ({rms_elbow*100:.2f} cm) is high.')
            print('  Possible causes:')
            print('  - Arm did not move enough / elbow angle was not varied')
            print('  - Transmitter shifted during the motion')
            answer = input('Save anyway? [y/N]: ').strip().lower()
            if answer != 'y':
                print('[Calibration] Cancelled. Please try Step 1 again.')
                return

        if split_diff > 0.03 and L1_2d is not None:
            print(f'\n[NOTE] Constrained and unconstrained solvers differ by '
                  f'{split_diff*100:.1f} cm in L1.')
            print('  This often means the elbow was not varied enough in Step 1.')
            print('  Using the constrained result (more stable).')

        # Sanity check on individual values
        for name, val, lo, hi in [('upper arm', L1, 0.15, 0.45),
                                   ('forearm',   L2, 0.15, 0.35)]:
            if not (lo < val < hi):
                print(f'\n[WARNING] {name} = {val*100:.1f} cm is outside the '
                      f'typical range ({lo*100:.0f}–{hi*100:.0f} cm).')

        # ================================================================
        # STEP 2 — Rest pose: arm hanging straight down
        # ================================================================
        print('=' * 60)
        print('STEP 2: Rest pose — record the "arm down" direction')
        print()
        print('  Let your arm hang STRAIGHT DOWN, relaxed at your side.')
        print('  Elbow roughly straight (not bent).')
        print('  Press ENTER, then hold still for 5 seconds.')
        print()
        print('  WHY: the EM transmitter may not sit perfectly vertical')
        print('       on your shoulder.  This step records the true')
        print('       "arm hanging down" direction so the retargeting')
        print('       can compensate for any tilt — fixing the roll offset.')
        print('=' * 60)
        input('Press ENTER, then hold arm straight down...')

        samples_2 = self._collect_samples_timed(5.0)
        arm_down_hat = self._compute_elbow_hat(
            samples_2, shoulder_joint, L2,
            label='arm-down', fallback=np.array([0.0, 0.0, -1.0]))

        roll_offset = math.degrees(
            math.asin(float(np.clip(arm_down_hat[1], -1.0, 1.0))))
        print(f'\n  → Arm-down direction in transmitter frame:')
        print(f'    [{arm_down_hat[0]:+.4f}, {arm_down_hat[1]:+.4f}, '
              f'{arm_down_hat[2]:+.4f}]')
        print(f'  → Vertical tilt to correct: {roll_offset:+.1f}°')

        # ================================================================
        # STEP 3 — Forward reference: arm raised straight forward
        # ================================================================
        print('=' * 60)
        print('STEP 3: Forward reference — align the horizontal frame')
        print()
        print('  Raise your arm STRAIGHT FORWARD to roughly shoulder height.')
        print('  Keep the elbow roughly straight.')
        print('  Press ENTER, then hold the pose still for 5 seconds.')
        print()
        print('  WHY: the transmitter may be rotated horizontally on your')
        print('       shoulder.  This corrects the roll error that appears')
        print('       when the arm is raised forward.')
        print('=' * 60)
        input('Press ENTER, then hold arm straight forward...')

        samples_3 = self._collect_samples_timed(5.0)
        arm_forward_hat = self._compute_elbow_hat(
            samples_3, shoulder_joint, L2,
            label='arm-forward', fallback=None)

        if arm_forward_hat is None:
            print('[WARNING] Could not determine forward direction — '
                  'horizontal correction will be skipped.')
            arm_forward_hat = np.array([1.0, 0.0, 0.0])  # neutral fallback

        # Report the horizontal angle (how far the transmitter X-axis is
        # from "forward" in the robot-aligned XY plane)
        # We use a quick preview: apply the arm-down correction and check XY angle.
        R_preview = _rotation_between(arm_down_hat, np.array([0.0, 0.0, -1.0]))
        fwd_aligned = R_preview @ arm_forward_hat
        horiz_angle = math.degrees(
            math.atan2(float(fwd_aligned[1]), float(fwd_aligned[0])))
        print(f'\n  → Arm-forward direction in transmitter frame:')
        print(f'    [{arm_forward_hat[0]:+.4f}, {arm_forward_hat[1]:+.4f}, '
              f'{arm_forward_hat[2]:+.4f}]')
        print(f'  → Horizontal rotation to correct: {horiz_angle:+.1f}°')

        # ================================================================
        # Compute yaw offset from Step 3 samples
        # ================================================================
        # The receiver sensor is mounted on the wrist strap at some angle
        # around the forearm axis.  This means the sensor Y-axis (FOREARM_LOCAL_AXIS)
        # doesn't perfectly align with the anatomical elbow-to-wrist direction,
        # producing a constant yaw bias.
        #
        # Solution: compute R_align now (same as retargeting_node), then measure
        # the yaw at the Step 3 "arm forward, arm straight" position.  That
        # measured yaw is the sensor mounting offset — subtract it in retargeting.
        R1_cal = _rotation_between(arm_down_hat, np.array([0.0, 0.0, -1.0]))
        fwd_after_R1 = R1_cal @ arm_forward_hat
        fxy = fwd_after_R1[:2]
        fxy_norm = float(np.linalg.norm(fxy))
        if fxy_norm > 0.1:
            theta = math.atan2(float(fxy[1]), float(fxy[0]))
            ct, st = math.cos(theta), math.sin(theta)
            R2_cal = np.array([[ ct, st, 0.0],
                               [-st, ct, 0.0],
                               [0.0, 0.0, 1.0]])
        else:
            R2_cal = np.eye(3)
        R_align_cal = R2_cal @ R1_cal

        yaw_offset = self._compute_yaw_offset(
            samples_3, shoulder_joint, L2, R_align_cal)

        print(f'  → Yaw offset (sensor mounting): {math.degrees(yaw_offset):+.1f}°')
        print(f'    (subtracted from shoulder_yaw in retargeting; '
              f'arm-forward pose will read 0°)')

        wrist_roll_ref_quat = self._compute_wrist_roll_ref_quat(samples_3)
        print(f'  → Wrist roll reference quaternion saved '
              f'(Step 3 pose = wrist_roll 0° on robot)')

        self._save_calibration(L1, L2, computed_total, shoulder_joint,
                               arm_down_hat, arm_forward_hat, yaw_offset,
                               wrist_roll_ref_quat)
        print(f'\n[Calibration] Saved to {CALIBRATION_FILE}')
        print('[Calibration] You can now start retargeting_node.')

    # ------------------------------------------------------------------
    # Reference direction helpers
    # ------------------------------------------------------------------

    def _compute_elbow_hat(self, samples, shoulder_joint, L2,
                           label: str, fallback: np.ndarray) -> np.ndarray:
        """
        Compute the mean unit vector from shoulder to elbow for a held pose.

        E_i = (W_i − shoulder_joint) − L2 · f_i   (elbow in shoulder-centred frame)

        Returns a normalised shape-(3,) vector, or `fallback` if the result
        is too short (arm not held in the expected position).
        """
        positions = np.array([
            [s.pose.position.x, s.pose.position.y, s.pose.position.z]
            for s in samples
        ])
        quats = np.array([
            [s.pose.orientation.w, s.pose.orientation.x,
             s.pose.orientation.y, s.pose.orientation.z]
            for s in samples
        ])
        forearm_dirs = np.array([
            _quat_to_matrix(q) @ FOREARM_LOCAL_AXIS for q in quats
        ])
        V     = positions - shoulder_joint          # wrist in shoulder-centred frame
        E_vecs = V - L2 * forearm_dirs              # elbow in shoulder-centred frame
        E_mean = np.mean(E_vecs, axis=0)
        norm   = np.linalg.norm(E_mean)
        if norm < 0.05:
            self.get_logger().warn(
                f'{label} reference vector is very short ({norm*100:.1f} cm) — '
                f'arm may not have been in the correct pose.  Using fallback.')
            return fallback
        return E_mean / norm

    def _compute_wrist_roll_ref_quat(self, samples_3) -> list:
        """
        Compute the mean sensor quaternion from Step 3 samples.
        Saved as the wrist_roll reference: the retargeting measures forearm
        axial rotation (pronation/supination) relative to this pose, mapping
        it to left_wrist_roll_joint.  Step 3 pose → wrist_roll = 0 on robot.
        """
        if not samples_3:
            return [1.0, 0.0, 0.0, 0.0]
        qs = np.array([
            [s.pose.orientation.w, s.pose.orientation.x,
             s.pose.orientation.y, s.pose.orientation.z]
            for s in samples_3
        ], dtype=float)
        q0 = qs[0].copy()
        for i in range(1, len(qs)):
            if np.dot(qs[i], q0) < 0.0:
                qs[i] = -qs[i]
        q_mean = np.mean(qs, axis=0)
        q_mean /= np.linalg.norm(q_mean)
        return [round(float(v), 6) for v in q_mean]

    def _compute_yaw_offset(self, samples, shoulder_joint, L2, R_align) -> float:
        """
        Compute the mean shoulder yaw at the Step 3 "arm forward, arm straight"
        pose.  This is the sensor mounting offset around the forearm axis.
        Subtracting it in retargeting makes yaw = 0 at the calibration pose.

        Uses the same yaw formula as retargeting_node._compute_joint_angles.
        Samples where the arm is too close to vertical are skipped.
        Returns the circular mean yaw in radians (0.0 if no valid samples).
        """
        world_down = np.array([0.0, 0.0, -1.0])
        yaws = []

        for s in samples:
            W = np.array([s.pose.position.x, s.pose.position.y,
                          s.pose.position.z]) - shoulder_joint
            q = np.array([s.pose.orientation.w, s.pose.orientation.x,
                          s.pose.orientation.y, s.pose.orientation.z])
            forearm_dir_raw = _quat_to_matrix(q) @ FOREARM_LOCAL_AXIS

            E_raw = W - L2 * forearm_dir_raw
            if np.linalg.norm(E_raw) < 1e-6:
                continue

            E_al = R_align @ E_raw
            fd_al = R_align @ forearm_dir_raw
            E_hat = E_al / np.linalg.norm(E_al)

            cos_to_down = float(np.dot(E_hat, world_down))
            if cos_to_down > math.cos(math.radians(20)):
                continue  # too close to vertical — yaw is ill-defined

            ref = world_down - float(np.dot(world_down, E_hat)) * E_hat
            ref_norm = float(np.linalg.norm(ref))
            fp = fd_al - float(np.dot(fd_al, E_hat)) * E_hat
            fp_norm = float(np.linalg.norm(fp))

            if ref_norm > 0.15 and fp_norm > 0.05:
                ref_n = ref / ref_norm
                fp_n = fp / fp_norm
                cross = np.cross(ref_n, fp_n)
                yaw = math.atan2(float(np.dot(cross, E_hat)),
                                 float(np.dot(ref_n, fp_n)))
                yaws.append(yaw)

        if not yaws:
            print('  [WARNING] Could not compute yaw offset — '
                  'no valid Step 3 samples (arm may be too close to vertical).')
            return 0.0

        angles = np.array(yaws)
        mean_yaw = math.atan2(float(np.mean(np.sin(angles))),
                              float(np.mean(np.cos(angles))))
        return mean_yaw

    # ------------------------------------------------------------------
    # Core solver
    # ------------------------------------------------------------------

    def _solve_arm_lengths_simultaneous(self, samples, shoulder_joint):
        """
        Solve for L1 and L2 simultaneously from many arm poses.

        For each pose:
          V = W − S,  f = forearm_dir
          Constraint: |V − L2·f|² = L1²
          Expanded:   2·(V·f)·L2 + (L1²−L2²) = |V|²

        Let u = L1²−L2².  Build the system A @ [L2, u]ᵀ = b:
          A[i,:] = [2·(V_i·f_i),  1]
          b[i]   = |V_i|²

        Solve with lstsq, then L1 = sqrt(L2² + u).

        Returns (L1, L2, rms_m) or (None, None, inf) on failure.
        """
        positions = np.array([
            [s.pose.position.x, s.pose.position.y, s.pose.position.z]
            for s in samples
        ])
        quats = np.array([
            [s.pose.orientation.w, s.pose.orientation.x,
             s.pose.orientation.y, s.pose.orientation.z]
            for s in samples
        ])

        # Forearm directions
        forearm_dirs = np.array([
            _quat_to_matrix(q) @ FOREARM_LOCAL_AXIS for q in quats
        ])  # (N, 3)

        V = positions - shoulder_joint    # (N, 3)
        Vdotf = np.sum(V * forearm_dirs, axis=1)   # (N,)
        V_sq  = np.sum(V ** 2,           axis=1)   # (N,)

        # Build linear system
        A = np.column_stack([2.0 * Vdotf, np.ones(len(samples))])
        b = V_sq

        sol, _, rank, sv = np.linalg.lstsq(A, b, rcond=None)

        # Condition check: if the two columns are nearly linearly dependent
        # (V·f barely varies), the system is ill-conditioned.
        cond = sv[0] / sv[-1] if sv[-1] > 1e-12 else float('inf')
        if cond > 1e6:
            print(f'  [Solver] Condition number too high ({cond:.0f}) — '
                  'not enough variation in elbow angle.')
            return None, None, float('inf')

        L2 = float(sol[0])
        u  = float(sol[1])   # u = L1² − L2²

        L1_sq = L2**2 + u
        if L2 <= 0.05 or L1_sq <= 0.0:
            return None, None, float('inf')

        L1 = math.sqrt(L1_sq)

        # RMS of elbow-distance residuals (interpretable: metres)
        computed_elbows = positions - L2 * forearm_dirs   # (N, 3)
        elbow_dists = np.linalg.norm(computed_elbows - shoulder_joint, axis=1)
        rms = float(np.sqrt(np.mean((elbow_dists - L1) ** 2)))

        return L1, L2, rms

    def _solve_arm_lengths_constrained(self, samples, shoulder_joint, L_total):
        """
        Solve for L2 (and L1 = L_total − L2) using the sphere radius as a
        hard constraint on the total arm length.

        WHY: the unconstrained 2D solve finds [L2, u=L1²−L2²] together.
        Slope (L2) and intercept (u) are negatively correlated in the
        regression — if L2 is overestimated, u drops by the same amount,
        keeping L1²=L2²+u roughly constant.  This makes L1 and L2
        individually unstable between sessions even though L1+L2 is stable.

        Fixing L1+L2 = sphere_radius (reliable from Step 0) eliminates the
        correlation and reduces the problem to a 1-D least squares in L2:

          |V − L2·f|² = (L_total − L2)²
          ↓ expand both sides, cancel L2² terms
          L2 · 2·(V·f − L_total) = |V|² − L_total²

        Stack one row per pose → 1-D normal equations → unique L2.

        Returns (L1, L2, rms_m) or (None, None, inf) on failure.
        """
        positions = np.array([
            [s.pose.position.x, s.pose.position.y, s.pose.position.z]
            for s in samples
        ])
        quats = np.array([
            [s.pose.orientation.w, s.pose.orientation.x,
             s.pose.orientation.y, s.pose.orientation.z]
            for s in samples
        ])
        forearm_dirs = np.array([
            _quat_to_matrix(q) @ FOREARM_LOCAL_AXIS for q in quats
        ])

        V     = positions - shoulder_joint
        Vdotf = np.sum(V * forearm_dirs, axis=1)
        V_sq  = np.sum(V ** 2,           axis=1)

        # 1-D system: a_i * L2 = b_i
        a = 2.0 * (Vdotf - L_total)   # (N,) — coefficient of L2
        b = V_sq - L_total ** 2        # (N,) — right-hand side

        # Normal equation for 1-D least squares: L2 = (aᵀb) / (aᵀa)
        AtA = float(np.dot(a, a))
        Atb = float(np.dot(a, b))

        if AtA < 1e-6:
            return None, None, float('inf')

        L2 = Atb / AtA
        L1 = L_total - L2

        if L2 <= 0.05 or L1 <= 0.05:
            return None, None, float('inf')

        # RMS of elbow-distance residuals (interpretable: metres)
        computed_elbows = positions - L2 * forearm_dirs
        elbow_dists = np.linalg.norm(computed_elbows - shoulder_joint, axis=1)
        rms = float(np.sqrt(np.mean((elbow_dists - L1) ** 2)))

        return L1, L2, rms

    # ------------------------------------------------------------------
    # Sample collection
    # ------------------------------------------------------------------

    def _collect_samples_timed(self, duration_sec: float) -> list:
        import time
        samples = []
        last_stamp = None
        end_time = time.time() + duration_sec
        print(f'  Sweeping for {duration_sec:.0f} s ', end='', flush=True)
        while time.time() < end_time:
            with self._pose_lock:
                msg = self.latest_pose
            if msg is not None:
                stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
                if stamp != last_stamp:
                    samples.append(msg)
                    last_stamp = stamp
                    if len(samples) % 50 == 0:
                        print('.', end='', flush=True)
            time.sleep(0.01)
        print(f' done ({len(samples)} samples)')
        return samples

    # ------------------------------------------------------------------
    # Sphere fit
    # ------------------------------------------------------------------

    def _fit_sphere(self, samples: list):
        """
        Algebraic least-squares sphere fit.
        Returns (center [m, shape (3,)], radius [m], rms_error [m]).
        """
        pts = np.array([
            [s.pose.position.x, s.pose.position.y, s.pose.position.z]
            for s in samples
        ])
        A_mat = np.column_stack([pts, np.ones(len(pts))])
        b_vec = -(pts[:, 0]**2 + pts[:, 1]**2 + pts[:, 2]**2)
        coeffs, _, _, _ = np.linalg.lstsq(A_mat, b_vec, rcond=None)
        A, B, C, D = coeffs
        cx, cy, cz = -A / 2.0, -B / 2.0, -C / 2.0
        radius = math.sqrt(max(cx**2 + cy**2 + cz**2 - D, 0.0))
        center = np.array([cx, cy, cz])
        dists = np.linalg.norm(pts - center, axis=1)
        rms = float(np.sqrt(np.mean((dists - radius) ** 2)))
        return center, radius, rms

    # ------------------------------------------------------------------
    # YAML load / save
    # ------------------------------------------------------------------

    def _save_calibration(self, upper_arm_m, forearm_m, total_m,
                          shoulder_joint, arm_down_hat, arm_forward_hat,
                          yaw_offset: float = 0.0,
                          wrist_roll_ref_quat: list = None):
        os.makedirs(os.path.dirname(CALIBRATION_FILE), exist_ok=True)
        data = {
            'calibration': {
                'upper_arm_length_m':  round(upper_arm_m, 4),
                'forearm_length_m':    round(forearm_m, 4),
                'total_arm_length_m':  round(total_m, 4),
                'shoulder_offset_x_m': round(float(shoulder_joint[0]), 4),
                'shoulder_offset_y_m': round(float(shoulder_joint[1]), 4),
                'shoulder_offset_z_m': round(float(shoulder_joint[2]), 4),
                # Unit vector from shoulder toward elbow when arm hangs straight
                # down, in the EM transmitter frame.
                # Used by retargeting_node to correct for vertical tilt.
                'arm_down_hat_x': round(float(arm_down_hat[0]), 4),
                'arm_down_hat_y': round(float(arm_down_hat[1]), 4),
                'arm_down_hat_z': round(float(arm_down_hat[2]), 4),
                # Unit vector from shoulder toward elbow when arm is raised
                # straight forward, in the EM transmitter frame.
                # Used to correct for horizontal (azimuthal) rotation of the
                # transmitter — fixes the roll error on forward raises.
                'arm_forward_hat_x': round(float(arm_forward_hat[0]), 4),
                'arm_forward_hat_y': round(float(arm_forward_hat[1]), 4),
                'arm_forward_hat_z': round(float(arm_forward_hat[2]), 4),
                # Shoulder yaw measured at the Step 3 "arm forward, arm straight"
                # pose.  This is the sensor mounting rotation around the forearm
                # axis.  Subtracted from every yaw reading in retargeting so that
                # the calibration pose gives yaw = 0°.
                'yaw_offset': round(float(yaw_offset), 4),
                # Sensor quaternion [w,x,y,z] at Step 3 (arm forward, wrist neutral).
                # Used as the zero-reference for wrist_roll tracking: the retargeting
                # measures how much the forearm has rotated around its own axis
                # (pronation/supination) relative to this pose.
                'wrist_roll_ref_quat': wrist_roll_ref_quat if wrist_roll_ref_quat else [1.0, 0.0, 0.0, 0.0],
                'calibrated_at': datetime.now(timezone.utc).isoformat(),
            }
        }
        with open(CALIBRATION_FILE, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

    def _load_calibration(self) -> dict:
        with open(CALIBRATION_FILE, 'r') as f:
            data = yaml.safe_load(f)
        return data['calibration']


# -----------------------------------------------------------------------
# Static helper — used by retargeting_node.py
# -----------------------------------------------------------------------

def load_calibration() -> dict:
    if not os.path.exists(CALIBRATION_FILE):
        raise FileNotFoundError(
            f'Calibration file not found at {CALIBRATION_FILE}. '
            'Please run calibration_node first.'
        )
    with open(CALIBRATION_FILE, 'r') as f:
        data = yaml.safe_load(f)
    return data['calibration']


# -----------------------------------------------------------------------

def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """Quaternion [w, x, y, z] → 3×3 rotation matrix (local → world)."""
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - z*w),    2*(x*z + y*w)],
        [    2*(x*y + z*w),  1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [    2*(x*z - y*w),  2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


def _rotation_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """
    3×3 rotation matrix R such that R @ a ≈ b (both unit vectors).
    Uses Rodrigues' rotation formula.
    """
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    s = float(np.linalg.norm(v))
    c = float(np.dot(a, b))
    if s < 1e-8:
        if c > 0:
            return np.eye(3)
        perp = np.array([1.0, 0.0, 0.0])
        if abs(float(np.dot(perp, a))) > 0.9:
            perp = np.array([0.0, 1.0, 0.0])
        ax = np.cross(a, perp)
        ax /= np.linalg.norm(ax)
        return 2.0 * np.outer(ax, ax) - np.eye(3)
    vx = np.array([[   0, -v[2],  v[1]],
                   [v[2],     0, -v[0]],
                   [-v[1],  v[0],    0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / (s * s))


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
