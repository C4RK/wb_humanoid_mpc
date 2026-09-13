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


CALIBRATION_FILE = os.path.expanduser('~/.ros/wmet_calibration.yaml')

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
        L1, L2, rms_elbow = self._solve_arm_lengths_simultaneous(
            samples_1, shoulder_joint)

        if L1 is None:
            print('\n[ERROR] Linear solve failed — no valid solution found.')
            print('  Likely cause: elbow angle did not vary enough.')
            print('  Please try again and make sure to bend/straighten the elbow.')
            return

        computed_total = L1 + L2
        crosscheck = abs(computed_total - sphere_radius)

        print(f'\n  → upper_arm_length (L1): {L1*100:.1f} cm')
        print(f'  → forearm_length   (L2): {L2*100:.1f} cm')
        print(f'  → L1 + L2:              {computed_total*100:.1f} cm')
        print(f'  → Sphere radius (Step 0): {sphere_radius*100:.1f} cm  '
              f'(cross-check, diff = {crosscheck*100:.1f} cm)')
        print(f'  → Elbow residual RMS:    {rms_elbow*100:.2f} cm')

        if rms_elbow > SOLVE_RMS_TOLERANCE_M:
            print(f'\n[WARNING] Elbow residual RMS ({rms_elbow*100:.2f} cm) is high.')
            print('  Possible causes:')
            print('  - Arm did not move enough / elbow angle was not varied')
            print('  - Transmitter shifted during the motion')
            answer = input('Save anyway? [y/N]: ').strip().lower()
            if answer != 'y':
                print('[Calibration] Cancelled. Please try Step 1 again.')
                return

        if crosscheck > CROSSCHECK_TOLERANCE_M:
            print(f'\n[NOTE] L1+L2 vs sphere radius differ by {crosscheck*100:.1f} cm.')
            print('  The elbow may have been slightly bent during Step 0.')
            print('  L1 and L2 from the linear solve are independent of this.')

        # Sanity check on individual values
        for name, val, lo, hi in [('upper arm', L1, 0.15, 0.45),
                                   ('forearm',   L2, 0.15, 0.35)]:
            if not (lo < val < hi):
                print(f'\n[WARNING] {name} = {val*100:.1f} cm is outside the '
                      f'typical range ({lo*100:.0f}–{hi*100:.0f} cm).')

        self._save_calibration(L1, L2, computed_total, shoulder_joint)
        print(f'\n[Calibration] Saved to {CALIBRATION_FILE}')
        print('[Calibration] You can now start retargeting_node.')

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

    def _save_calibration(self, upper_arm_m, forearm_m, total_m, shoulder_joint):
        os.makedirs(os.path.dirname(CALIBRATION_FILE), exist_ok=True)
        data = {
            'calibration': {
                'upper_arm_length_m':  round(upper_arm_m, 4),
                'forearm_length_m':    round(forearm_m, 4),
                'total_arm_length_m':  round(total_m, 4),
                'shoulder_offset_x_m': round(float(shoulder_joint[0]), 4),
                'shoulder_offset_y_m': round(float(shoulder_joint[1]), 4),
                'shoulder_offset_z_m': round(float(shoulder_joint[2]), 4),
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
