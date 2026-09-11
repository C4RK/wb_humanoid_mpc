#!/usr/bin/env python3

"""
calibration_node.py

Determines the user's arm lengths and shoulder joint position using the
WMET EM tracker. Results are saved to a YAML file and reused every session.

Calibration procedure — three steps:

  Step 0 — Shoulder sphere:
    Extend arm fully straight (elbow locked). Slowly swing it in as many
    directions as possible for 8 seconds. The wrist traces a sphere whose
    centre is the shoulder joint.
      → shoulder_joint  (3D position in transmitter frame)
      → sphere_radius   (cross-check total arm length)

  Step 1 — Arm hanging straight down:
    Let the arm hang fully relaxed and straight. Record a 2-second average.
      → total_arm_length = |wrist − shoulder_joint|  (accurate, single static pose)
      → Verify that sphere_radius ≈ total_arm_length.

  Step 2 — Elbow bent (any angle, roughly 90° is ideal):
    Bend the elbow to any noticeable angle (60°–120° is fine).
    Hold still. The calibration uses the forearm orientation vector from the
    sensor, so the exact angle does not matter — only "not straight".

    Law-of-Cosines formula:
      V          = wrist − shoulder_joint
      d_sw       = |V|
      L2 = (L_total² − d_sw²) / (2 × (L_total − V · forearm_dir))
      L1 = L_total − L2

    This is exact regardless of elbow angle — no sweeping, no fixed reference
    surface, no precision poses required.

Why this works better than a second sphere fit:
  The elbow is a hinge joint, not ball-and-socket. Sweeping the forearm
  traces an arc (1-D), not a sphere (2-D surface). A sphere fit of an arc
  is numerically ill-conditioned and produces unreliable radii. The law-of-
  cosines formula instead exploits the receiver's orientation output, which
  is already highly accurate.

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


# Where the calibration file is saved.
CALIBRATION_FILE = os.path.expanduser('~/.ros/wmet_calibration.yaml')

# Duration of the shoulder sphere sweep, seconds.
SWEEP_SECONDS = 8.0

# Static-pose sample count (50 Hz × 2 s = 100 samples).
NUM_SAMPLES = 100

# Forearm direction in the receiver's local frame (empirically confirmed).
FOREARM_LOCAL_AXIS = np.array([0.0, 1.0, 0.0])

# Acceptable sphere-fit RMS error.
SPHERE_FIT_TOLERANCE_M = 0.03  # 3 cm

# Acceptable deviation between sphere radius and static total arm length.
ARM_LENGTH_CROSSCHECK_M = 0.05  # 5 cm


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
                ox = cal['shoulder_offset_x_m']
                oy = cal['shoulder_offset_y_m']
                oz = cal.get('shoulder_offset_z_m', 0.0)
                print(f'  shoulder_joint   = x={ox*100:.1f} cm, y={oy*100:.1f} cm, '
                      f'z={oz*100:.1f} cm  (transmitter frame)')
            answer = input('\nUse existing calibration? [Y/n]: ').strip().lower()
            if answer != 'n':
                print('[Calibration] Using existing calibration. Node will now exit.')
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
        print('  2. Press ENTER, then slowly swing your whole arm for')
        print(f'     {SWEEP_SECONDS:.0f} seconds in as many directions as possible:')
        print('       → forward, sideways, up, down, diagonal.')
        print('  3. Keep the elbow LOCKED the ENTIRE time.')
        print('  4. Move slowly and smoothly — fast motion is not needed.')
        print()
        print('  WHY: the wrist traces a sphere; its centre is your')
        print('       shoulder joint, regardless of transmitter tilt.')
        print('=' * 60)
        input('Press ENTER, then start sweeping...')

        pivot_samples = self._collect_samples_timed(SWEEP_SECONDS)
        shoulder_joint, sphere_radius, rms_err = self._fit_sphere(pivot_samples)

        print(f'\n  Shoulder joint (transmitter frame):')
        print(f'    x = {shoulder_joint[0]*100:+.1f} cm')
        print(f'    y = {shoulder_joint[1]*100:+.1f} cm')
        print(f'    z = {shoulder_joint[2]*100:+.1f} cm')
        print(f'  → Sphere radius (cross-check): {sphere_radius*100:.1f} cm')
        print(f'  → Fit error: {rms_err*100:.1f} cm RMS')

        if rms_err > SPHERE_FIT_TOLERANCE_M:
            print(f'\n[WARNING] Fit error ({rms_err*100:.1f} cm) > '
                  f'{SPHERE_FIT_TOLERANCE_M*100:.0f} cm.')
            print('  Likely cause: elbow was not kept locked straight.')
            answer = input('Retry? [Y/n]: ').strip().lower()
            if answer != 'n':
                print('[Calibration] Please redo Step 0 with elbow locked.')
                return
        else:
            print('[Calibration] Step 0 passed.\n')

        # ================================================================
        # STEP 1 — Arm hanging straight down (accurate total arm length)
        # ================================================================
        print('=' * 60)
        print('STEP 1: Arm hanging straight down')
        print()
        print('  Stand upright. Let your arm hang fully relaxed at your side.')
        print('  Keep elbow STRAIGHT. Do NOT move during recording.')
        print()
        print('  This gives the accurate total arm length (shoulder to wrist).')
        print('=' * 60)
        input('Press ENTER when ready...')

        samples_1 = self._collect_samples(NUM_SAMPLES)
        pos_1, _ = self._average_pose(samples_1)
        P1 = np.array(pos_1)

        P1_rel = P1 - shoulder_joint
        total_arm_length = float(np.linalg.norm(P1_rel))

        print(f'  Recorded wrist: x={P1[0]*100:.1f} cm, y={P1[1]*100:.1f} cm, '
              f'z={P1[2]*100:.1f} cm')
        print(f'  → Total arm length: {total_arm_length*100:.1f} cm  '
              f'(sphere radius was {sphere_radius*100:.1f} cm)')

        crosscheck = abs(total_arm_length - sphere_radius)
        if crosscheck > ARM_LENGTH_CROSSCHECK_M:
            print(f'\n[WARNING] Arm length vs sphere radius differ by '
                  f'{crosscheck*100:.1f} cm (>{ARM_LENGTH_CROSSCHECK_M*100:.0f} cm).')
            print('  Possible causes:')
            print('  - Elbow was bent during Step 0 sweep (sphere radius too small)')
            print('  - Arm was not fully straight in Step 1')
            answer = input('Continue anyway? [y/N]: ').strip().lower()
            if answer != 'y':
                print('[Calibration] Cancelled. Please redo.')
                return
        else:
            print(f'  → Cross-check passed (diff = {crosscheck*100:.1f} cm).\n')

        # ================================================================
        # STEP 2 — Elbow bent: compute forearm length from law of cosines
        # ================================================================
        print('=' * 60)
        print('STEP 2: Elbow bent — compute arm segment lengths')
        print()
        print('  Bend your elbow to roughly 90° (anywhere from 60° to 120°')
        print('  works — the exact angle does NOT matter).')
        print()
        print('  Keep the upper arm naturally at your side.')
        print('  Hold completely still. Do NOT move during recording.')
        print()
        print('  HOW IT WORKS: the sensor already knows your forearm direction.')
        print('  Combined with the shoulder position (Step 0) and total arm')
        print('  length (Step 1), the exact split between upper arm and forearm')
        print('  is solved mathematically from the law of cosines.')
        print('=' * 60)
        input('Press ENTER when ready...')

        samples_2 = self._collect_samples(NUM_SAMPLES)
        pos_2, q_avg = self._average_pose(samples_2)
        P2 = np.array(pos_2)

        # Forearm direction from receiver orientation.
        R = _quat_to_matrix(np.array(q_avg))
        forearm_dir = R @ FOREARM_LOCAL_AXIS  # unit vector: elbow → wrist

        # Law of cosines in the shoulder-elbow-wrist triangle.
        V = P2 - shoulder_joint              # shoulder → wrist vector
        d_sw = float(np.linalg.norm(V))      # shoulder-to-wrist distance
        V_dot_f = float(np.dot(V, forearm_dir))

        denom = 2.0 * (total_arm_length - V_dot_f)

        if abs(denom) < 1e-4:
            print('\n[ERROR] Arm appears to be nearly straight in Step 2.')
            print('  Please bend the elbow to at least 30° and try again.')
            return

        forearm_length = (total_arm_length**2 - d_sw**2) / denom
        upper_arm_length = total_arm_length - forearm_length

        print(f'\n  Recorded wrist: x={P2[0]*100:.1f} cm, y={P2[1]*100:.1f} cm, '
              f'z={P2[2]*100:.1f} cm')
        print(f'  Shoulder-to-wrist distance: {d_sw*100:.1f} cm')
        print(f'  Forearm direction: [{forearm_dir[0]:+.3f}, {forearm_dir[1]:+.3f}, '
              f'{forearm_dir[2]:+.3f}]')
        print()
        print(f'  → upper_arm_length: {upper_arm_length*100:.1f} cm')
        print(f'  → forearm_length:   {forearm_length*100:.1f} cm')
        print(f'  → L1 + L2:          {(upper_arm_length+forearm_length)*100:.1f} cm  '
              f'(= total arm ✓)')

        # Sanity checks
        ok = True
        if forearm_length <= 0.0 or forearm_length >= total_arm_length:
            print('\n[ERROR] Forearm length is out of range. Likely causes:')
            print('  - Arm was nearly straight in Step 2 (bend more)')
            print('  - Forearm sensor axis is wrong (flip FOREARM_LOCAL_AXIS)')
            ok = False
        if upper_arm_length <= 0.0:
            print('\n[ERROR] Upper arm length is negative. Check Step 0 and Step 2.')
            ok = False
        if ok and (forearm_length < 0.15 or forearm_length > 0.40):
            print(f'\n[WARNING] Forearm length {forearm_length*100:.1f} cm seems '
                  'outside typical range (15–40 cm). Please verify.')
        if ok and (upper_arm_length < 0.15 or upper_arm_length > 0.45):
            print(f'\n[WARNING] Upper arm length {upper_arm_length*100:.1f} cm seems '
                  'outside typical range (15–45 cm). Please verify.')

        if not ok:
            return

        self._save_calibration(upper_arm_length, forearm_length,
                               total_arm_length, shoulder_joint)
        print(f'\n[Calibration] Saved to {CALIBRATION_FILE}')
        print('[Calibration] You can now start retargeting_node.')

    # ------------------------------------------------------------------
    # Sample collection helpers
    # ------------------------------------------------------------------

    def _collect_samples(self, n: int) -> list:
        """Collect n unique pose samples from /em/pose."""
        import time
        samples = []
        last_stamp = None
        print(f'  Recording {n} samples ', end='', flush=True)
        while len(samples) < n:
            with self._pose_lock:
                msg = self.latest_pose
            if msg is not None:
                stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
                if stamp != last_stamp:
                    samples.append(msg)
                    last_stamp = stamp
                    if len(samples) % 10 == 0:
                        print('.', end='', flush=True)
            time.sleep(0.01)
        print(f' done ({n} samples)')
        return samples

    def _collect_samples_timed(self, duration_sec: float) -> list:
        """Collect all unique pose samples during a timed sweep."""
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

    def _average_pose(self, samples: list):
        """
        Returns (mean_position [x,y,z], mean_quaternion [w,x,y,z]).
        Quaternion average: mean the components then re-normalise
        (valid for small orientation spread, as in a held-still pose).
        """
        pos = [
            sum(s.pose.position.x for s in samples) / len(samples),
            sum(s.pose.position.y for s in samples) / len(samples),
            sum(s.pose.position.z for s in samples) / len(samples),
        ]
        q = np.array([
            [s.pose.orientation.w, s.pose.orientation.x,
             s.pose.orientation.y, s.pose.orientation.z]
            for s in samples
        ]).mean(axis=0)
        q /= np.linalg.norm(q)
        return pos, q.tolist()

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
    """
    Load calibration from the YAML file.

        from remote_control.calibration_node import load_calibration
        cal = load_calibration()
        upper_arm = cal['upper_arm_length_m']
        forearm   = cal['forearm_length_m']
    """
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
    """Quaternion [w, x, y, z] → 3x3 rotation matrix (local → world)."""
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
