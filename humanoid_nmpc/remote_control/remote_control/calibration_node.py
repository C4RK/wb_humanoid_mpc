#!/usr/bin/env python3

"""
calibration_node.py

Determines the user's arm lengths and shoulder joint position using the
WMET EM tracker. Results are saved to a YAML file and reused every session.

Calibration procedure — two sphere fits:

  Step 0 — Shoulder sphere:
    Extend arm fully straight (elbow locked). Slowly swing it in as many
    directions as possible for 6 seconds. The wrist traces a sphere whose
    centre is the shoulder joint.
      → shoulder_joint (3D position in transmitter frame)
      → total_arm_length = sphere radius  (cross-check only)

  Step 1 — Elbow sphere:
    Press your elbow against a fixed point (corner of a wall, table edge).
    Keep the elbow joint stationary. Rotate ONLY the forearm in as many
    directions as possible for 6 seconds — the wrist sweeps a sphere
    centred at the elbow joint.
      → elbow_joint (3D position in transmitter frame)
      → forearm_length = sphere radius  (elbow to wrist)

  Compute:
      upper_arm_length = |elbow_joint − shoulder_joint|
      Validate: upper_arm + forearm ≈ total_arm (cross-check; not critical)

Why two sphere fits?
  - No required joint angle (no "exactly 90°", no "arm perfectly down").
  - No assumption about which axis is vertical or how the transmitter is tilted.
  - The sphere fit is robust to noisy data; result is independent of pose.
  - Each step isolates exactly one joint: shoulder (Step 0), elbow (Step 1).

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

# Duration of each sphere sweep in seconds.
SWEEP_SECONDS = 8.0

# Acceptable sphere-fit RMS error — 3 cm.
FIT_TOLERANCE_M = 0.03

# Acceptable difference between (L1 + L2) and the shoulder-sphere radius.
# This is a loose cross-check only; L1 and L2 themselves are accurate.
CROSSCHECK_TOLERANCE_M = 0.05  # 5 cm


class CalibrationNode(Node):

    def __init__(self):
        super().__init__('calibration_node')

        self.latest_pose = None
        self._pose_lock = threading.Lock()

        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/em/pose',
            self._on_pose_received,
            10
        )

        self.get_logger().info('Calibration node started. Waiting for /em/pose ...')

        self._calibration_thread = threading.Thread(
            target=self._run_calibration_sequence,
            daemon=True
        )
        self._calibration_thread.start()

    # ------------------------------------------------------------------
    # Subscriber callback
    # ------------------------------------------------------------------

    def _on_pose_received(self, msg: PoseStamped):
        """Stores the latest pose. Called by rclpy at 50 Hz."""
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
        print('  WHY: the wrist traces a sphere whose CENTRE is your')
        print('       shoulder joint — no matter how the transmitter is tilted.')
        print('=' * 60)
        input('Press ENTER, then start sweeping...')

        samples_0 = self._collect_samples_timed(SWEEP_SECONDS)
        shoulder_joint, shoulder_sphere_radius, rms_0 = self._fit_sphere(samples_0)

        print(f'\n  Shoulder joint (transmitter frame):')
        print(f'    x = {shoulder_joint[0]*100:+.1f} cm')
        print(f'    y = {shoulder_joint[1]*100:+.1f} cm')
        print(f'    z = {shoulder_joint[2]*100:+.1f} cm')
        print(f'  → Sphere radius (≈ total arm length): {shoulder_sphere_radius*100:.1f} cm')
        print(f'  → Fit error: {rms_0*100:.1f} cm RMS')

        if rms_0 > FIT_TOLERANCE_M:
            print(f'\n[WARNING] Fit error ({rms_0*100:.1f} cm) > {FIT_TOLERANCE_M*100:.0f} cm.')
            print('  Likely cause: elbow was not kept locked straight.')
            answer = input('Try again? [Y/n]: ').strip().lower()
            if answer != 'n':
                print('[Calibration] Cancelled. Please redo Step 0 with elbow locked.')
                return
        else:
            print('[Calibration] Step 0 passed.\n')

        # ================================================================
        # STEP 1 — Elbow sphere
        # ================================================================
        print('=' * 60)
        print('STEP 1: Elbow sphere — measure forearm length')
        print()
        print('  Goal: keep your ELBOW JOINT stationary while sweeping')
        print('        the wrist in as many directions as possible.')
        print()
        print('  How to fix the elbow:')
        print('    • Best:  press the point of your elbow into a wall corner.')
        print('    • Also:  rest the back of the elbow on a table edge.')
        print('    • Also:  grip the upper arm firmly with your other hand')
        print('             just above the elbow, pressing the elbow inward.')
        print()
        print('  Motion: once the elbow is fixed, move ONLY the forearm —')
        print('    rotate it up, down, left, right, in circles.')
        print('    The wrist should trace a sphere centred at the elbow.')
        print()
        print(f'  Sweep for {SWEEP_SECONDS:.0f} seconds after pressing ENTER.')
        print('=' * 60)
        input('Press ENTER, then start sweeping...')

        samples_1 = self._collect_samples_timed(SWEEP_SECONDS)
        elbow_joint, forearm_length, rms_1 = self._fit_sphere(samples_1)

        print(f'\n  Elbow joint (transmitter frame):')
        print(f'    x = {elbow_joint[0]*100:+.1f} cm')
        print(f'    y = {elbow_joint[1]*100:+.1f} cm')
        print(f'    z = {elbow_joint[2]*100:+.1f} cm')
        print(f'  → Forearm length (elbow to wrist): {forearm_length*100:.1f} cm')
        print(f'  → Fit error: {rms_1*100:.1f} cm RMS')

        if rms_1 > FIT_TOLERANCE_M:
            print(f'\n[WARNING] Fit error ({rms_1*100:.1f} cm) > {FIT_TOLERANCE_M*100:.0f} cm.')
            print('  Likely cause: elbow joint moved during the sweep.')
            answer = input('Try again? [Y/n]: ').strip().lower()
            if answer != 'n':
                print('[Calibration] Cancelled. Please redo Step 1 with elbow held still.')
                return
        else:
            print('[Calibration] Step 1 passed.\n')

        # ================================================================
        # Compute upper arm and validate
        # ================================================================
        upper_arm_length = float(np.linalg.norm(elbow_joint - shoulder_joint))
        computed_total = upper_arm_length + forearm_length

        print(f'  → upper_arm_length: {upper_arm_length*100:.1f} cm  '
              f'(distance between sphere centres)')
        print(f'  → forearm_length:   {forearm_length*100:.1f} cm  '
              f'(Step 1 sphere radius)')
        print(f'  → L1 + L2:          {computed_total*100:.1f} cm')
        print(f'  → Step 0 radius:    {shoulder_sphere_radius*100:.1f} cm  '
              f'(cross-check)')

        cross_error = abs(computed_total - shoulder_sphere_radius)
        print(f'  → Cross-check diff: {cross_error*100:.1f} cm  '
              f'(target < {CROSSCHECK_TOLERANCE_M*100:.0f} cm)')

        if cross_error > CROSSCHECK_TOLERANCE_M:
            print(f'\n[NOTE] Cross-check diff ({cross_error*100:.1f} cm) > '
                  f'{CROSSCHECK_TOLERANCE_M*100:.0f} cm.')
            print('  This usually means the elbow was slightly bent in Step 0,')
            print('  making the shoulder sphere radius smaller than L1+L2.')
            print('  L1 and L2 themselves (from sphere centres and Step 1 radius)')
            print('  are still accurate — this warning is informational only.')
            input('Press ENTER to save anyway...')
        else:
            print('[Calibration] Cross-check passed.')

        self._save_calibration(upper_arm_length, forearm_length,
                               computed_total, shoulder_joint)
        print(f'\n[Calibration] Saved to {CALIBRATION_FILE}')
        print('[Calibration] You can now start retargeting_node.')

    # ------------------------------------------------------------------
    # Sample collection helpers
    # ------------------------------------------------------------------

    def _collect_samples_timed(self, duration_sec: float) -> list:
        """
        Collects all unique pose samples received over a fixed duration.
        """
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

    def _fit_sphere(self, samples: list):
        """
        Algebraic least-squares sphere fit.

        Every point p on a sphere satisfies:
            |p - c|² = r²
        Rearranged: x² + y² + z² + Ax + By + Cz + D = 0
        which is linear in [A, B, C, D] → solved with numpy lstsq.

        Returns:
            center   — np.array([cx, cy, cz])  in metres
            radius   — float, in metres
            rms      — RMS distance residual, in metres
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
        r_sq = cx**2 + cy**2 + cz**2 - D
        radius = math.sqrt(max(r_sq, 0.0))

        center = np.array([cx, cy, cz])
        dists = np.linalg.norm(pts - center, axis=1)
        rms = float(np.sqrt(np.mean((dists - radius) ** 2)))

        return center, radius, rms

    # ------------------------------------------------------------------
    # YAML load / save
    # ------------------------------------------------------------------

    def _save_calibration(self, upper_arm_m: float, forearm_m: float,
                          total_m: float, shoulder_joint: np.ndarray):
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
    Import this in retargeting_node.py:

        from remote_control.calibration_node import load_calibration
        cal = load_calibration()
        upper_arm = cal['upper_arm_length_m']
        forearm   = cal['forearm_length_m']

    Raises FileNotFoundError if calibration has not been run yet.
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
