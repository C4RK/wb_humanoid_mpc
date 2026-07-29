#!/usr/bin/env python3

"""
calibration_node.py

Determines the user's arm lengths (upper_arm and forearm) from two calibration
poses using the WMET EM tracker. Results are saved to a YAML file and reused
in every session without repeating the calibration.

Calibration procedure (two poses):

  Pose 1 — Arm hanging straight down:
    User stands upright, arm fully relaxed at side, elbow straight.
    The entire arm (upper_arm + forearm) hangs vertically.
    → total_arm_length = |position_mm| / 1000

  Pose 2 — Elbow bent 90°, upper arm hanging, forearm horizontal:
    User keeps upper arm hanging straight down, bends elbow to exactly 90°,
    forearm points horizontally forward.
    → upper_arm_length = |position.z|   (vertical component = upper arm)
    → forearm_length   = sqrt(x² + y²)  (horizontal component = forearm)

Coordinate frame: transmitter is at shoulder (origin).
"""

import os
import math
import yaml
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from datetime import datetime, timezone


# Where the calibration file is saved.
# ~/.ros/wmet_calibration.yaml — always accessible regardless of workspace.
CALIBRATION_FILE = os.path.expanduser('~/.ros/wmet_calibration.yaml')

# How many pose samples to average per calibration pose (at 50 Hz → 2 seconds).
NUM_SAMPLES = 100

# Maximum allowed difference between total arm length (Pose 1) and
# upper_arm + forearm (Pose 2). If exceeded, calibration is likely wrong.
VALIDATION_TOLERANCE_M = 0.03  # 3 cm


class CalibrationNode(Node):

    def __init__(self):
        super().__init__('calibration_node')

        # Stores the most recent pose received from the EM tracker.
        self.latest_pose = None
        self._pose_lock = threading.Lock()

        # Subscribe to the EM tracker topic published by em_tracker_node.py
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/em/pose',
            self._on_pose_received,
            10
        )

        self.get_logger().info('Calibration node started. Waiting for /em/pose ...')

        # Run the calibration sequence in a background thread so that
        # rclpy.spin() continues running and subscriber callbacks fire normally.
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
    # Calibration sequence (runs in background thread)
    # ------------------------------------------------------------------

    def _run_calibration_sequence(self):
        """
        Guides the user through two calibration poses interactively.
        Runs in a separate thread; uses input() to wait for the user.
        """

        # ---- Check for existing calibration ----
        if os.path.exists(CALIBRATION_FILE):
            print(f'\n[Calibration] Existing calibration found at {CALIBRATION_FILE}')
            cal = self._load_calibration()
            print(f'  upper_arm_length = {cal["upper_arm_length_m"]*100:.1f} cm')
            print(f'  forearm_length   = {cal["forearm_length_m"]*100:.1f} cm')
            answer = input('\nUse existing calibration? [Y/n]: ').strip().lower()
            if answer != 'n':
                print('[Calibration] Using existing calibration. Node will now exit.')
                return

        # ---- Wait for first pose to arrive ----
        print('\n[Calibration] Waiting for EM tracker data on /em/pose ...')
        while rclpy.ok():
            with self._pose_lock:
                if self.latest_pose is not None:
                    break
            import time; time.sleep(0.1)
        print('[Calibration] EM tracker data received.\n')

        # ---- Pose 1: Arm hanging straight down ----
        print('=' * 55)
        print('POSE 1: Arm hanging straight down')
        print('  Stand upright. Let your arm hang fully relaxed.')
        print('  Keep elbow straight. Do NOT move during recording.')
        print('=' * 55)
        input('Press ENTER when ready...')

        samples_1 = self._collect_samples(NUM_SAMPLES)
        P1 = self._average_position(samples_1)
        total_arm_length = math.sqrt(P1[0]**2 + P1[1]**2 + P1[2]**2)

        print(f'  Recorded position: x={P1[0]*100:.1f} cm, y={P1[1]*100:.1f} cm, z={P1[2]*100:.1f} cm')
        print(f'  → Total arm length: {total_arm_length*100:.1f} cm\n')

        # ---- Pose 2: Elbow bent 90°, upper arm down, forearm horizontal ----
        print('=' * 55)
        print('POSE 2: Elbow bent 90°')
        print('  Keep upper arm hanging straight down.')
        print('  Bend elbow to exactly 90°.')
        print('  Forearm points horizontally forward.')
        print('  Do NOT move during recording.')
        print('=' * 55)
        input('Press ENTER when ready...')

        samples_2 = self._collect_samples(NUM_SAMPLES)
        P2 = self._average_position(samples_2)

        # In this pose:
        #   upper arm hangs down → z-component = -upper_arm_length
        #   forearm points forward → x,y-components = forearm_length
        upper_arm_length = abs(P2[2])
        forearm_length   = math.sqrt(P2[0]**2 + P2[1]**2)

        print(f'  Recorded position: x={P2[0]*100:.1f} cm, y={P2[1]*100:.1f} cm, z={P2[2]*100:.1f} cm')
        print(f'  → upper_arm_length: {upper_arm_length*100:.1f} cm')
        print(f'  → forearm_length:   {forearm_length*100:.1f} cm\n')

        # ---- Validation ----
        computed_total = upper_arm_length + forearm_length
        error = abs(computed_total - total_arm_length)
        print(f'Validation: Pose1 total = {total_arm_length*100:.1f} cm, '
              f'Pose2 sum = {computed_total*100:.1f} cm, '
              f'difference = {error*100:.1f} cm')

        if error > VALIDATION_TOLERANCE_M:
            print(f'\n[WARNING] Difference ({error*100:.1f} cm) exceeds tolerance '
                  f'({VALIDATION_TOLERANCE_M*100:.0f} cm).')
            print('  Possible causes:')
            print('  - Arm was not fully straight in Pose 1')
            print('  - Elbow was not exactly 90° in Pose 2')
            answer = input('Save anyway? [y/N]: ').strip().lower()
            if answer != 'y':
                print('[Calibration] Calibration cancelled. Please try again.')
                return
        else:
            print('[Calibration] Validation passed.')

        # ---- Save ----
        self._save_calibration(upper_arm_length, forearm_length, total_arm_length)
        print(f'\n[Calibration] Saved to {CALIBRATION_FILE}')
        print('[Calibration] You can now start retargeting_node.py.')

    # ------------------------------------------------------------------
    # Sample collection helpers
    # ------------------------------------------------------------------

    def _collect_samples(self, n: int) -> list:
        """
        Collects n pose samples from /em/pose.
        Waits for each new sample (skips duplicates by tracking the last stamp).
        Returns a list of PoseStamped messages.
        """
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
            time.sleep(0.01)  # poll at 100 Hz, data arrives at 50 Hz

        print(f' done ({n} samples)')
        return samples

    def _average_position(self, samples: list) -> list:
        """Returns the mean [x, y, z] position in meters from a list of PoseStamped."""
        x = sum(s.pose.position.x for s in samples) / len(samples)
        y = sum(s.pose.position.y for s in samples) / len(samples)
        z = sum(s.pose.position.z for s in samples) / len(samples)
        return [x, y, z]

    # ------------------------------------------------------------------
    # YAML load / save
    # ------------------------------------------------------------------

    def _save_calibration(self, upper_arm_m: float, forearm_m: float, total_m: float):
        """Writes calibration results to YAML file."""
        os.makedirs(os.path.dirname(CALIBRATION_FILE), exist_ok=True)
        data = {
            'calibration': {
                'upper_arm_length_m': round(upper_arm_m, 4),
                'forearm_length_m':   round(forearm_m, 4),
                'total_arm_length_m': round(total_m, 4),
                'calibrated_at': datetime.now(timezone.utc).isoformat(),
            }
        }
        with open(CALIBRATION_FILE, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

    def _load_calibration(self) -> dict:
        """Reads calibration results from YAML file. Returns the inner dict."""
        with open(CALIBRATION_FILE, 'r') as f:
            data = yaml.safe_load(f)
        return data['calibration']


# -----------------------------------------------------------------------
# Static helper — used by other nodes (retargeting_node.py)
# -----------------------------------------------------------------------

def load_calibration() -> dict:
    """
    Load calibration from the YAML file.
    Import this function in retargeting_node.py:

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
