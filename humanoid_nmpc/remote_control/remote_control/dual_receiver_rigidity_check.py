#!/usr/bin/env python3

"""Check receiver consistency during controlled straight-arm flexion."""

from datetime import datetime, timezone
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from remote_control.dual_calibration_node import (
    CALIBRATION_LOG_DIR,
    PosePairSample,
    _message_pose,
    _stamp_ns,
)
from remote_control.dual_retargeting_kinematics import (
    ROTATION_CONVENTIONS,
    apply_rotation_convention,
    rigid_pair_consistency,
    rotation_error_deg,
)


SWEEP_SECONDS = 12.0


class DualReceiverRigidityCheck(Node):
    def __init__(self):
        super().__init__('dual_receiver_rigidity_check')
        self._pair_lock = threading.Lock()
        self._latest_pair = None
        self._pending_upper = {}
        self._pending_forearm = {}

        self.create_subscription(
            PoseStamped, '/em/upper_arm_pose', self._on_upper_pose, 20)
        self.create_subscription(
            PoseStamped, '/em/forearm_pose', self._on_forearm_pose, 20)
        self.get_logger().info(
            'Straight-arm receiver check started; waiting for synchronized poses.')
        self._worker = threading.Thread(
            target=self._run_guarded, daemon=True)
        self._worker.start()

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
        if upper.header.frame_id != forearm.header.frame_id:
            self.get_logger().warn('Receiver frame_id mismatch; pair dropped.')
        else:
            p_u, R_u = _message_pose(upper)
            p_f, R_f = _message_pose(forearm)
            with self._pair_lock:
                self._latest_pair = PosePairSample(
                    key, p_u, R_u, p_f, R_f)
        self._pending_upper.pop(key, None)
        self._pending_forearm.pop(key, None)
        self._trim_pending()

    def _trim_pending(self):
        for pending in (self._pending_upper, self._pending_forearm):
            while len(pending) > 30:
                pending.pop(min(pending))

    def _run_guarded(self):
        try:
            self._run()
        except (EOFError, KeyboardInterrupt):
            self.get_logger().info('Rigidity check cancelled.')
        except Exception as exc:
            self.get_logger().error(f'Rigidity check failed: {exc}')

    def _run(self):
        print('\n[Rigidity check] Waiting for synchronized receiver pairs...')
        while rclpy.ok():
            with self._pair_lock:
                ready = self._latest_pair is not None
            if ready:
                break
            time.sleep(0.1)
        if not rclpy.ok():
            return

        print('[Rigidity check] Both receivers are available.')
        print('\n' + '=' * 72)
        print('STRAIGHT-ARM FLEXION CHECK / 直臂前举检查')
        print('1. Keep the torso still and the LEFT elbow comfortably fully extended.')
        print('2. DOWN: palm faces RIGHT/inward; thumb points FORWARD.')
        print('3. Raise the whole straight arm FORWARD to shoulder height:')
        print('   palm still faces RIGHT/inward; thumb now points UP.')
        print('4. Return to DOWN and repeat slowly. Never turn the palm or move sideways.')
        print('1. 躯干保持不动，左肘舒适地完全伸直。')
        print('2. 垂下时：手掌朝右/朝身体内侧，拇指指向前方。')
        print('3. 整条直臂向前抬到肩高：手掌仍朝右，拇指转为朝上。')
        print('4. 再回到垂下，慢速往返；不转掌，不向侧面抬手。')
        print('=' * 72)
        input('Press ENTER, then begin the 12-second DOWN <-> FORWARD motion...')
        samples = self._collect_samples(SWEEP_SECONDS)

        p_u = np.array([sample.upper_position for sample in samples])
        R_u_reported = np.array([sample.upper_rotation for sample in samples])
        p_f = np.array([sample.forearm_position for sample in samples])
        R_f_reported = np.array([sample.forearm_rotation for sample in samples])
        path = self._save_diagnostic(
            p_u, R_u_reported, p_f, R_f_reported)

        receiver_distances = np.linalg.norm(p_f - p_u, axis=1)
        position_motion_range = self._direction_range_deg(p_f - p_u)
        distance_centre = float(np.median(receiver_distances))
        distance_errors = receiver_distances - distance_centre
        distance_rms = float(np.sqrt(np.mean(np.square(distance_errors))))
        distance_max = float(np.max(np.abs(distance_errors)))
        upper_steps = self._orientation_steps_deg(R_u_reported)
        forearm_steps = self._orientation_steps_deg(R_f_reported)
        upper_jumps = int(np.count_nonzero(upper_steps > 45.0))
        forearm_jumps = int(np.count_nonzero(forearm_steps > 45.0))

        print('\n  Raw stream checks:')
        print(f'    receiver distance: median={distance_centre*100:.1f} cm, '
              f'RMS change={distance_rms*1000:.1f} mm, '
              f'max change={distance_max*1000:.1f} mm')
        print(f'    position-implied arm sweep: {position_motion_range:.1f} deg')
        print(f'    orientation discontinuities (>45 deg/frame): '
              f'upper={upper_jumps}, forearm={forearm_jumps}')
        print(f'    largest one-frame rotation: upper={np.max(upper_steps):.1f} deg, '
              f'forearm={np.max(forearm_steps):.1f} deg')

        results = {}
        print('\n  Rigid-pair consistency results:')
        for convention in ROTATION_CONVENTIONS:
            R_u = apply_rotation_convention(R_u_reported, convention)
            R_f = apply_rotation_convention(R_f_reported, convention)
            pair = rigid_pair_consistency(p_u, R_u, p_f, R_f)
            upper_range = self._motion_range_deg(R_u)
            forearm_range = self._motion_range_deg(R_f)
            score = (
                pair.translation_rms_m / 0.015 +
                pair.rotation_rms_deg / 5.0)
            results[convention] = (
                score, pair, upper_range, forearm_range)
            print(f'\n    {convention}:')
            print(f'      motion range: upper={upper_range:.1f} deg, '
                  f'forearm={forearm_range:.1f} deg')
            print(f'      receiver-pair translation: RMS={pair.translation_rms_m*1000:.1f} mm, '
                  f'max={pair.translation_max_m*1000:.1f} mm')
            print(f'      receiver-pair orientation: RMS={pair.rotation_rms_deg:.1f} deg, '
                  f'max={pair.rotation_max_deg:.1f} deg')

        selected = min(results, key=lambda name: results[name][0])
        (_, pair, upper_range, forearm_range) = results[selected]
        print(f'\n  Selected convention for this diagnostic: {selected}')
        self._print_conclusion(
            pair.translation_rms_m, pair.rotation_rms_deg,
            position_motion_range, upper_range, forearm_range, distance_rms,
            upper_jumps, forearm_jumps)
        print(f'  Raw diagnostic saved: {path}')
        print('[Rigidity check] Test complete. This did not read or write the calibration YAML.')
        print('[Rigidity check] Press Ctrl+C to exit.')

    def _collect_samples(self, duration_s):
        samples = []
        last_stamp = None
        deadline = time.monotonic() + duration_s
        print(f'  Collecting {duration_s:.0f} s ', end='', flush=True)
        while rclpy.ok() and time.monotonic() < deadline:
            with self._pair_lock:
                sample = self._latest_pair
            if sample is not None and sample.stamp_ns != last_stamp:
                samples.append(sample)
                last_stamp = sample.stamp_ns
                if len(samples) % 50 == 0:
                    print('.', end='', flush=True)
            time.sleep(0.005)
        print(f' done ({len(samples)} pairs)')
        if len(samples) < max(20, int(duration_s * 15.0)):
            raise RuntimeError(
                f'Only {len(samples)} synchronized pairs collected. '
                'Check dual_em_tracker_node warnings.')
        return samples

    @staticmethod
    def _motion_range_deg(rotations):
        rotations = np.asarray(rotations)
        indices = np.unique(np.linspace(
            0, len(rotations) - 1, min(60, len(rotations)), dtype=int))
        selected = rotations[indices]
        return max(
            rotation_error_deg(first, second)
            for first_index, first in enumerate(selected[:-1])
            for second in selected[first_index + 1:])

    @staticmethod
    def _orientation_steps_deg(rotations):
        return np.asarray([
            rotation_error_deg(first, second)
            for first, second in zip(rotations[:-1], rotations[1:])
        ])

    @staticmethod
    def _direction_range_deg(vectors):
        vectors = np.asarray(vectors, dtype=float)
        norms = np.linalg.norm(vectors, axis=1)
        if np.any(norms < 1e-9):
            raise ValueError('receiver separation is zero')
        directions = vectors / norms[:, None]
        indices = np.unique(np.linspace(
            0, len(directions) - 1, min(100, len(directions)), dtype=int))
        selected = directions[indices]
        minimum_dot = float(np.min(np.clip(
            selected @ selected.T, -1.0, 1.0)))
        return float(np.degrees(np.arccos(minimum_dot)))

    @staticmethod
    def _print_conclusion(pair_translation_rms, pair_rotation_rms,
                          position_motion_range, upper_orientation_range,
                          forearm_orientation_range, distance_rms,
                          upper_jumps, forearm_jumps):
        if upper_jumps or forearm_jumps:
            print('[RESULT] FAIL: the receiver orientation stream contains discontinuities.')
            print(f'Detected >45 deg one-frame jumps: upper={upper_jumps}, '
                  f'forearm={forearm_jumps}. The slow arm motion cannot cause these jumps.')
            if distance_rms <= 0.005:
                print('Receiver positions remained stable, so the fault is isolated to orientation.')
            print('Do not calibrate or command a robot from this stream yet.')
            return

        if position_motion_range < 60.0:
            print('[RESULT] INCONCLUSIVE: arm motion was too small (<60 deg).')
            print('Repeat the slow DOWN <-> FORWARD motion through about 90 degrees.')
            return

        minimum_expected_orientation = 0.65 * position_motion_range
        if (upper_orientation_range < minimum_expected_orientation or
                forearm_orientation_range < minimum_expected_orientation):
            print('[RESULT] FAIL: reported orientation did not follow the measured arm motion.')
            print(f'Position indicates {position_motion_range:.1f} deg of arm motion, but '
                  f'orientation reports upper={upper_orientation_range:.1f} deg and '
                  f'forearm={forearm_orientation_range:.1f} deg.')
            print('The action amplitude was sufficient; inspect raw quaternion output and parsing.')
            return

        good = (
            distance_rms <= 0.005 and
            pair_translation_rms <= 0.015 and
            pair_rotation_rms <= 5.0)
        marginal = (
            distance_rms <= 0.010 and
            pair_translation_rms <= 0.030 and
            pair_rotation_rms <= 10.0)
        if good:
            print('[RESULT] PASS: both receivers remained consistent during the arm motion.')
            print('The calibration STEP 2/axis model is the next item to correct.')
        elif marginal:
            print('[RESULT] MARGINAL: the receiver chain moved, but errors are not catastrophic.')
            print('Tighten both arm attachments and repeat this check once.')
        else:
            print('[RESULT] FAIL: the receiver relationship changed during the arm motion.')
            print('Check arm attachment, unintended palm rotation, nearby metal, and raw pose parsing.')

    @staticmethod
    def _save_diagnostic(p_u, R_u, p_f, R_f):
        os.makedirs(CALIBRATION_LOG_DIR, exist_ok=True)
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        path = os.path.join(
            CALIBRATION_LOG_DIR, f'dual_rigidity_{timestamp}.npz')
        np.savez_compressed(
            path,
            upper_positions_m=p_u,
            upper_rotations_reported=R_u,
            forearm_positions_m=p_f,
            forearm_rotations_reported=R_f,
        )
        return path


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DualReceiverRigidityCheck()
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
