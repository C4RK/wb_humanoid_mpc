#!/usr/bin/env python3

"""Functional calibration for one transmitter and two arm receivers."""

from dataclasses import dataclass
from datetime import datetime, timezone
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import yaml

from remote_control.dual_retargeting_kinematics import (
    fit_body_alignment,
    mean_rotation,
    reference_mounts,
    solve_elbow_pivot,
    solve_shoulder_pivot,
)
from remote_control.retargeting_kinematics import quaternion_to_matrix


DUAL_CALIBRATION_FILE = (
    '/wb_humanoid_mpc_ws/src/wb_humanoid_mpc/'
    'humanoid_nmpc/remote_control/config/wmet_dual_calibration.yaml')

STATIC_SECONDS = 4.0
SHOULDER_SWEEP_SECONDS = 12.0
ELBOW_SWEEP_SECONDS = 15.0
REFERENCE_SECONDS = 2.0
REFERENCE_REPEATS = 3


@dataclass(frozen=True)
class PosePairSample:
    stamp_ns: int
    upper_position: np.ndarray
    upper_rotation: np.ndarray
    forearm_position: np.ndarray
    forearm_rotation: np.ndarray


def _stamp_ns(msg: PoseStamped) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def _message_pose(msg: PoseStamped):
    p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
    q = np.array([msg.pose.orientation.w, msg.pose.orientation.x,
                  msg.pose.orientation.y, msg.pose.orientation.z], dtype=float)
    return p, quaternion_to_matrix(q)


def _matrix_list(matrix: np.ndarray):
    return [[round(float(v), 8) for v in row] for row in np.asarray(matrix)]


def _vector_list(vector: np.ndarray):
    return [round(float(v), 8) for v in np.asarray(vector)]


def load_dual_calibration(path: str = DUAL_CALIBRATION_FILE) -> dict:
    if not os.path.exists(path):
        raise FileNotFoundError(
            f'Dual-receiver calibration file not found at {path}. '
            'Run dual_calibration_node first.')
    with open(path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    calibration = data.get('calibration', {}) if data else {}
    if calibration.get('mode') != 'dual_receiver_functional':
        raise ValueError(f'{path} is not a dual-receiver calibration file')
    return calibration


class DualCalibrationNode(Node):
    def __init__(self):
        super().__init__('dual_calibration_node')
        self._pair_lock = threading.Lock()
        self._latest_pair = None
        self._pending_upper = {}
        self._pending_forearm = {}

        self.create_subscription(
            PoseStamped, '/em/upper_arm_pose', self._on_upper_pose, 20)
        self.create_subscription(
            PoseStamped, '/em/forearm_pose', self._on_forearm_pose, 20)
        self.get_logger().info(
            'Dual calibration node started; waiting for synchronized receiver topics.')

        self._worker = threading.Thread(target=self._run_sequence_guarded, daemon=True)
        self._worker.start()

    def _run_sequence_guarded(self):
        try:
            self._run_sequence()
        except (EOFError, KeyboardInterrupt):
            self.get_logger().info('Dual calibration input cancelled.')
        except Exception as exc:
            self.get_logger().error(f'Dual calibration failed: {exc}')

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
            sample = PosePairSample(key, p_u, R_u, p_f, R_f)
            with self._pair_lock:
                self._latest_pair = sample
        self._pending_upper.pop(key, None)
        self._pending_forearm.pop(key, None)
        self._trim_pending()

    def _trim_pending(self):
        for pending in (self._pending_upper, self._pending_forearm):
            while len(pending) > 30:
                pending.pop(min(pending))

    def _run_sequence(self):
        if os.path.exists(DUAL_CALIBRATION_FILE):
            print(f'\n[Dual calibration] Existing file: {DUAL_CALIBRATION_FILE}')
            answer = input('Use existing dual-receiver calibration? [Y/n]: ').strip().lower()
            if answer != 'n':
                print('[Dual calibration] Existing calibration kept.')
                return

        print('\n[Dual calibration] Waiting for synchronized receiver pairs...')
        while rclpy.ok():
            with self._pair_lock:
                ready = self._latest_pair is not None
            if ready:
                break
            time.sleep(0.1)
        if not rclpy.ok():
            return
        print('[Dual calibration] Both receivers are available.')

        # --------------------------------------------------------------
        print('\n' + '=' * 68)
        print('STEP 0: Static quality check')
        print('Stand naturally and keep the torso and left arm completely still.')
        print('The tape and all three devices must remain fixed.')
        print('=' * 68)
        input('Press ENTER, then stay still...')
        static_samples = self._collect_samples(STATIC_SECONDS)
        static_quality = self._static_quality(static_samples)
        self._print_static_quality(static_quality)
        if (max(static_quality['upper_position_rms_m'],
                static_quality['forearm_position_rms_m']) > 0.003 or
                max(static_quality['upper_angle_rms_deg'],
                    static_quality['forearm_angle_rms_deg']) > 3.0):
            print('[WARNING] Static noise is high. Check tape, metal objects and tracking range.')
            if input('Continue anyway? [y/N]: ').strip().lower() != 'y':
                print('[Dual calibration] Cancelled at static check.')
                return

        # --------------------------------------------------------------
        print('\n' + '=' * 68)
        print('STEP 1: Shoulder functional centre')
        print(f'For {SHOULDER_SWEEP_SECONDS:.0f} seconds, keep the elbow at a comfortable fixed bend.')
        print('Move the upper arm slowly forward, sideways and diagonally.')
        print('Include small upper-arm twists. Keep the torso still.')
        print('=' * 68)
        input('Press ENTER, then begin the shoulder sweep...')
        shoulder_samples = self._collect_samples(SHOULDER_SWEEP_SECONDS)
        p_u, R_u, _, _ = self._arrays(shoulder_samples)
        shoulder_result = solve_shoulder_pivot(p_u, R_u)
        self._print_pivot('shoulder', shoulder_result)
        if shoulder_result.rank < 6:
            print('[ERROR] Shoulder calibration is rank deficient. Use more varied directions.')
            return
        if shoulder_result.rms_m > 0.02 or shoulder_result.condition_number > 200.0:
            print('[WARNING] Shoulder fit quality is poor; tape may have shifted or motion was limited.')
            if input('Save this fit and continue? [y/N]: ').strip().lower() != 'y':
                return

        # --------------------------------------------------------------
        print('\n' + '=' * 68)
        print('STEP 2: Elbow functional centre')
        print(f'For {ELBOW_SWEEP_SECONDS:.0f} seconds, keep the upper arm roughly in front of you.')
        print('Continuously bend and straighten the elbow through a wide range.')
        print('Also rotate the forearm palm-up/palm-down. Move slowly; keep the wrist rigid.')
        print('=' * 68)
        input('Press ENTER, then begin the elbow/forearm sweep...')
        elbow_samples = self._collect_samples(ELBOW_SWEEP_SECONDS)
        p_u, R_u, p_f, R_f = self._arrays(elbow_samples)
        elbow_result = solve_elbow_pivot(p_u, R_u, p_f, R_f)
        self._print_pivot('elbow', elbow_result)
        if elbow_result.rank < 6:
            print('[ERROR] Elbow calibration is rank deficient.')
            print('Repeat with both elbow flexion and palm-up/palm-down rotation.')
            return
        if elbow_result.rms_m > 0.02 or elbow_result.condition_number > 300.0:
            print('[WARNING] Elbow fit quality is poor; check both receiver attachments.')
            if input('Save this fit and continue? [y/N]: ').strip().lower() != 'y':
                return

        sensor_to_shoulder = shoulder_result.first_local_offset
        shoulder_center = shoulder_result.second_local_offset_or_center
        upper_sensor_to_elbow = elbow_result.first_local_offset
        forearm_sensor_to_elbow = elbow_result.second_local_offset_or_center
        upper_axis_local = upper_sensor_to_elbow - sensor_to_shoulder
        upper_length = float(np.linalg.norm(upper_axis_local))
        forearm_axis_local = -forearm_sensor_to_elbow
        forearm_effective_length = float(np.linalg.norm(forearm_axis_local))
        if upper_length < 0.10 or forearm_effective_length < 0.10:
            print('[ERROR] A recovered segment vector is too short; functional fit is invalid.')
            return
        upper_axis_local /= upper_length
        forearm_axis_local /= forearm_effective_length
        print(f'  Estimated upper-arm length:          {upper_length*100:.1f} cm')
        print(f'  Elbow-to-forearm-receiver distance:  {forearm_effective_length*100:.1f} cm')

        # --------------------------------------------------------------
        reference_specs = [
            ('down',
             'Let the LEFT upper arm hang straight down. Keep the torso upright.',
             np.array([0.0, 0.0, -1.0])),
            ('forward',
             'Upper arm straight forward at shoulder height; elbow 90 degrees;\n'
             'forearm straight up; LEFT palm faces right/inward; thumb points forward.',
             np.array([1.0, 0.0, 0.0])),
            ('sideways',
             'Upper arm straight out to the LEFT at shoulder height; torso upright.',
             np.array([0.0, 1.0, 0.0])),
        ]
        all_measured = []
        all_targets = []
        reference_samples = {}
        for label, instruction, target in reference_specs:
            samples_for_label = []
            print('\n' + '=' * 68)
            print(f'REFERENCE: {label.upper()} ({REFERENCE_REPEATS} repeats)')
            print(instruction)
            print('=' * 68)
            for repeat in range(REFERENCE_REPEATS):
                input(f'Press ENTER for repeat {repeat+1}/{REFERENCE_REPEATS}, then hold still...')
                held = self._collect_samples(REFERENCE_SECONDS)
                samples_for_label.extend(held)
                _, rotations, _, _ = self._arrays(held)
                direction = np.mean(
                    np.einsum('nij,j->ni', rotations, upper_axis_local), axis=0)
                direction /= np.linalg.norm(direction)
                all_measured.append(direction)
                all_targets.append(target)
            reference_samples[label] = samples_for_label

        R_align, reference_errors = fit_body_alignment(
            np.array(all_measured), np.array(all_targets))
        print('\n  Reference angular residuals:')
        labels = [spec[0] for spec in reference_specs for _ in range(REFERENCE_REPEATS)]
        for label in ('down', 'forward', 'sideways'):
            values = [error for name, error in zip(labels, reference_errors) if name == label]
            print(f'    {label:8s}: mean={np.mean(values):5.2f} deg, max={np.max(values):5.2f} deg')
        max_reference_error = float(np.max(reference_errors))
        if max_reference_error > 10.0:
            print('[WARNING] Reference poses disagree by more than 10 degrees.')
            if input('Save calibration anyway? [y/N]: ').strip().lower() != 'y':
                print('[Dual calibration] Cancelled. Repeat the three reference poses.')
                return

        forward = reference_samples['forward']
        _, forward_R_u, _, forward_R_f = self._arrays(forward)
        upper_mount, forearm_mount = reference_mounts(
            R_align, mean_rotation(forward_R_u), mean_rotation(forward_R_f))

        calibration = {
            'version': 2,
            'mode': 'dual_receiver_functional',
            'calibrated_at': datetime.now(timezone.utc).isoformat(),
            'shoulder_center_tx_m': _vector_list(shoulder_center),
            'upper_sensor_to_shoulder_m': _vector_list(sensor_to_shoulder),
            'upper_sensor_to_elbow_m': _vector_list(upper_sensor_to_elbow),
            'forearm_sensor_to_elbow_m': _vector_list(forearm_sensor_to_elbow),
            'upper_axis_local': _vector_list(upper_axis_local),
            'forearm_axis_local': _vector_list(forearm_axis_local),
            'upper_arm_length_m': round(upper_length, 6),
            'forearm_effective_length_m': round(forearm_effective_length, 6),
            'rotation_tx_to_body': _matrix_list(R_align),
            'upper_sensor_mount_rotation': _matrix_list(upper_mount),
            'forearm_sensor_mount_rotation': _matrix_list(forearm_mount),
            'quality': {
                **{key: round(float(value), 8) for key, value in static_quality.items()},
                'shoulder_pivot_rms_m': round(shoulder_result.rms_m, 8),
                'shoulder_condition_number': round(shoulder_result.condition_number, 3),
                'shoulder_inliers': shoulder_result.inlier_count,
                'shoulder_samples': shoulder_result.sample_count,
                'elbow_pivot_rms_m': round(elbow_result.rms_m, 8),
                'elbow_condition_number': round(elbow_result.condition_number, 3),
                'elbow_inliers': elbow_result.inlier_count,
                'elbow_samples': elbow_result.sample_count,
                'reference_max_error_deg': round(max_reference_error, 4),
                'reference_errors_deg': [round(float(v), 4) for v in reference_errors],
            },
        }
        self._save(calibration)
        print(f'\n[Dual calibration] Saved to {DUAL_CALIBRATION_FILE}')
        print('[Dual calibration] Start dual_retargeting_node next. Press Ctrl+C here.')

    def _collect_samples(self, duration_s: float):
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
                f'Only {len(samples)} synchronized pairs collected in {duration_s:.0f} s. '
                'Check receiver connectivity and pair-skew warnings.')
        return samples

    @staticmethod
    def _arrays(samples):
        return (
            np.array([s.upper_position for s in samples]),
            np.array([s.upper_rotation for s in samples]),
            np.array([s.forearm_position for s in samples]),
            np.array([s.forearm_rotation for s in samples]),
        )

    @staticmethod
    def _rotation_rms_deg(rotations):
        centre = mean_rotation(rotations)
        angles = []
        for rotation in rotations:
            cosine = (float(np.trace(centre.T @ rotation)) - 1.0) / 2.0
            angles.append(math.acos(float(np.clip(cosine, -1.0, 1.0))))
        return math.degrees(float(np.sqrt(np.mean(np.square(angles)))))

    def _static_quality(self, samples):
        p_u, R_u, p_f, R_f = self._arrays(samples)
        position_rms = lambda p: float(np.sqrt(np.mean(np.sum((p - np.mean(p, axis=0)) ** 2, axis=1))))
        return {
            'upper_position_rms_m': position_rms(p_u),
            'forearm_position_rms_m': position_rms(p_f),
            'upper_angle_rms_deg': self._rotation_rms_deg(R_u),
            'forearm_angle_rms_deg': self._rotation_rms_deg(R_f),
            'static_samples': len(samples),
        }

    @staticmethod
    def _print_static_quality(quality):
        print('  Static RMS:')
        print(f"    upper:   position={quality['upper_position_rms_m']*1000:.2f} mm, "
              f"angle={quality['upper_angle_rms_deg']:.2f} deg")
        print(f"    forearm: position={quality['forearm_position_rms_m']*1000:.2f} mm, "
              f"angle={quality['forearm_angle_rms_deg']:.2f} deg")

    @staticmethod
    def _print_pivot(label, result):
        print(f'  {label.capitalize()} pivot: RMS={result.rms_m*1000:.2f} mm, '
              f'condition={result.condition_number:.1f}, rank={result.rank}, '
              f'inliers={result.inlier_count}/{result.sample_count}')

    @staticmethod
    def _save(calibration):
        os.makedirs(os.path.dirname(DUAL_CALIBRATION_FILE), exist_ok=True)
        with open(DUAL_CALIBRATION_FILE, 'w', encoding='utf-8') as stream:
            yaml.safe_dump({'calibration': calibration}, stream,
                           sort_keys=False, allow_unicode=True)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DualCalibrationNode()
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
