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
    ROTATION_AS_REPORTED,
    ROTATION_CONVENTIONS,
    ROTATION_TRANSPOSE_REPORTED,
    apply_rotation_convention,
    estimate_local_rotation_axis,
    fit_position_direction_calibration,
    fit_single_direction_with_known_axis,
    mean_rotation,
    relative_rotation_errors,
    rotation_error_deg,
    select_rotation_convention,
)
from remote_control.retargeting_kinematics import (
    FOREARM_ZERO_AXIS,
    UPPER_ARM_ZERO_AXIS,
    arm_frames,
    quaternion_to_matrix,
)


DUAL_CALIBRATION_FILE = (
    '/wb_humanoid_mpc_ws/src/wb_humanoid_mpc/'
    'humanoid_nmpc/remote_control/config/wmet_dual_calibration.yaml')
CALIBRATION_LOG_DIR = '/wb_humanoid_mpc_ws/calibration_logs'

STATIC_SECONDS = 4.0
REFERENCE_SECONDS = 2.0
REFERENCE_REPEATS = 3
ASSIGNMENT_CHECK_SECONDS = 6.0
TWIST_AXIS_SECONDS = 8.0
REFERENCE_REPEAT_LIMIT_DEG = 15.0
UPPER_REFERENCE_HARD_LIMIT_DEG = 15.0
FOREARM_REFERENCE_HARD_LIMIT_DEG = 25.0
TWIST_REFERENCE_HARD_LIMIT_DEG = 15.0
HARD_PIVOT_RMS_LIMIT_M = 0.04
MIN_UPPER_ARM_LENGTH_M = 0.15
MAX_UPPER_ARM_LENGTH_M = 0.45
MIN_FOREARM_RECEIVER_DISTANCE_M = 0.08
MAX_FOREARM_RECEIVER_DISTANCE_M = 0.40


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
    mode = calibration.get('mode')
    if mode not in ('dual_receiver_functional', 'dual_receiver_orientation',
                    'dual_receiver_direction', 'dual_receiver_hybrid'):
        raise ValueError(f'{path} is not a dual-receiver calibration file')
    quality = calibration.get('quality', {})
    failures = []
    reference_error = float(quality.get('reference_max_error_deg', float('inf')))
    isolated_axis_hybrid = (
        mode == 'dual_receiver_hybrid' and
        calibration.get('forearm_axis_method') == 'isolated_pronation_supination')
    if isolated_axis_hybrid:
        upper_errors = quality.get('upper_position_direction_errors_deg', [])
        forearm_errors = quality.get('forearm_direction_errors_deg', [])
        twist_errors = quality.get('forearm_twist_repeatability_errors_deg', [])
        upper_max = max(map(float, upper_errors), default=reference_error)
        forearm_max = max(map(float, forearm_errors), default=reference_error)
        twist_max = max(map(float, twist_errors), default=reference_error)
        if not math.isfinite(upper_max) or upper_max > UPPER_REFERENCE_HARD_LIMIT_DEG:
            failures.append('upper-arm reference error exceeds 15 degrees')
        if (not math.isfinite(forearm_max) or
                forearm_max > FOREARM_REFERENCE_HARD_LIMIT_DEG):
            failures.append('forearm reference error exceeds 25 degrees')
        if not math.isfinite(twist_max) or twist_max > TWIST_REFERENCE_HARD_LIMIT_DEG:
            failures.append('forearm twist reference error exceeds 15 degrees')
    elif not math.isfinite(reference_error) or reference_error > 15.0:
        failures.append('reference-pose error exceeds 15 degrees')
    if mode == 'dual_receiver_functional':
        shoulder_rms = float(quality.get('shoulder_pivot_rms_m', float('inf')))
        elbow_rms = float(quality.get('elbow_pivot_rms_m', float('inf')))
        if not math.isfinite(shoulder_rms) or shoulder_rms > HARD_PIVOT_RMS_LIMIT_M:
            failures.append('shoulder pivot RMS exceeds 40 mm')
        if not math.isfinite(elbow_rms) or elbow_rms > HARD_PIVOT_RMS_LIMIT_M:
            failures.append('elbow pivot RMS exceeds 40 mm')
        upper_length = float(calibration.get('upper_arm_length_m', float('nan')))
        if not MIN_UPPER_ARM_LENGTH_M <= upper_length <= MAX_UPPER_ARM_LENGTH_M:
            failures.append('estimated upper-arm length is implausible')
        forearm_distance = float(
            calibration.get('forearm_effective_length_m', float('nan')))
        if not (MIN_FOREARM_RECEIVER_DISTANCE_M <= forearm_distance <=
                MAX_FOREARM_RECEIVER_DISTANCE_M):
            failures.append('elbow-to-forearm-receiver distance is implausible')
    if failures:
        raise ValueError(
            f'Dual calibration at {path} failed safety/quality checks: '
            + '; '.join(failures) + '. Run dual_calibration_node again.')
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
            try:
                load_dual_calibration(DUAL_CALIBRATION_FILE)
            except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
                print(f'[Dual calibration] Existing calibration rejected: {exc}')
                print('[Dual calibration] A new calibration will be collected.')
            else:
                print('[Dual calibration] Recalibrate after removing, charging, or '
                      'reattaching any device.')
                answer = input(
                    'Reuse existing calibration only if no device moved? [y/N]: '
                ).strip().lower()
                if answer == 'y':
                    print('[Dual calibration] Existing calibration kept.')
                    return
                print('[Dual calibration] Collecting a new calibration.')

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
        print('STEP 1: Receiver assignment check')
        print(f'For {ASSIGNMENT_CHECK_SECONDS:.0f} seconds, keep the torso and LEFT upper arm still.')
        print('Repeatedly bend/straighten only the elbow and turn the palm up/down.')
        print('The upper-arm receiver should remain nearly still while the forearm moves.')
        print('=' * 68)
        input('Press ENTER, then begin the elbow-only motion...')
        assignment_samples = self._collect_samples(ASSIGNMENT_CHECK_SECONDS)
        _, assignment_R_u, _, assignment_R_f = self._arrays(assignment_samples)
        upper_excursion = self._rotation_excursion_deg(assignment_R_u)
        forearm_excursion = self._rotation_excursion_deg(assignment_R_f)
        print('  Receiver orientation excursion:')
        print(f'    upper topic:   {upper_excursion:5.1f} deg')
        print(f'    forearm topic: {forearm_excursion:5.1f} deg')
        if (upper_excursion > 15.0 or forearm_excursion < 25.0 or
                forearm_excursion < upper_excursion + 15.0):
            print('[ERROR] Receiver assignment check failed.')
            print('Confirm 10.42.0.143 is attached to the upper arm, keep the shoulder')
            print('still, and move only the elbow/forearm through a wide range.')
            print('[Dual calibration] Stopped without saving.')
            return

        # --------------------------------------------------------------
        twist_attempt_limit = 3
        for twist_attempt in range(1, twist_attempt_limit + 1):
            print('\n' + '=' * 68)
            print('STEP 2: Forearm rotation-axis calibration')
            print(f'For {TWIST_AXIS_SECONDS:.0f} seconds, keep the LEFT upper arm vertical,')
            print('the elbow bent 90 degrees and the forearm pointing forward.')
            print('Keep the shoulder, elbow and wrist position fixed. Start with the')
            print('thumb up, then slowly rotate about 45 degrees palm-down and about')
            print('45 degrees palm-up. Return to thumb-up and repeat twice.')
            print('Do not force the full rotation range, bend the wrist, or move the elbow.')
            print('=' * 68)
            input('Press ENTER, then begin the forearm-only rotation...')
            twist_samples = self._collect_samples(TWIST_AXIS_SECONDS)
            _, twist_R_u_reported, _, twist_R_f_reported = self._arrays(
                twist_samples)
            twist_upper_excursion = self._rotation_excursion_deg(
                twist_R_u_reported)
            twist_forearm_excursion = self._rotation_excursion_deg(
                twist_R_f_reported)
            print('  Isolated rotation excursion:')
            print(f'    upper topic:   {twist_upper_excursion:5.1f} deg')
            print(f'    forearm topic: {twist_forearm_excursion:5.1f} deg')
            try:
                twist_axis_preview = estimate_local_rotation_axis(
                    twist_R_f_reported)
            except ValueError as exc:
                print(f'    axis fit: {exc}')
                twist_axis_preview = None
            if twist_axis_preview is not None:
                print(f'    axis fit: RMS={twist_axis_preview.rms_error_deg:.2f} deg, '
                      f'max={twist_axis_preview.max_error_deg:.2f} deg, '
                      f'range={twist_axis_preview.motion_range_deg:.1f} deg')
            if (twist_upper_excursion <= 15.0 and
                    twist_axis_preview is not None and
                    twist_axis_preview.motion_range_deg >= 55.0 and
                    twist_axis_preview.rms_error_deg <= 15.0):
                if twist_axis_preview.rms_error_deg > 8.0:
                    print('[WARNING] Twist-axis motion is usable but marginal; '
                          'keep the elbow steadier when practical.')
                break
            if twist_attempt == twist_attempt_limit:
                print('[ERROR] Forearm rotation axis was not stable after three attempts.')
                print('[Dual calibration] Stopped without saving. Do not repeat the')
                print('motion again with this attachment. Refix the forearm receiver')
                print('5-8 cm above the wrist, with its local Y axis parallel to the')
                print('elbow-to-wrist direction, then restart calibration.')
                return
            if (twist_axis_preview is not None and
                    twist_axis_preview.motion_range_deg > 140.0):
                print('[RETRY] The motion was too large. Do not turn to the anatomical')
                print('end stops. Use only about 45 degrees to each side of thumb-up.')
            elif (twist_axis_preview is None or
                  twist_axis_preview.motion_range_deg < 55.0):
                print('[RETRY] Rotate slightly farther, but keep the total motion')
                print('controlled and keep the elbow and wrist positions fixed.')
            else:
                print('[RETRY] Use a smaller, slower rotation about the forearm itself;')
                print('keep the thumb, hand and receiver moving together as one unit.')

        # Three poses span independent upper-arm directions.  A fourth,
        # STRAIGHT_DOWN, overdetermines the forearm-direction fit and checks it
        # near full elbow extension.
        reference_specs = [
            ('down',
             'LEFT upper arm vertical down; elbow bent 90 degrees;\n'
             'forearm horizontal and pointing straight forward; palm faces\n'
             'right/inward and thumb points up.',
             (0.0, 0.0, 0.0, 0.0)),
            ('forward',
             'LEFT upper arm horizontal and pointing forward; elbow bent 90 degrees;\n'
             'forearm straight up without twisting it; palm faces right/inward;\n'
             'thumb points backward toward your body.',
             (-math.pi / 2.0, 0.0, 0.0, 0.0)),
            ('sideways',
             'LEFT arm straight and horizontal to the left; straighten the elbow;\n'
             'palm faces down and thumb points forward.',
             (0.0, math.pi / 2.0, 0.0, math.pi / 2.0)),
            ('straight_down',
             'LEFT arm relaxed and straight down beside the body; fully extend\n'
             'the elbow; palm faces right/inward and thumb points forward.',
             (0.0, 0.0, 0.0, math.pi / 2.0)),
        ]
        reference_labels = tuple(spec[0] for spec in reference_specs)
        upper_reference_positions = []
        forearm_reference_positions = []
        upper_reference_rotations = []
        forearm_reference_rotations = []
        target_upper_rotations = []
        target_forearm_rotations = []
        labels = []
        for label, instruction, target_angles in reference_specs:
            target_upper, target_forearm = arm_frames(*target_angles)
            print('\n' + '=' * 68)
            print(f'REFERENCE: {label.upper()} ({REFERENCE_REPEATS} repeats)')
            print(instruction)
            print('Between repeats, leave the pose, rebuild it carefully, then press ENTER.')
            print('=' * 68)
            while True:
                pose_upper_positions = []
                pose_forearm_positions = []
                pose_upper_rotations = []
                pose_forearm_rotations = []
                for repeat in range(REFERENCE_REPEATS):
                    input(
                        f'Press ENTER for repeat {repeat+1}/{REFERENCE_REPEATS}, '
                        'then hold still...')
                    held = self._collect_samples(REFERENCE_SECONDS)
                    held_p_u, held_R_u, held_p_f, held_R_f = self._arrays(held)
                    pose_upper_positions.append(np.mean(held_p_u, axis=0))
                    pose_forearm_positions.append(np.mean(held_p_f, axis=0))
                    pose_upper_rotations.append(mean_rotation(held_R_u))
                    pose_forearm_rotations.append(mean_rotation(held_R_f))

                if self._print_pose_repeatability(
                        np.asarray(pose_upper_rotations),
                        np.asarray(pose_forearm_rotations), label):
                    upper_reference_positions.extend(pose_upper_positions)
                    forearm_reference_positions.extend(pose_forearm_positions)
                    upper_reference_rotations.extend(pose_upper_rotations)
                    forearm_reference_rotations.extend(pose_forearm_rotations)
                    target_upper_rotations.extend(
                        [target_upper] * REFERENCE_REPEATS)
                    target_forearm_rotations.extend(
                        [target_forearm] * REFERENCE_REPEATS)
                    labels.extend([label] * REFERENCE_REPEATS)
                    break

                print(f'\n[RETRY] Only the {label.upper()} reference will be recollected;')
                print('the earlier accepted reference poses are retained. Rebuild all')
                print('three repeats with the same arm, palm and thumb directions.')

        target_upper_directions = np.array([
            rotation @ UPPER_ARM_ZERO_AXIS
            for rotation in target_upper_rotations
        ])
        target_forearm_directions = np.array([
            rotation @ FOREARM_ZERO_AXIS
            for rotation in target_forearm_rotations
        ])
        reported_upper_rotations = np.array(upper_reference_rotations)
        reported_forearm_rotations = np.array(forearm_reference_rotations)
        diagnostic_path = self._save_reference_diagnostic(
            np.array(upper_reference_positions),
            np.array(forearm_reference_positions),
            reported_upper_rotations,
            reported_forearm_rotations,
            np.array(target_upper_rotations),
            np.array(target_forearm_rotations),
            labels,
            twist_R_u_reported,
            twist_R_f_reported)
        print(f'  Reference diagnostic saved: {diagnostic_path}')

        position_fit = fit_position_direction_calibration(
            np.array(upper_reference_positions),
            target_upper_directions)

        convention_fits = {}
        for convention in ROTATION_CONVENTIONS:
            corrected_forearm = apply_rotation_convention(
                reported_forearm_rotations, convention)
            corrected_twist = apply_rotation_convention(
                twist_R_f_reported, convention)
            axis_result = estimate_local_rotation_axis(corrected_twist)
            fit = fit_single_direction_with_known_axis(
                corrected_forearm, target_forearm_directions,
                axis_result.axis_local)
            relative_errors = relative_rotation_errors(
                corrected_forearm,
                np.asarray(target_forearm_rotations),
                position_fit.rotation_tx_to_body)
            convention_fits[convention] = (
                float(np.sqrt(np.mean(np.square(fit.errors_deg)))),
                float(np.max(fit.errors_deg)),
                float(np.sqrt(np.mean(np.square(relative_errors)))),
                float(np.max(relative_errors)),
                fit,
                corrected_forearm,
                axis_result,
            )
        print('\n  Forearm quaternion convention diagnostic:')
        for convention in ROTATION_CONVENTIONS:
            (direction_rms, direction_max, relative_rms, relative_max,
             _, _, axis_result) = (
                convention_fits[convention])
            print(f'    {convention:20s}: direction RMS={direction_rms:5.2f}, '
                  f'max={direction_max:5.2f} deg; relative-motion '
                  f'RMS={relative_rms:5.2f}, max={relative_max:5.2f} deg;\n'
                  f'      twist-axis RMS={axis_result.rms_error_deg:4.2f}, '
                  f'max={axis_result.max_error_deg:4.2f} deg, '
                  f'range={axis_result.motion_range_deg:5.1f} deg')
        rotation_convention = select_rotation_convention(
            {name: convention_fits[name][0] for name in ROTATION_CONVENTIONS},
            {name: convention_fits[name][2] for name in ROTATION_CONVENTIONS})
        (_, _, _, _, forearm_fit, corrected_forearm_rotations,
         twist_axis_result) = (
            convention_fits[rotation_convention])
        print(f'  Selected rotation convention from direction residual '
              f'(relative motion used as tie-break): '
              f'{rotation_convention}')

        upper_position_align = position_fit.rotation_tx_to_body
        forearm_orientation_align = forearm_fit.rotation_tx_to_body
        forearm_axis_local = forearm_fit.axis_local
        upper_reference_errors = position_fit.direction_errors_deg
        forearm_reference_errors = forearm_fit.errors_deg

        _, wrist_zero_target = arm_frames(0.0, 0.0, 0.0, 0.0)
        wrist_zero_forearm_rotations = [
            rotation for rotation, label in zip(corrected_forearm_rotations, labels)
            if label == 'down'
        ]
        forearm_mount = mean_rotation(np.array([
            wrist_zero_target.T @ forearm_orientation_align @ rotation
            for rotation in wrist_zero_forearm_rotations
        ]))
        forearm_twist_reference_errors = np.array([
            rotation_error_deg(
                forearm_orientation_align @ rotation @ forearm_mount.T,
                wrist_zero_target)
            for rotation in wrist_zero_forearm_rotations
        ])

        position_rms_m = float(np.sqrt(np.mean(
            np.square(position_fit.position_errors_m))))
        position_max_m = float(np.max(position_fit.position_errors_m))
        print('\n  Hybrid position/orientation residuals:')
        for label in reference_labels:
            values = [
                error for name, error in zip(labels, upper_reference_errors)
                if name == label
            ]
            print(f'    upper position {label:8s}: mean={np.mean(values):5.2f} deg, '
                  f'max={np.max(values):5.2f} deg')
        for label in reference_labels:
            values = [
                error for name, error in zip(labels, forearm_reference_errors)
                if name == label
            ]
            print(f'    forearm {label:6s}: mean={np.mean(values):5.2f} deg, '
                  f'max={np.max(values):5.2f} deg')
        print(f'    forearm twist repeatability: '
              f'mean={np.mean(forearm_twist_reference_errors):5.2f} deg, '
              f'max={np.max(forearm_twist_reference_errors):5.2f} deg')
        print(f'    upper position model: radius='
              f'{position_fit.receiver_radius_m*100:.1f} cm, '
              f'RMS={position_rms_m*1000:.1f} mm, '
              f'max={position_max_m*1000:.1f} mm')
        upper_reference_max = float(np.max(upper_reference_errors))
        forearm_reference_max = float(np.max(forearm_reference_errors))
        forearm_twist_reference_max = float(np.max(
            forearm_twist_reference_errors))
        max_reference_error = float(max(
            upper_reference_max, forearm_reference_max,
            forearm_twist_reference_max))
        if (twist_axis_result.motion_range_deg < 55.0 or
                twist_axis_result.rms_error_deg > 15.0):
            print('[ERROR] Forearm rotation-axis calibration is unreliable.')
            print('[Dual calibration] Stopped without saving. Repeat STEP 2')
            print('with the elbow fixed and a wider, slower palm rotation.')
            return
        if not 0.08 <= position_fit.receiver_radius_m <= 0.35:
            print('[ERROR] Upper receiver position radius is implausible.')
            print('[Dual calibration] Stopped without saving.')
            return
        position_quality_warning = (
            position_rms_m > 0.03 or position_max_m > 0.05)
        if position_rms_m > 0.035 or position_max_m > 0.065:
            print('[ERROR] Upper receiver position model error is too large.')
            print('[Dual calibration] Stopped without saving. Keep the torso and')
            print('transmitter fixed, then repeat the reference poses.')
            return
        if position_quality_warning:
            print('[WARNING] Upper receiver position model is marginal '
                  '(recommended RMS <= 30 mm and max <= 50 mm).')
        if upper_reference_max > UPPER_REFERENCE_HARD_LIMIT_DEG:
            print('[ERROR] Upper-arm reference error exceeds 15 degrees.')
            print('[Dual calibration] Stopped without saving. Repeat the reference poses.')
            return
        if forearm_reference_max > FOREARM_REFERENCE_HARD_LIMIT_DEG:
            print('[ERROR] Forearm reference error exceeds 25 degrees.')
            print('[Dual calibration] Stopped without saving. Check the forearm')
            print('attachment and repeat the reference poses.')
            return
        if forearm_twist_reference_max > TWIST_REFERENCE_HARD_LIMIT_DEG:
            print('[ERROR] Forearm twist reference error exceeds 15 degrees.')
            print('[Dual calibration] Stopped without saving. Repeat the DOWN pose')
            print('with the same palm and thumb direction.')
            return
        reference_quality_warning = max_reference_error > 10.0
        dry_run_only = forearm_reference_max > 15.0
        if reference_quality_warning:
            print('[WARNING] Reference-pose error exceeds 10 degrees.')
            if forearm_reference_max > 15.0:
                print('[WARNING] The isolated anatomical rotation axis differs from')
                print('the idealized forearm direction. This calibration will be')
                print('saved as dry-run-only and cannot publish commands to hardware.')
        if position_quality_warning or reference_quality_warning:
            if input('Save calibration with these warnings? [y/N]: ').strip().lower() != 'y':
                print('[Dual calibration] Cancelled. Repeat the reference poses.')
                return

        calibration = {
            'version': 9,
            'mode': 'dual_receiver_hybrid',
            'dry_run_only': dry_run_only,
            'calibrated_at': datetime.now(timezone.utc).isoformat(),
            'reference_pose_labels': list(reference_labels),
            'rotation_convention': rotation_convention,
            'rotation_convention_method': 'direction_residual_then_relative_motion',
            'forearm_axis_method': 'isolated_pronation_supination',
            'forearm_axis_local': _vector_list(forearm_axis_local),
            'upper_position_rotation_tx_to_body': _matrix_list(
                upper_position_align),
            'upper_shoulder_center_tx_m': _vector_list(
                position_fit.shoulder_center_tx_m),
            'upper_receiver_radius_m': round(
                position_fit.receiver_radius_m, 8),
            'forearm_rotation_tx_to_body': _matrix_list(
                forearm_orientation_align),
            'forearm_sensor_mount_rotation': _matrix_list(forearm_mount),
            'quality': {
                **{key: round(float(value), 8) for key, value in static_quality.items()},
                'upper_assignment_excursion_deg': round(upper_excursion, 4),
                'forearm_assignment_excursion_deg': round(forearm_excursion, 4),
                'twist_upper_excursion_deg': round(
                    twist_upper_excursion, 4),
                'twist_forearm_excursion_deg': round(
                    twist_forearm_excursion, 4),
                'twist_axis_rms_error_deg': round(
                    twist_axis_result.rms_error_deg, 4),
                'twist_axis_max_error_deg': round(
                    twist_axis_result.max_error_deg, 4),
                'twist_axis_motion_range_deg': round(
                    twist_axis_result.motion_range_deg, 4),
                'as_reported_relative_rotation_rms_deg': round(
                    convention_fits[ROTATION_AS_REPORTED][2], 4),
                'transpose_relative_rotation_rms_deg': round(
                    convention_fits[ROTATION_TRANSPOSE_REPORTED][2], 4),
                'reference_max_error_deg': round(max_reference_error, 4),
                'upper_reference_max_error_deg': round(
                    upper_reference_max, 4),
                'forearm_reference_max_error_deg': round(
                    forearm_reference_max, 4),
                'forearm_twist_max_error_deg': round(
                    forearm_twist_reference_max, 4),
                'upper_position_direction_errors_deg': [
                    round(float(value), 4) for value in upper_reference_errors],
                'forearm_direction_errors_deg': [
                    round(float(value), 4) for value in forearm_reference_errors],
                'upper_position_model_rms_m': round(position_rms_m, 8),
                'upper_position_model_max_m': round(position_max_m, 8),
                'forearm_twist_repeatability_errors_deg': [
                    round(float(value), 4)
                    for value in forearm_twist_reference_errors],
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
    def _arrays(samples, rotation_convention=ROTATION_AS_REPORTED):
        upper_rotations = np.array([s.upper_rotation for s in samples])
        forearm_rotations = np.array([s.forearm_rotation for s in samples])
        return (
            np.array([s.upper_position for s in samples]),
            apply_rotation_convention(upper_rotations, rotation_convention),
            np.array([s.forearm_position for s in samples]),
            apply_rotation_convention(forearm_rotations, rotation_convention),
        )

    @staticmethod
    def _rotation_rms_deg(rotations):
        centre = mean_rotation(rotations)
        angles = []
        for rotation in rotations:
            cosine = (float(np.trace(centre.T @ rotation)) - 1.0) / 2.0
            angles.append(math.acos(float(np.clip(cosine, -1.0, 1.0))))
        return math.degrees(float(np.sqrt(np.mean(np.square(angles)))))

    @staticmethod
    def _rotation_excursion_deg(rotations):
        centre = mean_rotation(rotations)
        errors = [rotation_error_deg(rotation, centre) for rotation in rotations]
        return float(np.percentile(errors, 95.0))

    @staticmethod
    def _save_reference_diagnostic(upper_positions, forearm_positions,
                                   upper_rotations, forearm_rotations,
                                   target_upper_rotations,
                                   target_forearm_rotations, labels,
                                   twist_upper_rotations,
                                   twist_forearm_rotations):
        os.makedirs(CALIBRATION_LOG_DIR, exist_ok=True)
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        path = os.path.join(
            CALIBRATION_LOG_DIR,
            f'dual_reference_{timestamp}.npz')
        np.savez_compressed(
            path,
            upper_positions_m=np.asarray(upper_positions),
            forearm_positions_m=np.asarray(forearm_positions),
            upper_rotations_reported=np.asarray(upper_rotations),
            forearm_rotations_reported=np.asarray(forearm_rotations),
            target_upper_rotations=np.asarray(target_upper_rotations),
            target_forearm_rotations=np.asarray(target_forearm_rotations),
            labels=np.asarray(labels),
            twist_upper_rotations_reported=np.asarray(
                twist_upper_rotations),
            twist_forearm_rotations_reported=np.asarray(
                twist_forearm_rotations),
        )
        return path

    @staticmethod
    def _rotation_distance_deg(first, second):
        return rotation_error_deg(first, second)

    @classmethod
    def _rotation_medoid(cls, rotations):
        rotations = np.asarray(rotations)
        distances = np.array([
            [cls._rotation_distance_deg(first, second) for second in rotations]
            for first in rotations
        ])
        index = int(np.argmin(np.sum(distances, axis=1)))
        return rotations[index], distances[index], index

    @classmethod
    def _print_reference_repeatability(cls, upper_rotations,
                                       forearm_rotations, labels):
        print('\n  Reference repeatability check:')
        passed = True
        labels_array = np.asarray(labels)
        for label in dict.fromkeys(labels):
            selected_upper = np.asarray(upper_rotations)[labels_array == label]
            selected_forearm = np.asarray(forearm_rotations)[labels_array == label]
            passed = cls._print_pose_repeatability(
                selected_upper, selected_forearm, label) and passed
        return passed

    @classmethod
    def _print_pose_repeatability(cls, upper_rotations,
                                  forearm_rotations, label):
        print(f'\n  {label.upper()} repeatability check:')
        passed = True
        for receiver, rotations in (
                ('upper', upper_rotations), ('forearm', forearm_rotations)):
            _, deviations, medoid_index = cls._rotation_medoid(rotations)
            maximum = float(np.max(deviations))
            print(f'    {receiver:7s}: max={maximum:6.2f} deg '
                  f'(representative repeat {medoid_index + 1})')
            if maximum > REFERENCE_REPEAT_LIMIT_DEG:
                failed_repeat = int(np.argmax(deviations)) + 1
                print(f'      ERROR: repeat {failed_repeat} differs by '
                      f'{maximum:.1f} deg; rebuild this pose with the same '
                      'palm and thumb directions.')
                passed = False
        return passed

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
