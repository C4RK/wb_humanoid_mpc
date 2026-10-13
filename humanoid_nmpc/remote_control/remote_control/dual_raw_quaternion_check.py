#!/usr/bin/env python3

"""Inspect WEMT quaternion fields before ROS conversion or normalization."""

from datetime import datetime, timezone
import itertools
import os
import time

import numpy as np
from wemt_api import DeviceConfig, WemtAPI

from remote_control.dual_retargeting_kinematics import rigid_pair_consistency


UPPER_IP = '10.42.0.143'
FOREARM_IP = '10.42.0.144'
PORT = 801
COLLECTION_SECONDS = 12.0
MAX_PAIR_SKEW_SECONDS = 0.030
DEFAULT_LOG_DIR = '/wb_humanoid_mpc_ws/calibration_logs'


def quaternion_matrices(quaternions):
    """Convert an ``N x 4`` wxyz array to rotation matrices."""
    q = np.asarray(quaternions, dtype=float)
    if q.ndim != 2 or q.shape[1] != 4:
        raise ValueError('quaternions must have shape (N, 4)')
    norms = np.linalg.norm(q, axis=1)
    if np.any(~np.isfinite(norms)) or np.any(norms < 1e-12):
        raise ValueError('quaternion stream contains zero or non-finite values')
    w, x, y, z = (q / norms[:, None]).T
    rotations = np.empty((len(q), 3, 3), dtype=float)
    rotations[:, 0, 0] = 1.0 - 2.0 * (y*y + z*z)
    rotations[:, 0, 1] = 2.0 * (x*y - z*w)
    rotations[:, 0, 2] = 2.0 * (x*z + y*w)
    rotations[:, 1, 0] = 2.0 * (x*y + z*w)
    rotations[:, 1, 1] = 1.0 - 2.0 * (x*x + z*z)
    rotations[:, 1, 2] = 2.0 * (y*z - x*w)
    rotations[:, 2, 0] = 2.0 * (x*z - y*w)
    rotations[:, 2, 1] = 2.0 * (y*z + x*w)
    rotations[:, 2, 2] = 1.0 - 2.0 * (x*x + y*y)
    return rotations


def rotation_range_deg(rotations):
    rotations = np.asarray(rotations, dtype=float)
    indices = np.unique(np.linspace(
        0, len(rotations) - 1, min(100, len(rotations)), dtype=int))
    selected = rotations[indices]
    maximum = 0.0
    for index, first in enumerate(selected[:-1]):
        relative = np.einsum('ij,njk->nik', first.T, selected[index + 1:])
        cosines = np.clip(
            (np.trace(relative, axis1=1, axis2=2) - 1.0) / 2.0,
            -1.0, 1.0)
        maximum = max(maximum, float(np.max(np.degrees(np.arccos(cosines)))))
    return maximum


def direction_range_deg(vectors):
    vectors = np.asarray(vectors, dtype=float)
    directions = vectors / np.linalg.norm(vectors, axis=1, keepdims=True)
    indices = np.unique(np.linspace(
        0, len(directions) - 1, min(100, len(directions)), dtype=int))
    selected = directions[indices]
    minimum_dot = float(np.min(np.clip(selected @ selected.T, -1.0, 1.0)))
    return float(np.degrees(np.arccos(minimum_dot)))


def quaternion_step_statistics(quaternions):
    q = np.asarray(quaternions, dtype=float)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    dots = np.sum(q[:-1] * q[1:], axis=1)
    sign_flips = int(np.count_nonzero(dots < 0.0))
    angles = np.degrees(2.0 * np.arccos(np.clip(np.abs(dots), 0.0, 1.0)))
    return sign_flips, angles


def best_fixed_quaternion_layout(p_u, q_u, p_f, q_f):
    """Find whether one fixed component/sign convention repairs the pair."""
    best = None
    # Fix the first sign to +1 because negating all four components represents
    # the same rotation and would duplicate every candidate.
    for permutation in itertools.permutations(range(4)):
        for tail_signs in itertools.product((1.0, -1.0), repeat=3):
            signs = np.array((1.0,) + tail_signs)
            R_u = quaternion_matrices(q_u[:, permutation] * signs)
            R_f = quaternion_matrices(q_f[:, permutation] * signs)
            pair = rigid_pair_consistency(p_u, R_u, p_f, R_f)
            score = (
                pair.translation_rms_m / 0.015 +
                pair.rotation_rms_deg / 5.0)
            candidate = (score, permutation, tuple(int(v) for v in signs), pair)
            if best is None or candidate[0] < best[0]:
                best = candidate
    return best


def _wait_for_receivers(api):
    print('[Raw quaternion check] Waiting for both receivers...')
    deadline = time.monotonic() + 10.0
    next_status = time.monotonic()
    last_poses = None
    while time.monotonic() < deadline:
        upper = api.pose('upper_arm')
        forearm = api.pose('forearm')
        last_poses = (upper, forearm)
        if (upper.connected and forearm.connected and
                upper.timestamp is not None and forearm.timestamp is not None):
            print('[Raw quaternion check] Both receivers are available.')
            return
        if time.monotonic() >= next_status:
            def status(pose):
                if pose.error:
                    return f'error={pose.error}'
                if pose.connected and pose.timestamp is None:
                    return 'connected, waiting for first frame'
                return 'connected' if pose.connected else 'not connected'
            print(f'  upper_arm: {status(upper)}; forearm: {status(forearm)}')
            next_status += 1.0
        time.sleep(0.05)
    details = ''
    if last_poses is not None:
        details = (f' (upper error={last_poses[0].error!r}, '
                   f'forearm error={last_poses[1].error!r})')
    raise TimeoutError(
        'Both receivers were not available within 10 seconds. Stop '
        'em_tracker_node and dual_em_tracker_node before running this '
        f'direct-connection diagnostic{details}')


def _collect(api):
    samples = []
    last_stamps = None
    deadline = time.monotonic() + COLLECTION_SECONDS
    print(f'  Collecting {COLLECTION_SECONDS:.0f} s ', end='', flush=True)
    while time.monotonic() < deadline:
        upper = api.pose('upper_arm')
        forearm = api.pose('forearm')
        if (upper.updated_at is None or forearm.updated_at is None or
                upper.timestamp is None or forearm.timestamp is None):
            time.sleep(0.002)
            continue
        stamps = (upper.timestamp, forearm.timestamp)
        skew = abs((upper.updated_at - forearm.updated_at).total_seconds())
        if (stamps != last_stamps and
                (last_stamps is None or
                 (stamps[0] != last_stamps[0] and stamps[1] != last_stamps[1])) and
                skew <= MAX_PAIR_SKEW_SECONDS):
            samples.append((
                upper.timestamp,
                forearm.timestamp,
                upper.updated_at.timestamp(),
                forearm.updated_at.timestamp(),
                np.asarray(upper.position_mm, dtype=float) / 1000.0,
                np.asarray(forearm.position_mm, dtype=float) / 1000.0,
                np.asarray(upper.quaternion_wxyz, dtype=float),
                np.asarray(forearm.quaternion_wxyz, dtype=float),
            ))
            last_stamps = stamps
            if len(samples) % 50 == 0:
                print('.', end='', flush=True)
        time.sleep(0.002)
    print(f' done ({len(samples)} pairs)')
    if len(samples) < 180:
        raise RuntimeError(
            f'Only {len(samples)} synchronized pairs were collected; '
            'check receiver connectivity')
    return samples


def _timestamp_report(label, timestamps):
    timestamps = np.asarray(timestamps, dtype=np.int64)
    differences = np.diff(timestamps)
    positive = differences[differences > 0]
    non_increasing = int(np.count_nonzero(differences <= 0))
    if len(positive):
        print(f'    {label}: non-increasing={non_increasing}, '
              f'positive step median={np.median(positive):.1f}, '
              f'range=[{np.min(positive)}, {np.max(positive)}]')
    else:
        print(f'    {label}: no positive timestamp steps')


def _save_raw(arrays):
    log_dir = DEFAULT_LOG_DIR
    os.makedirs(log_dir, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    path = os.path.join(log_dir, f'dual_raw_quaternion_{timestamp}.npz')
    np.savez_compressed(path, **arrays)
    return path


def _run():
    devices = [
        DeviceConfig(id='upper_arm', host=UPPER_IP, port=PORT),
        DeviceConfig(id='forearm', host=FOREARM_IP, port=PORT),
    ]
    api = WemtAPI(devices=devices, smoothing_window=1)
    api.start_tracking()
    try:
        _wait_for_receivers(api)
        print('\n' + '=' * 72)
        print('RAW QUATERNION CHECK / 原始四元数检查')
        print('Keep the current attachments unchanged.')
        print('LEFT arm DOWN: palm right/inward, thumb forward.')
        print('Slowly raise the straight arm FORWARD to shoulder height, then return.')
        print('Repeat without bending the elbow or rotating the palm.')
        print('保持当前安装不变。左臂垂下时手掌朝右、拇指朝前；')
        print('整条直臂缓慢向前抬到肩高再放下，不弯肘、不转掌。')
        print('=' * 72)
        input('Press ENTER, then begin the 12-second motion...')
        samples = _collect(api)
    finally:
        api.stop_tracking()

    arrays = {
        'upper_timestamps': np.asarray([s[0] for s in samples], dtype=np.int64),
        'forearm_timestamps': np.asarray([s[1] for s in samples], dtype=np.int64),
        'upper_arrival_time_s': np.asarray([s[2] for s in samples]),
        'forearm_arrival_time_s': np.asarray([s[3] for s in samples]),
        'upper_positions_m': np.asarray([s[4] for s in samples]),
        'forearm_positions_m': np.asarray([s[5] for s in samples]),
        'upper_quaternions_raw_wxyz': np.asarray([s[6] for s in samples]),
        'forearm_quaternions_raw_wxyz': np.asarray([s[7] for s in samples]),
    }
    p_u = arrays['upper_positions_m']
    p_f = arrays['forearm_positions_m']
    q_u = arrays['upper_quaternions_raw_wxyz']
    q_f = arrays['forearm_quaternions_raw_wxyz']
    norm_u = np.linalg.norm(q_u, axis=1)
    norm_f = np.linalg.norm(q_f, axis=1)
    R_u = quaternion_matrices(q_u)
    R_f = quaternion_matrices(q_f)
    physical_range = direction_range_deg(p_f - p_u)
    upper_range = rotation_range_deg(R_u)
    forearm_range = rotation_range_deg(R_f)
    upper_flips, upper_steps = quaternion_step_statistics(q_u)
    forearm_flips, forearm_steps = quaternion_step_statistics(q_f)
    pair = rigid_pair_consistency(p_u, R_u, p_f, R_f)
    best = best_fixed_quaternion_layout(p_u, q_u, p_f, q_f)

    print('\n  Raw quaternion norms (a valid unit quaternion should stay near 1.0):')
    print(f'    upper:   min={np.min(norm_u):.6f}, median={np.median(norm_u):.6f}, '
          f'max={np.max(norm_u):.6f}')
    print(f'    forearm: min={np.min(norm_f):.6f}, median={np.median(norm_f):.6f}, '
          f'max={np.max(norm_f):.6f}')
    print('  Raw component ranges [w, x, y, z]:')
    print(f'    upper:   {np.round(np.ptp(q_u, axis=0), 6).tolist()}')
    print(f'    forearm: {np.round(np.ptp(q_f, axis=0), 6).tolist()}')
    print('  Hardware timestamp checks:')
    _timestamp_report('upper', arrays['upper_timestamps'])
    _timestamp_report('forearm', arrays['forearm_timestamps'])
    print('  Quaternion continuity:')
    print(f'    upper: sign flips={upper_flips}, largest physical step={np.max(upper_steps):.2f} deg')
    print(f'    forearm: sign flips={forearm_flips}, largest physical step={np.max(forearm_steps):.2f} deg')
    print('  Motion response:')
    print(f'    position-implied sweep={physical_range:.1f} deg')
    print(f'    reported orientation range: upper={upper_range:.1f} deg, '
          f'forearm={forearm_range:.1f} deg')
    print('  Current wxyz pair residual:')
    print(f'    translation RMS={pair.translation_rms_m*1000:.1f} mm, '
          f'orientation RMS={pair.rotation_rms_deg:.1f} deg')
    print('  Best fixed component/sign convention:')
    print(f'    output wxyz <- raw indices {best[1]}, signs={best[2]}')
    print(f'    translation RMS={best[3].translation_rms_m*1000:.1f} mm, '
          f'orientation RMS={best[3].rotation_rms_deg:.1f} deg')

    if np.any(np.abs(norm_u - 1.0) > 0.05) or np.any(np.abs(norm_f - 1.0) > 0.05):
        print('[RESULT] RAW QUATERNION INVALID: norm differs materially from 1.0.')
        print('Packet format or device quaternion output must be corrected.')
    elif physical_range >= 60.0 and min(upper_range, forearm_range) < 0.65 * physical_range:
        print('[RESULT] RAW ORIENTATION UNDER-RESPONDS TO THE MEASURED MOTION.')
        if best[3].translation_rms_m > 0.050:
            print('No fixed quaternion component/sign convention repairs the data.')
        print('The fault is present before ROS conversion and retargeting.')
    else:
        print('[RESULT] Raw quaternion response is not obviously invalid in this trial.')

    path = _save_raw(arrays)
    print(f'  Raw diagnostic saved: {path}')
    print('  This command did not read or write the calibration YAML.')


def main():
    try:
        _run()
    except KeyboardInterrupt:
        print('\n[Raw quaternion check] Cancelled.')


if __name__ == '__main__':
    main()
