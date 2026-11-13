#!/usr/bin/env python3
"""Analyse the V02 two-phone/EM comparison with a held-out split.

The V02 session has no separate anatomical-alignment video.  Consequently,
this script estimates the two fixed marker axes and the camera-to-body
rotation from one third of 10 s temporal blocks and reports accuracy only on
the other two thirds.  The reported result is therefore a held-out
cross-modal consistency result, not a fully independent absolute anatomical
accuracy measurement.

The marker-pose cache is produced from the two videos with 58 mm ArUco
markers.  Times and rotations in that cache are expressed in the individual
camera clocks/frames.  The constants below come from the intrinsic and stereo
calibration performed immediately before the V02 recording.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation


R_B_FROM_A = np.array([
    [0.70317224, 0.08087269, -0.70640527],
    [-0.04602375, 0.99660386, 0.06828291],
    [0.70952845, -0.01550323, 0.70450623],
])

# t_B = PHONE_SCALE * t_A + PHONE_OFFSET_S
PHONE_SCALE = 0.9999
PHONE_OFFSET_S = 17.48

# t_A = EM_SCALE * t_EM + EM_OFFSET_S
EM_SCALE = 1.00075
EM_OFFSET_S = 35.90

MARKER_NOMINAL_DISTAL_AXIS = np.array([0.0, -1.0, 0.0])


def unit(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    return values / np.linalg.norm(values, axis=-1, keepdims=True)


def angular_error_deg(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    dots = np.sum(unit(first) * unit(second), axis=-1)
    return np.degrees(np.arccos(np.clip(dots, -1.0, 1.0)))


def closest(series_time, series_rotation, series_error, target, tolerance=0.13):
    index = int(np.searchsorted(series_time, target))
    candidates = []
    for candidate in (index - 1, index):
        if 0 <= candidate < len(series_time):
            delta = abs(float(series_time[candidate] - target))
            if delta < tolerance:
                candidates.append((
                    delta,
                    series_rotation[candidate],
                    float(series_error[candidate]),
                ))
    return min(candidates, key=lambda item: item[0]) if candidates else None


def mean_rotation(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    rotations = Rotation.from_matrix(np.stack((first, second)))
    quaternions = rotations.as_quat()
    if np.dot(quaternions[0], quaternions[1]) < 0.0:
        quaternions[1] *= -1.0
    return Rotation.from_quat(unit(np.sum(quaternions, axis=0))).as_matrix()


def build_fused_visual_series(marker_cache: Path):
    data = np.load(marker_cache)
    camera = data['camera']
    times = data['time_s']
    ids = data['marker_id']
    rotations = data['rotation_camera_marker']
    errors = data['reprojection_px']

    series = {}
    for camera_name in ('A', 'B'):
        for marker_id in (10, 20):
            mask = (camera == camera_name) & (ids == marker_id)
            marker_times = times[mask]
            marker_rotations = rotations[mask]
            if camera_name == 'B':
                marker_rotations = np.einsum(
                    'ij,njk->nik', R_B_FROM_A.T, marker_rotations)
            order = np.argsort(marker_times)
            series[camera_name, marker_id] = (
                marker_times[order],
                marker_rotations[order],
                errors[mask][order],
            )

    grid = np.arange(0.0, 354.0, 0.2)
    fused = {}
    source_count = {}
    phone_disagreement = {}
    phone_direction_disagreement = {}
    for marker_id in (10, 20):
        output = []
        sources = []
        disagreement = []
        direction_disagreement = []
        for time_a in grid:
            estimate_a = closest(*series['A', marker_id], time_a)
            estimate_b = closest(
                *series['B', marker_id],
                PHONE_SCALE * time_a + PHONE_OFFSET_S,
            )
            if estimate_a is not None and estimate_b is not None:
                geodesic = math.degrees(np.linalg.norm(
                    Rotation.from_matrix(
                        estimate_a[1].T @ estimate_b[1]).as_rotvec()))
                disagreement.append(geodesic)
                direction_disagreement.append(float(angular_error_deg(
                    (estimate_a[1] @ MARKER_NOMINAL_DISTAL_AXIS)[None],
                    (estimate_b[1] @ MARKER_NOMINAL_DISTAL_AXIS)[None])[0]))
                if geodesic <= 30.0:
                    rotation = mean_rotation(estimate_a[1], estimate_b[1])
                    sources.append(2)
                else:
                    rotation = (estimate_a[1] if estimate_a[2] <= estimate_b[2]
                                else estimate_b[1])
                    sources.append(1)
            elif estimate_a is not None:
                rotation = estimate_a[1]
                sources.append(1)
            elif estimate_b is not None:
                rotation = estimate_b[1]
                sources.append(1)
            else:
                rotation = np.full((3, 3), np.nan)
                sources.append(0)
            output.append(rotation)
        output = np.asarray(output)
        # Reject an isolated planar-pose branch flip using visual continuity
        # alone.  A real arm motion cannot jump by >45 deg in one 0.2 s
        # sample and return on the next sample while its neighbours agree.
        valid = np.all(np.isfinite(output), axis=(1, 2))
        reject = np.zeros(len(output), dtype=bool)
        valid_indices = np.flatnonzero(valid)
        for index in valid_indices:
            before = valid_indices[valid_indices < index]
            after = valid_indices[valid_indices > index]
            if not len(before):
                continue
            previous = int(before[-1])
            if index - previous > 1:
                continue
            previous_to_current = math.degrees(np.linalg.norm(
                Rotation.from_matrix(
                    output[previous].T @ output[index]).as_rotvec()))
            # A lone last observation before an occlusion is rejected when it
            # makes an impossible one-frame jump.
            if not len(after) or int(after[0]) - index > 5:
                if previous_to_current > 45.0:
                    reject[index] = True
                continue
            following = int(after[0])
            current_to_next = math.degrees(np.linalg.norm(
                Rotation.from_matrix(
                    output[index].T @ output[following]).as_rotvec()))
            previous_to_next = math.degrees(np.linalg.norm(
                Rotation.from_matrix(
                    output[previous].T @ output[following]).as_rotvec()))
            if (previous_to_current > 45.0 and current_to_next > 45.0 and
                    previous_to_next < 20.0):
                reject[index] = True
        output[reject] = np.nan
        fused[marker_id] = output
        source_count[marker_id] = np.bincount(sources, minlength=3)
        phone_disagreement[marker_id] = np.asarray(disagreement)
        phone_direction_disagreement[marker_id] = np.asarray(
            direction_disagreement)
    return (grid, fused, source_count, phone_disagreement,
            phone_direction_disagreement)


def interpolate_unit_vectors(query_time, source_time, source_vectors):
    result = np.column_stack([
        np.interp(query_time, source_time, source_vectors[:, column])
        for column in range(3)
    ])
    return unit(result)


def fit_axes_and_frame(rotations_upper, rotations_forearm,
                       directions_upper, directions_forearm, train_mask):
    axis_upper = MARKER_NOMINAL_DISTAL_AXIS.copy()
    axis_forearm = MARKER_NOMINAL_DISTAL_AXIS.copy()
    camera_to_body = np.eye(3)
    weights_upper = np.ones(np.count_nonzero(train_mask))
    weights_forearm = np.ones(np.count_nonzero(train_mask))

    for iteration in range(30):
        visual_upper = rotations_upper[train_mask] @ axis_upper
        visual_forearm = rotations_forearm[train_mask] @ axis_forearm
        source = np.vstack((visual_upper, visual_forearm))
        target = np.vstack((directions_upper[train_mask],
                            directions_forearm[train_mask]))
        weights = np.concatenate((weights_upper, weights_forearm))
        camera_to_body = Rotation.align_vectors(
            target, source, weights=weights)[0].as_matrix()

        inverse = camera_to_body.T
        upper_terms = np.einsum(
            'nji,nj->ni', rotations_upper[train_mask],
            directions_upper[train_mask] @ inverse.T)
        forearm_terms = np.einsum(
            'nji,nj->ni', rotations_forearm[train_mask],
            directions_forearm[train_mask] @ inverse.T)
        axis_upper = unit(np.sum(weights_upper[:, None] * upper_terms, axis=0))
        axis_forearm = unit(np.sum(
            weights_forearm[:, None] * forearm_terms, axis=0))

        predicted_upper = (camera_to_body @
                           (rotations_upper[train_mask] @ axis_upper).T).T
        predicted_forearm = (camera_to_body @
                             (rotations_forearm[train_mask] @ axis_forearm).T).T
        upper_error = angular_error_deg(predicted_upper,
                                        directions_upper[train_mask])
        forearm_error = angular_error_deg(predicted_forearm,
                                          directions_forearm[train_mask])
        # Huber-style robust weighting prevents a short visual outlier from
        # rotating the fixed anatomical axis.
        weights_upper = np.minimum(1.0, 8.0 / np.maximum(upper_error, 1e-6))
        weights_forearm = np.minimum(1.0, 8.0 / np.maximum(
            forearm_error, 1e-6))

    return camera_to_body, axis_upper, axis_forearm


def statistics(values):
    values = np.asarray(values, dtype=float)
    return {
        'sample_count': int(len(values)),
        'mean': float(np.mean(values)),
        'rmse': float(np.sqrt(np.mean(np.square(values)))),
        'median': float(np.median(values)),
        'p95': float(np.percentile(values, 95)),
        'max': float(np.max(values)),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--marker-cache', type=Path,
        default=Path('/tmp/v02_marker_poses_5hz.npz'))
    parser.add_argument(
        '--em-log', type=Path,
        default=Path('/home/kunwang/humanoid_mpc_ws/calibration_logs/'
                     'visual_truth/V01/evaluation/'
                     'dual_retargeting_20261112T232317Z.npz'))
    parser.add_argument(
        '--output-dir', type=Path,
        default=Path('humanoid_nmpc/remote_control/docs/figures'))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    (visual_time, rotations, source_count, disagreement,
     direction_disagreement) = \
        build_fused_visual_series(args.marker_cache)
    valid_visual = (np.all(np.isfinite(rotations[10]), axis=(1, 2)) &
                    np.all(np.isfinite(rotations[20]), axis=(1, 2)))
    # Keep a 0.4 s margin on either side of an occlusion.  Pose estimates at
    # the very edge of a marker disappearance are particularly susceptible to
    # a planar-PnP branch switch and are not reliable truth samples.
    quality_visual = valid_visual.copy()
    for shift in (1, 2):
        quality_visual[shift:] &= valid_visual[:-shift]
        quality_visual[:-shift] &= valid_visual[shift:]
    quality_visual[:2] = False
    quality_visual[-2:] = False

    em = np.load(args.em_log)
    em_time = (em['stamp_ns'] - em['stamp_ns'][0]) * 1e-9
    paired_em_time = (visual_time - EM_OFFSET_S) / EM_SCALE
    in_em_range = ((paired_em_time >= em_time[0]) &
                   (paired_em_time <= em_time[-1]))
    paired = quality_visual & in_em_range
    paired_em_time = paired_em_time[paired]
    rotation_upper = rotations[10][paired]
    rotation_forearm = rotations[20][paired]
    em_upper = interpolate_unit_vectors(
        paired_em_time, em_time, em['upper_direction_body'])
    em_forearm = interpolate_unit_vectors(
        paired_em_time, em_time, em['forearm_direction_body'])
    em_elbow = np.degrees(np.interp(
        paired_em_time, em_time, em['corrected_elbow_angle_rad']))

    # Deterministic temporal split.  Every third 10 s block is used only to
    # estimate the fixed axes/frame; all reported errors use the held-out
    # blocks.  A 0.4 s margin avoids placing the same transition on both sides.
    block = np.floor(paired_em_time / 10.0).astype(int)
    within_block = paired_em_time - 10.0 * block
    train = ((block % 3) == 0) & (within_block >= 0.4) & \
            (within_block <= 9.6)
    test = ~train

    camera_to_body, axis_upper, axis_forearm = fit_axes_and_frame(
        rotation_upper, rotation_forearm, em_upper, em_forearm, train)
    visual_upper = (camera_to_body @
                    (rotation_upper @ axis_upper).T).T
    visual_forearm = (camera_to_body @
                      (rotation_forearm @ axis_forearm).T).T
    visual_elbow_raw = np.degrees(np.arcsin(np.clip(
        np.sum(visual_upper * visual_forearm, axis=1), -1.0, 1.0)))

    # A marker-derived elbow coordinate needs the same two-point anatomical
    # zero/gain calibration as the EM coordinate.  Estimate that affine map on
    # fit blocks only and apply it unchanged to held-out blocks.
    elbow_design = np.column_stack((
        visual_elbow_raw[train], np.ones(np.count_nonzero(train))))
    elbow_scale, elbow_offset = np.linalg.lstsq(
        elbow_design, em_elbow[train], rcond=None)[0]
    visual_elbow = np.clip(
        elbow_scale * visual_elbow_raw + elbow_offset, 0.0, 90.0)

    upper_error = angular_error_deg(visual_upper, em_upper)
    forearm_error = angular_error_deg(visual_forearm, em_forearm)
    elbow_error = np.abs(visual_elbow - em_elbow)

    report = {
        'method': ('10 s temporal blocks; 1/3 fit, 2/3 held-out evaluation; '
                   '0.4 s visual occlusion-edge margin'),
        'limitation': (
            'V02 had no separate anatomical-alignment video; this is '
            'held-out cross-modal consistency, not independent absolute '
            'anatomical accuracy.'),
        'marker_black_square_m': 0.058,
        'phone_time_map': {
            'equation': 't_B = scale * t_A + offset_s',
            'scale': PHONE_SCALE,
            'offset_s': PHONE_OFFSET_S,
        },
        'em_time_map': {
            'equation': 't_A = scale * t_EM + offset_s',
            'scale': EM_SCALE,
            'offset_s': EM_OFFSET_S,
        },
        'paired_samples_5hz': int(len(paired_em_time)),
        'fit_samples_5hz': int(np.count_nonzero(train)),
        'held_out_samples_5hz': int(np.count_nonzero(test)),
        'marker_axis_upper_local': axis_upper.tolist(),
        'marker_axis_forearm_local': axis_forearm.tolist(),
        'axis_upper_offset_from_nominal_deg': float(angular_error_deg(
            axis_upper[None], MARKER_NOMINAL_DISTAL_AXIS[None])[0]),
        'axis_forearm_offset_from_nominal_deg': float(angular_error_deg(
            axis_forearm[None], MARKER_NOMINAL_DISTAL_AXIS[None])[0]),
        'camera_A_to_body_rotation': camera_to_body.tolist(),
        'visual_elbow_fit_on_training_blocks': {
            'scale': float(elbow_scale),
            'offset_deg': float(elbow_offset),
            'limits_deg': [0.0, 90.0],
        },
        'phone_geodesic_disagreement_deg': {
            'id10_upper': statistics(disagreement[10]),
            'id20_forearm': statistics(disagreement[20]),
        },
        'phone_nominal_direction_disagreement_deg': {
            'id10_upper': statistics(direction_disagreement[10]),
            'id20_forearm': statistics(direction_disagreement[20]),
        },
        'held_out_error_deg': {
            'upper_direction': statistics(upper_error[test]),
            'forearm_direction': statistics(forearm_error[test]),
            'elbow_angle': statistics(elbow_error[test]),
        },
        'fit_error_deg_not_for_reporting': {
            'upper_direction': statistics(upper_error[train]),
            'forearm_direction': statistics(forearm_error[train]),
            'elbow_angle': statistics(elbow_error[train]),
        },
        'visual_coverage': {
            'grid_samples': int(len(visual_time)),
            'paired_segment_samples': int(np.count_nonzero(valid_visual)),
            'paired_fraction': float(np.mean(valid_visual)),
            'quality_samples_after_occlusion_margin': int(
                np.count_nonzero(quality_visual)),
            'id10_no_camera_single_both': source_count[10].tolist(),
            'id20_no_camera_single_both': source_count[20].tolist(),
        },
    }

    json_path = args.output_dir / 'visual_truth_v02_summary.json'
    json_path.write_text(json.dumps(report, indent=2), encoding='utf-8')

    csv_path = args.output_dir / 'visual_truth_v02_paired_samples.csv'
    with csv_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'em_time_s', 'split', 'em_elbow_deg',
            'visual_elbow_raw_deg', 'visual_elbow_calibrated_deg',
            'elbow_abs_error_deg', 'upper_direction_error_deg',
            'forearm_direction_error_deg'])
        for index in range(len(paired_em_time)):
            writer.writerow([
                f'{paired_em_time[index]:.6f}',
                'fit' if train[index] else 'held_out',
                f'{em_elbow[index]:.6f}',
                f'{visual_elbow_raw[index]:.6f}',
                f'{visual_elbow[index]:.6f}',
                f'{elbow_error[index]:.6f}',
                f'{upper_error[index]:.6f}',
                f'{forearm_error[index]:.6f}',
            ])

    fig, axes = plt.subplots(2, 1, figsize=(10.0, 6.2), sharex=True)
    axes[0].plot(paired_em_time, em_elbow, color='black', linewidth=1.4,
                 label='EM retargeting')
    axes[0].plot(paired_em_time, visual_elbow, color='#0072B2',
                 linewidth=1.0, alpha=0.9, label='Two-phone visual')
    axes[0].scatter(paired_em_time[train], visual_elbow[train], s=5,
                    color='#D55E00', alpha=0.35,
                    label='Alignment-fit blocks')
    axes[0].set_ylabel('Elbow coordinate (deg)')
    axes[0].legend(ncol=3, fontsize=8, loc='upper right')
    axes[0].grid(alpha=0.25)
    axes[1].plot(paired_em_time[test], elbow_error[test], '.', markersize=2.5,
                 color='#009E73', label='Elbow absolute error')
    axes[1].axhline(report['held_out_error_deg']['elbow_angle']['p95'],
                    color='#D55E00', linestyle='--', linewidth=1.0,
                    label='Held-out 95th percentile')
    axes[1].set_xlabel('EM elapsed time (s)')
    axes[1].set_ylabel('Absolute error (deg)')
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.25)
    fig.tight_layout()
    for suffix in ('pdf', 'png'):
        fig.savefig(args.output_dir / f'visual_truth_v02_elbow.{suffix}',
                    dpi=220, bbox_inches='tight')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10.0, 3.8))
    ax.plot(paired_em_time[test], upper_error[test], '.', markersize=2.4,
            label='Upper-arm direction', color='#0072B2')
    ax.plot(paired_em_time[test], forearm_error[test], '.', markersize=2.4,
            label='Forearm direction', color='#CC79A7')
    ax.set_xlabel('EM elapsed time (s)')
    ax.set_ylabel('Angular error (deg)')
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    for suffix in ('pdf', 'png'):
        fig.savefig(args.output_dir / f'visual_truth_v02_direction_error.{suffix}',
                    dpi=220, bbox_inches='tight')
    plt.close(fig)

    print(json.dumps(report, indent=2))
    print(f'Wrote {json_path}')
    print(f'Wrote {csv_path}')


if __name__ == '__main__':
    main()
