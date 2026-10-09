"""Geometry for dual-receiver EM arm calibration and retargeting.

The upper-arm and forearm receivers are treated as two rigid bodies measured
in the transmitter frame.  Functional calibration identifies the shoulder and
elbow joint centres without manual anthropometric measurements.  Retargeting
uses orientations only, so the human link lengths are diagnostics rather than
inputs to the robot inverse kinematics.
"""

from dataclasses import dataclass
import math
from typing import Tuple

import numpy as np

from remote_control.retargeting_kinematics import (
    FOREARM_ZERO_AXIS,
    UPPER_ARM_ZERO_AXIS,
    arm_frames,
    shoulder_pitch_roll,
    unwrap_near,
    wrist_roll,
)


@dataclass(frozen=True)
class PivotResult:
    """Result and quality metrics from a functional pivot calibration."""

    first_local_offset: np.ndarray
    second_local_offset_or_center: np.ndarray
    rms_m: float
    condition_number: float
    rank: int
    inlier_count: int
    sample_count: int


def _validate_pose_arrays(positions: np.ndarray, rotations: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    positions = np.asarray(positions, dtype=float)
    rotations = np.asarray(rotations, dtype=float)
    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError('positions must have shape (N, 3)')
    if rotations.shape != (len(positions), 3, 3):
        raise ValueError('rotations must have shape (N, 3, 3)')
    if len(positions) < 6:
        raise ValueError('at least 6 pose samples are required')
    if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(rotations)):
        raise ValueError('pose arrays contain non-finite values')
    return positions, rotations


def _robust_block_lstsq(A: np.ndarray, b: np.ndarray, sample_count: int):
    """Solve a 3-row-per-sample system and reject isolated gross outliers."""
    solution, _, rank, singular_values = np.linalg.lstsq(A, b, rcond=None)
    residual_norms = np.linalg.norm((A @ solution - b).reshape(sample_count, 3), axis=1)

    median = float(np.median(residual_norms))
    mad = float(np.median(np.abs(residual_norms - median)))
    robust_sigma = 1.4826 * mad
    threshold = max(0.005, median + 4.0 * robust_sigma)
    inliers = residual_norms <= threshold

    # A second solve is useful for packet glitches, but never discard so much
    # data that the geometry becomes governed by a small subset.
    if np.count_nonzero(inliers) >= max(6, int(0.6 * sample_count)) and not np.all(inliers):
        row_mask = np.repeat(inliers, 3)
        solution, _, rank, singular_values = np.linalg.lstsq(A[row_mask], b[row_mask], rcond=None)
    else:
        inliers = np.ones(sample_count, dtype=bool)

    final_residuals = (A @ solution - b).reshape(sample_count, 3)
    rms = float(np.sqrt(np.mean(np.sum(final_residuals[inliers] ** 2, axis=1))))
    if len(singular_values) == 0 or singular_values[-1] <= 1e-12:
        condition = float('inf')
    else:
        condition = float(singular_values[0] / singular_values[-1])
    return solution, rms, condition, int(rank), inliers


def solve_shoulder_pivot(upper_positions: np.ndarray,
                         upper_rotations: np.ndarray) -> PivotResult:
    """Estimate upper-sensor-to-shoulder offset and shoulder centre.

    For every sample ``p_U + R_U s_U = S``.  ``s_U`` is expressed in the
    upper receiver frame and ``S`` in the transmitter frame.
    """
    p, R = _validate_pose_arrays(upper_positions, upper_rotations)
    sample_count = len(p)
    A = np.empty((3 * sample_count, 6), dtype=float)
    b = np.empty(3 * sample_count, dtype=float)
    for i in range(sample_count):
        A[3*i:3*i+3, :3] = R[i]
        A[3*i:3*i+3, 3:] = -np.eye(3)
        b[3*i:3*i+3] = -p[i]

    solution, rms, condition, rank, inliers = _robust_block_lstsq(A, b, sample_count)
    return PivotResult(
        first_local_offset=solution[:3],
        second_local_offset_or_center=solution[3:],
        rms_m=rms,
        condition_number=condition,
        rank=rank,
        inlier_count=int(np.count_nonzero(inliers)),
        sample_count=sample_count,
    )


def solve_elbow_pivot(upper_positions: np.ndarray,
                      upper_rotations: np.ndarray,
                      forearm_positions: np.ndarray,
                      forearm_rotations: np.ndarray) -> PivotResult:
    """Estimate the elbow centre in both receiver-local frames.

    Each paired sample obeys ``p_U + R_U e_U = p_F + R_F e_F``.
    Motions must include elbow flexion and forearm pronation/supination so the
    six offset components are observable.
    """
    p_u, R_u = _validate_pose_arrays(upper_positions, upper_rotations)
    p_f, R_f = _validate_pose_arrays(forearm_positions, forearm_rotations)
    if len(p_u) != len(p_f):
        raise ValueError('upper-arm and forearm arrays must contain the same number of samples')

    sample_count = len(p_u)
    A = np.empty((3 * sample_count, 6), dtype=float)
    b = np.empty(3 * sample_count, dtype=float)
    for i in range(sample_count):
        A[3*i:3*i+3, :3] = R_u[i]
        A[3*i:3*i+3, 3:] = -R_f[i]
        b[3*i:3*i+3] = p_f[i] - p_u[i]

    solution, rms, condition, rank, inliers = _robust_block_lstsq(A, b, sample_count)
    return PivotResult(
        first_local_offset=solution[:3],
        second_local_offset_or_center=solution[3:],
        rms_m=rms,
        condition_number=condition,
        rank=rank,
        inlier_count=int(np.count_nonzero(inliers)),
        sample_count=sample_count,
    )


def mean_rotation(rotations: np.ndarray) -> np.ndarray:
    """Return the closest proper rotation to the arithmetic matrix mean."""
    rotations = np.asarray(rotations, dtype=float)
    if rotations.ndim != 3 or rotations.shape[1:] != (3, 3) or len(rotations) == 0:
        raise ValueError('rotations must have shape (N, 3, 3) with N > 0')
    U, _, Vt = np.linalg.svd(np.mean(rotations, axis=0))
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(U @ Vt)
    return U @ correction @ Vt


def fit_body_alignment(measured_directions: np.ndarray,
                       body_directions: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Solve Wahba's problem for transmitter-to-body rotation.

    Rows of ``measured_directions`` and ``body_directions`` are corresponding
    unit vectors.  The returned rotation maps transmitter-frame vectors to the
    robot/body convention (+X forward, +Y left, +Z up).
    """
    measured = np.asarray(measured_directions, dtype=float)
    target = np.asarray(body_directions, dtype=float)
    if measured.shape != target.shape or measured.ndim != 2 or measured.shape[1] != 3:
        raise ValueError('direction arrays must both have shape (N, 3)')
    if len(measured) < 2:
        raise ValueError('at least two reference directions are required')
    measured = measured / np.linalg.norm(measured, axis=1, keepdims=True)
    target = target / np.linalg.norm(target, axis=1, keepdims=True)

    cross_covariance = target.T @ measured
    U, _, Vt = np.linalg.svd(cross_covariance)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(U @ Vt)
    rotation = U @ correction @ Vt

    aligned = (rotation @ measured.T).T
    dots = np.sum(aligned * target, axis=1)
    errors_deg = np.degrees(np.arccos(np.clip(dots, -1.0, 1.0)))
    return rotation, errors_deg


def extract_shoulder_angles(shoulder_rotation: np.ndarray,
                            yaw_reference: float = 0.0) -> Tuple[float, float, float]:
    """Decompose ``Ry(pitch) Rx(roll) Rz(yaw)`` using the upper-arm direction."""
    shoulder_rotation = np.asarray(shoulder_rotation, dtype=float)
    upper_direction = shoulder_rotation @ UPPER_ARM_ZERO_AXIS
    pitch, roll = shoulder_pitch_roll(upper_direction)

    # Removing pitch and roll leaves the axial shoulder rotation Rz(yaw).
    from remote_control.retargeting_kinematics import rotation_x, rotation_y
    residual = (rotation_y(pitch) @ rotation_x(roll)).T @ shoulder_rotation
    yaw = math.atan2(float(residual[1, 0]), float(residual[0, 0]))
    return pitch, roll, unwrap_near(yaw, yaw_reference)


def sensor_mount_from_reference(reference_sensor_aligned: np.ndarray,
                                reference_segment_rotation: np.ndarray) -> np.ndarray:
    """Fixed sensor mounting rotation in ``R_sensor = R_segment R_mount``."""
    return np.asarray(reference_segment_rotation).T @ np.asarray(reference_sensor_aligned)


def dual_joint_angles(upper_sensor_rotation_tx: np.ndarray,
                      forearm_sensor_rotation_tx: np.ndarray,
                      rotation_tx_to_body: np.ndarray,
                      upper_sensor_mount: np.ndarray,
                      forearm_sensor_mount: np.ndarray,
                      previous_yaw: float = 0.0,
                      previous_wrist: float = 0.0) -> np.ndarray:
    """Map a synchronized receiver pair to the five G1 left-arm angles."""
    R_align = np.asarray(rotation_tx_to_body, dtype=float)
    R_u_aligned = R_align @ np.asarray(upper_sensor_rotation_tx, dtype=float)
    R_f_aligned = R_align @ np.asarray(forearm_sensor_rotation_tx, dtype=float)

    shoulder_rotation = R_u_aligned @ np.asarray(upper_sensor_mount, dtype=float).T
    pitch, roll, yaw = extract_shoulder_angles(shoulder_rotation, previous_yaw)

    forearm_observed = R_f_aligned @ np.asarray(forearm_sensor_mount, dtype=float).T
    upper_direction = shoulder_rotation @ UPPER_ARM_ZERO_AXIS
    forearm_direction = forearm_observed @ FOREARM_ZERO_AXIS
    elbow = math.asin(float(np.clip(np.dot(upper_direction, forearm_direction), -1.0, 1.0)))

    wrist = wrist_roll(
        R_f_aligned,
        forearm_sensor_mount,
        pitch,
        roll,
        yaw,
        elbow,
        fallback=previous_wrist,
    )
    return np.array([pitch, roll, yaw, elbow, wrist], dtype=float)


def reference_mounts(rotation_tx_to_body: np.ndarray,
                     upper_reference_tx: np.ndarray,
                     forearm_reference_tx: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Compute both mounts from arm-forward, elbow-90, forearm-up pose."""
    reference_angles = (-math.pi / 2.0, 0.0, 0.0, 0.0)
    shoulder_reference, forearm_reference = arm_frames(*reference_angles)
    R_align = np.asarray(rotation_tx_to_body, dtype=float)
    upper_mount = sensor_mount_from_reference(
        R_align @ np.asarray(upper_reference_tx), shoulder_reference)
    forearm_mount = sensor_mount_from_reference(
        R_align @ np.asarray(forearm_reference_tx), forearm_reference)
    return upper_mount, forearm_mount
