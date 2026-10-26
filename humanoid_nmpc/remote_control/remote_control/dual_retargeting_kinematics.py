"""Geometry for dual-receiver EM arm calibration and retargeting.

The upper-arm and forearm receivers are treated as two rigid bodies measured
in the transmitter frame.  The active calibration uses known reference poses
to identify each segment direction without manual anthropometric measurements.
Legacy functional-centre helpers remain available for older calibration files.
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

import numpy as np

from remote_control.retargeting_kinematics import (
    FOREARM_ZERO_AXIS,
    UPPER_ARM_ZERO_AXIS,
    arm_frames,
    shoulder_pitch_roll,
    shoulder_yaw,
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


@dataclass(frozen=True)
class OrientationCalibrationResult:
    """Transmitter/body alignment and fixed receiver mounting rotation."""

    rotation_tx_to_body: np.ndarray
    sensor_mount_rotation: np.ndarray
    errors_deg: np.ndarray


@dataclass(frozen=True)
class DirectionCalibrationResult:
    """Body alignment and receiver-local longitudinal segment axes."""

    rotation_tx_to_body: np.ndarray
    upper_axis_local: np.ndarray
    forearm_axis_local: np.ndarray
    upper_errors_deg: np.ndarray
    forearm_errors_deg: np.ndarray


@dataclass(frozen=True)
class SingleDirectionCalibrationResult:
    """Alignment and one receiver-local segment axis."""

    rotation_tx_to_body: np.ndarray
    axis_local: np.ndarray
    errors_deg: np.ndarray


@dataclass(frozen=True)
class LocalRotationAxisResult:
    """Receiver-local hinge axis estimated from an isolated axial motion."""

    axis_local: np.ndarray
    rms_error_deg: float
    max_error_deg: float
    motion_range_deg: float
    pair_count: int


@dataclass(frozen=True)
class PositionDirectionCalibrationResult:
    """Upper-arm direction model obtained from receiver positions."""

    rotation_tx_to_body: np.ndarray
    shoulder_center_tx_m: np.ndarray
    receiver_radius_m: float
    direction_errors_deg: np.ndarray
    position_errors_m: np.ndarray


@dataclass(frozen=True)
class AffinePositionDirectionCalibrationResult:
    """Upper-arm direction model for a non-spherical receiver trajectory.

    The model is ``p_tx = centre_tx + direction_body @ body_to_tx_matrix``.
    A general invertible matrix retains the fixed-centre safety check while
    allowing the measured forward-raise path to be ellipsoidal instead of
    forcing every reference onto one sphere.
    """

    centre_tx_m: np.ndarray
    body_to_tx_matrix_m: np.ndarray
    direction_errors_deg: np.ndarray
    position_errors_m: np.ndarray
    equivalent_radius_m: float
    condition_number: float


@dataclass(frozen=True)
class RigidPairConsistencyResult:
    """Scatter of the transform between two receivers on one rigid chain."""

    translation_rms_m: float
    translation_max_m: float
    rotation_rms_deg: float
    rotation_max_deg: float


@dataclass(frozen=True)
class ElbowAngleCalibrationResult:
    """Affine correction for the direction-derived elbow angle."""

    scale: float
    offset_rad: float
    raw_angles_rad: np.ndarray
    corrected_angles_rad: np.ndarray
    target_angles_rad: np.ndarray
    raw_errors_deg: np.ndarray
    corrected_errors_deg: np.ndarray


ROTATION_AS_REPORTED = 'as_reported'
ROTATION_TRANSPOSE_REPORTED = 'transpose_reported'
ROTATION_CONVENTIONS = (
    ROTATION_AS_REPORTED,
    ROTATION_TRANSPOSE_REPORTED,
)


def apply_elbow_angle_calibration(angle_rad: float,
                                  scale: float,
                                  offset_rad: float,
                                  minimum_rad: float = 0.0,
                                  maximum_rad: float = math.pi / 2.0) -> float:
    """Apply the per-installation elbow gain/zero and anatomical limits."""
    if not all(math.isfinite(value) for value in (
            angle_rad, scale, offset_rad, minimum_rad, maximum_rad)):
        raise ValueError('elbow calibration contains a non-finite value')
    if scale <= 0.0 or minimum_rad >= maximum_rad:
        raise ValueError('invalid elbow calibration scale or limits')
    return float(np.clip(
        scale * float(angle_rad) + offset_rad,
        minimum_rad,
        maximum_rad))


def fit_elbow_angle_calibration(measured_angles_rad: np.ndarray,
                                target_angles_rad: np.ndarray,
                                minimum_rad: float = 0.0,
                                maximum_rad: float = math.pi / 2.0,
                                ) -> ElbowAngleCalibrationResult:
    """Fit elbow gain and zero from the existing static reference poses.

    Direction-fit residuals make a fully extended elbow appear several
    degrees short of 90 degrees.  A two-parameter least-squares fit uses both
    bent and straight references, avoiding the large bent-arm error caused by
    adding a fixed extension offset.  Limits are applied only when reporting
    the corrected residual and later during online retargeting.
    """
    measured = np.asarray(measured_angles_rad, dtype=float).reshape(-1)
    target = np.asarray(target_angles_rad, dtype=float).reshape(-1)
    if measured.shape != target.shape or measured.size < 4:
        raise ValueError('at least four paired elbow references are required')
    if not np.all(np.isfinite(measured)) or not np.all(np.isfinite(target)):
        raise ValueError('elbow reference angles contain non-finite values')
    if float(np.ptp(target)) < math.radians(45.0):
        raise ValueError('elbow references do not span bent and straight poses')

    design = np.column_stack((measured, np.ones(measured.size)))
    solution, _, rank, _ = np.linalg.lstsq(design, target, rcond=None)
    if rank < 2:
        raise ValueError('elbow gain and zero are not independently observable')
    scale, offset = map(float, solution)
    # These bounds catch a failed reference acquisition while allowing the
    # 1.14--1.32 gains seen across successful reattachments.
    if not 0.75 <= scale <= 1.50:
        raise ValueError(f'implausible elbow angle scale {scale:.3f}')
    if abs(offset) > math.radians(20.0):
        raise ValueError(
            f'implausible elbow angle offset {math.degrees(offset):.1f} deg')

    corrected = np.clip(scale * measured + offset, minimum_rad, maximum_rad)
    raw_errors = np.degrees(measured - target)
    corrected_errors = np.degrees(corrected - target)
    return ElbowAngleCalibrationResult(
        scale=scale,
        offset_rad=offset,
        raw_angles_rad=measured,
        corrected_angles_rad=corrected,
        target_angles_rad=target,
        raw_errors_deg=raw_errors,
        corrected_errors_deg=corrected_errors,
    )


def select_rotation_convention(direction_rms_by_convention,
                               relative_rms_by_convention) -> str:
    """Select the quaternion direction that best recovers the segment axis.

    Full relative-orientation residuals also contain human wrist twist and
    reference-pose reconstruction error.  They are therefore only a tie-break
    for the segment-direction residual used by retargeting.
    """
    for convention in ROTATION_CONVENTIONS:
        if convention not in direction_rms_by_convention:
            raise ValueError(f'missing direction score for {convention}')
        if convention not in relative_rms_by_convention:
            raise ValueError(f'missing relative-rotation score for {convention}')
        scores = (float(direction_rms_by_convention[convention]),
                  float(relative_rms_by_convention[convention]))
        if not all(math.isfinite(score) for score in scores):
            raise ValueError(f'non-finite rotation score for {convention}')
    return min(
        ROTATION_CONVENTIONS,
        key=lambda name: (float(direction_rms_by_convention[name]),
                          float(relative_rms_by_convention[name])))


def apply_rotation_convention(rotations: np.ndarray, convention: str) -> np.ndarray:
    """Convert reported receiver rotations to receiver-local-to-TX rotations.

    WEMT packet documentation specifies quaternion order but does not state
    whether the quaternion maps receiver coordinates to transmitter
    coordinates or the inverse.  A rigid pivot motion makes that ambiguity
    observable.  This helper keeps calibration and online retargeting on the
    convention selected from the multi-pose reference diagnostic.
    """
    rotations = np.asarray(rotations, dtype=float)
    if rotations.shape[-2:] != (3, 3):
        raise ValueError('rotations must end with shape (3, 3)')
    if convention == ROTATION_AS_REPORTED:
        return rotations
    if convention == ROTATION_TRANSPOSE_REPORTED:
        return np.swapaxes(rotations, -1, -2)
    raise ValueError(
        f'unknown rotation convention {convention!r}; expected one of '
        f'{ROTATION_CONVENTIONS}')


def relative_rotation_errors(rotations_tx: np.ndarray,
                             target_rotations_body: np.ndarray,
                             rotation_tx_to_body: np.ndarray) -> np.ndarray:
    """Score a quaternion direction using known relative reference motions.

    For ``T = A R M.T``, the unknown receiver mounting ``M`` cancels between
    two poses: ``T_j T_i.T = A R_j R_i.T A.T``.  This makes the reported versus
    transposed quaternion convention observable once ``A`` has independently
    been obtained from the upper-receiver position fit.
    """
    rotations = np.asarray(rotations_tx, dtype=float)
    targets = np.asarray(target_rotations_body, dtype=float)
    alignment = np.asarray(rotation_tx_to_body, dtype=float)
    if rotations.shape != targets.shape or rotations.ndim != 3 or rotations.shape[1:] != (3, 3):
        raise ValueError('receiver and target rotations must both have shape (N, 3, 3)')
    if alignment.shape != (3, 3):
        raise ValueError('rotation_tx_to_body must have shape (3, 3)')

    errors = []
    for first in range(len(rotations)):
        for second in range(first + 1, len(rotations)):
            target_relative = targets[second] @ targets[first].T
            target_angle_deg = rotation_error_deg(target_relative, np.eye(3))
            if target_angle_deg < 5.0:
                continue
            measured_relative = (
                alignment @ rotations[second] @ rotations[first].T @ alignment.T)
            errors.append(rotation_error_deg(measured_relative, target_relative))
    if not errors:
        raise ValueError('reference poses do not contain a distinct relative rotation')
    return np.asarray(errors)


def diagnose_rotation_convention(upper_positions: np.ndarray,
                                 reported_upper_rotations: np.ndarray):
    """Fit the shoulder pivot under both possible quaternion directions."""
    results = {
        convention: solve_shoulder_pivot(
            upper_positions,
            apply_rotation_convention(reported_upper_rotations, convention))
        for convention in ROTATION_CONVENTIONS
    }
    selected = min(results, key=lambda name: results[name].rms_m)
    return selected, results


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


def rigid_pair_consistency(upper_positions: np.ndarray,
                           upper_rotations: np.ndarray,
                           forearm_positions: np.ndarray,
                           forearm_rotations: np.ndarray,
                           ) -> RigidPairConsistencyResult:
    """Measure whether two receiver poses maintain one rigid transform.

    During a straight-arm sweep with a neutral wrist, the upper-arm and
    forearm receivers should behave as one rigid body.  Expressing the second
    receiver in the first receiver's frame removes torso and transmitter
    motion from this check.
    """
    p_u, R_u = _validate_pose_arrays(upper_positions, upper_rotations)
    p_f, R_f = _validate_pose_arrays(forearm_positions, forearm_rotations)
    if len(p_u) != len(p_f):
        raise ValueError('upper-arm and forearm arrays must contain the same number of samples')

    relative_positions = np.einsum(
        'nji,nj->ni', R_u, p_f - p_u)
    position_centre = np.median(relative_positions, axis=0)
    position_errors = np.linalg.norm(
        relative_positions - position_centre, axis=1)

    relative_rotations = np.einsum(
        'nji,njk->nik', R_u, R_f)
    rotation_centre = mean_rotation(relative_rotations)
    rotation_errors = np.array([
        rotation_error_deg(rotation, rotation_centre)
        for rotation in relative_rotations
    ])

    return RigidPairConsistencyResult(
        translation_rms_m=float(np.sqrt(np.mean(np.square(position_errors)))),
        translation_max_m=float(np.max(position_errors)),
        rotation_rms_deg=float(np.sqrt(np.mean(np.square(rotation_errors)))),
        rotation_max_deg=float(np.max(rotation_errors)),
    )


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


def rotation_vector(rotation: np.ndarray) -> np.ndarray:
    """Return the shortest SO(3) rotation vector for a rotation matrix."""
    rotation = np.asarray(rotation, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError('rotation must have shape (3, 3)')
    angle = math.acos(float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)))
    if angle < 1e-8:
        return np.zeros(3)
    if math.pi - angle < 1e-5:
        # The ordinary skew formula is singular at pi.  Extract a stable axis
        # from the symmetric part and recover signs from the off-diagonals.
        diagonal = np.maximum((np.diag(rotation) + 1.0) / 2.0, 0.0)
        axis = np.sqrt(diagonal)
        index = int(np.argmax(axis))
        if axis[index] < 1e-8:
            raise ValueError('cannot recover rotation axis')
        for other in range(3):
            if other != index:
                axis[other] = ((rotation[index, other] + rotation[other, index]) /
                               (4.0 * axis[index]))
        axis /= np.linalg.norm(axis)
        return angle * axis
    skew = np.array([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ])
    return angle * skew / (2.0 * math.sin(angle))


def estimate_local_rotation_axis(
        rotations: np.ndarray,
        min_pair_angle_deg: float = 20.0,
        max_pair_angle_deg: float = 170.0,
        max_samples: int = 80) -> LocalRotationAxisResult:
    """Estimate a fixed receiver-local axis from an isolated twist motion.

    During pronation/supination, the anatomical forearm axis is invariant:
    there is a receiver-local vector ``a`` for which ``R_i a`` is constant in
    the transmitter frame.  The dominant right singular vector of the mean
    rotation estimates ``a`` directly.  This invariant-vector formulation is
    less sensitive than fitting the axes of every long-range sample pair:
    small elbow corrections do not get amplified by pairs spanning opposite
    ends of the motion.

    Pairwise rotations are still used to measure the total motion range.  A
    robust iteration rejects isolated orientation glitches before reporting
    the invariant-axis residual.
    """
    rotations = np.asarray(rotations, dtype=float)
    if rotations.ndim != 3 or rotations.shape[1:] != (3, 3):
        raise ValueError('rotations must have shape (N, 3, 3)')
    if len(rotations) < 20:
        raise ValueError('at least 20 rotations are required for axis calibration')
    if not np.all(np.isfinite(rotations)):
        raise ValueError('rotation samples contain non-finite values')

    sample_count = min(len(rotations), max_samples)
    indices = np.unique(np.linspace(
        0, len(rotations) - 1, sample_count, dtype=int))
    motion_range = 0.0
    usable_pair_count = 0
    min_angle = math.radians(min_pair_angle_deg)
    max_angle = math.radians(max_pair_angle_deg)
    for first_offset, first in enumerate(indices[:-1]):
        for second in indices[first_offset + 1:]:
            vector = rotation_vector(
                rotations[first].T @ rotations[second])
            angle = float(np.linalg.norm(vector))
            motion_range = max(motion_range, angle)
            if min_angle <= angle <= max_angle:
                usable_pair_count += 1
    if usable_pair_count < 20:
        raise ValueError(
            'isolated twist motion did not contain enough rotation; '
            'rotate the palm farther in both directions')

    inliers = np.ones(len(rotations), dtype=bool)
    for _ in range(6):
        _, singular_values, right_vectors = np.linalg.svd(
            np.mean(rotations[inliers], axis=0))
        axis = right_vectors[0]
        directions = np.einsum('nij,j->ni', rotations, axis)
        mean_direction = np.mean(directions[inliers], axis=0)
        mean_norm = float(np.linalg.norm(mean_direction))
        if mean_norm < 1e-9:
            raise ValueError('cannot identify a stable forearm rotation axis')
        mean_direction /= mean_norm
        residuals = np.degrees(np.arccos(np.clip(
            directions @ mean_direction, -1.0, 1.0)))
        selected = residuals[inliers]
        median = float(np.median(selected))
        mad = float(np.median(np.abs(selected - median)))
        threshold = max(8.0, median + 3.0 * 1.4826 * mad)
        new_inliers = residuals <= threshold
        if np.count_nonzero(new_inliers) < max(20, int(0.6 * len(rotations))):
            break
        if np.array_equal(new_inliers, inliers):
            break
        inliers = new_inliers

    # Recompute once with the final inlier set so the returned axis and errors
    # correspond to the same samples.
    _, singular_values, right_vectors = np.linalg.svd(
        np.mean(rotations[inliers], axis=0))
    axis = right_vectors[0]
    directions = np.einsum('nij,j->ni', rotations, axis)
    mean_direction = np.mean(directions[inliers], axis=0)
    mean_direction /= np.linalg.norm(mean_direction)
    residuals = np.degrees(np.arccos(np.clip(
        directions @ mean_direction, -1.0, 1.0)))

    # With little axial motion all three singular values remain close to one,
    # making the dominant vector arbitrary.  The pairwise range check above is
    # the primary guard; this gap check catches a dispersed non-axial motion.
    if singular_values[0] - singular_values[1] < 1e-4:
        raise ValueError('motion does not reveal a unique forearm rotation axis')

    # Give the otherwise sign-ambiguous result a deterministic representation.
    largest_component = int(np.argmax(np.abs(axis)))
    if axis[largest_component] < 0.0:
        axis = -axis

    rms = float(np.sqrt(np.mean(np.square(residuals[inliers]))))
    maximum = float(np.max(residuals[inliers]))
    return LocalRotationAxisResult(
        axis_local=axis,
        rms_error_deg=rms,
        max_error_deg=maximum,
        motion_range_deg=math.degrees(motion_range),
        pair_count=int(np.count_nonzero(inliers)),
    )


def rotation_error_deg(actual: np.ndarray, wanted: np.ndarray) -> float:
    """Geodesic orientation error between two rotation matrices."""
    return math.degrees(float(np.linalg.norm(
        rotation_vector(np.asarray(wanted).T @ np.asarray(actual)))))


def fit_orientation_calibration(sensor_rotations_tx: np.ndarray,
                                target_segment_rotations_body: np.ndarray
                                ) -> OrientationCalibrationResult:
    """Fit transmitter alignment and sensor mounting from known arm poses.

    The model is ``R_segment_body = A R_sensor_tx M.T``.  Relative rotations
    first identify ``A`` without knowing the sensor mounting ``M``.  A short
    alternating refinement then minimizes full orientation disagreement over
    all calibration poses.  This avoids estimating a translating anatomical
    shoulder centre from skin-mounted sensors.
    """
    observed = np.asarray(sensor_rotations_tx, dtype=float)
    targets = np.asarray(target_segment_rotations_body, dtype=float)
    if observed.shape != targets.shape or observed.ndim != 3 or observed.shape[1:] != (3, 3):
        raise ValueError('sensor and target rotations must both have shape (N, 3, 3)')
    if len(observed) < 3:
        raise ValueError('at least three known calibration poses are required')

    observed_relative = []
    target_relative = []
    for first in range(len(observed)):
        for second in range(first + 1, len(observed)):
            observed_vector = rotation_vector(
                observed[first] @ observed[second].T)
            target_vector = rotation_vector(
                targets[first] @ targets[second].T)
            if (np.linalg.norm(observed_vector) > 0.05 and
                    np.linalg.norm(target_vector) > 0.05):
                observed_relative.append(observed_vector)
                target_relative.append(target_vector)
    if len(observed_relative) < 3:
        raise ValueError('known poses do not contain enough distinct rotations')

    cross_covariance = np.asarray(target_relative).T @ np.asarray(observed_relative)
    U, _, Vt = np.linalg.svd(cross_covariance)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(U @ Vt)
    alignment = U @ correction @ Vt
    mount = mean_rotation(np.array([
        target.T @ alignment @ sensor
        for sensor, target in zip(observed, targets)
    ]))

    for _ in range(20):
        measured_axes = []
        target_axes = []
        for sensor, target in zip(observed, targets):
            sensor_segment = sensor @ mount.T
            measured_axes.extend(sensor_segment.T)
            target_axes.extend(target.T)
        alignment, _ = fit_body_alignment(
            np.asarray(measured_axes), np.asarray(target_axes))
        mount = mean_rotation(np.array([
            target.T @ alignment @ sensor
            for sensor, target in zip(observed, targets)
        ]))

    errors = np.array([
        rotation_error_deg(alignment @ sensor @ mount.T, target)
        for sensor, target in zip(observed, targets)
    ])
    return OrientationCalibrationResult(alignment, mount, errors)


def fit_segment_direction_calibration(upper_rotations_tx: np.ndarray,
                                      forearm_rotations_tx: np.ndarray,
                                      target_upper_directions_body: np.ndarray,
                                      target_forearm_directions_body: np.ndarray,
                                      ) -> DirectionCalibrationResult:
    """Fit body alignment and both longitudinal axes from known arm poses.

    Unlike a full upper-arm orientation fit, this model does not assume that a
    skin-mounted upper-arm receiver accurately observes humeral axial twist.
    Shoulder axial rotation is later obtained from the plane formed by the two
    measured segment directions.
    """
    upper_rotations = np.asarray(upper_rotations_tx, dtype=float)
    forearm_rotations = np.asarray(forearm_rotations_tx, dtype=float)
    upper_targets = np.asarray(target_upper_directions_body, dtype=float)
    forearm_targets = np.asarray(target_forearm_directions_body, dtype=float)
    if upper_rotations.shape != forearm_rotations.shape:
        raise ValueError('upper and forearm rotations must have matching shapes')
    if upper_rotations.ndim != 3 or upper_rotations.shape[1:] != (3, 3):
        raise ValueError('receiver rotations must have shape (N, 3, 3)')
    if upper_targets.shape != (len(upper_rotations), 3):
        raise ValueError('upper target directions must have shape (N, 3)')
    if forearm_targets.shape != (len(upper_rotations), 3):
        raise ValueError('forearm target directions must have shape (N, 3)')
    if len(upper_rotations) < 3:
        raise ValueError('at least three paired known poses are required')
    upper_targets = upper_targets / np.linalg.norm(upper_targets, axis=1, keepdims=True)
    forearm_targets = forearm_targets / np.linalg.norm(
        forearm_targets, axis=1, keepdims=True)

    seeds = []
    for sign in (-1.0, 1.0):
        for axis_index in range(3):
            axis = np.zeros(3)
            axis[axis_index] = sign
            seeds.append(axis)

    best = None
    for upper_seed in seeds:
        for forearm_seed in seeds:
            upper_axis = upper_seed.copy()
            forearm_axis = forearm_seed.copy()
            for _ in range(50):
                measured_upper = np.einsum(
                    'nij,j->ni', upper_rotations, upper_axis)
                measured_forearm = np.einsum(
                    'nij,j->ni', forearm_rotations, forearm_axis)
                alignment, _ = fit_body_alignment(
                    np.vstack([measured_upper, measured_forearm]),
                    np.vstack([upper_targets, forearm_targets]))

                upper_estimates = np.einsum(
                    'nji,nj->ni', upper_rotations,
                    (alignment.T @ upper_targets.T).T)
                forearm_estimates = np.einsum(
                    'nji,nj->ni', forearm_rotations,
                    (alignment.T @ forearm_targets.T).T)
                new_upper_axis = np.mean(upper_estimates, axis=0)
                new_forearm_axis = np.mean(forearm_estimates, axis=0)
                new_upper_axis /= np.linalg.norm(new_upper_axis)
                new_forearm_axis /= np.linalg.norm(new_forearm_axis)
                change = (np.linalg.norm(new_upper_axis - upper_axis) +
                          np.linalg.norm(new_forearm_axis - forearm_axis))
                upper_axis = new_upper_axis
                forearm_axis = new_forearm_axis
                if change < 1e-10:
                    break

            measured_upper = np.einsum(
                'nij,j->ni', upper_rotations, upper_axis)
            measured_forearm = np.einsum(
                'nij,j->ni', forearm_rotations, forearm_axis)
            alignment, errors = fit_body_alignment(
                np.vstack([measured_upper, measured_forearm]),
                np.vstack([upper_targets, forearm_targets]))
            rms = float(np.sqrt(np.mean(np.square(errors))))
            if best is None or rms < best[0]:
                best = (rms, alignment, upper_axis, forearm_axis, errors)

    _, alignment, upper_axis, forearm_axis, errors = best
    sample_count = len(upper_rotations)
    return DirectionCalibrationResult(
        rotation_tx_to_body=alignment,
        upper_axis_local=upper_axis,
        forearm_axis_local=forearm_axis,
        upper_errors_deg=errors[:sample_count],
        forearm_errors_deg=errors[sample_count:],
    )


def fit_single_direction_calibration(rotations_tx: np.ndarray,
                                     target_directions_body: np.ndarray,
                                     ) -> SingleDirectionCalibrationResult:
    """Fit a body alignment and receiver-local longitudinal axis."""
    rotations = np.asarray(rotations_tx, dtype=float)
    targets = np.asarray(target_directions_body, dtype=float)
    if rotations.ndim != 3 or rotations.shape[1:] != (3, 3):
        raise ValueError('receiver rotations must have shape (N, 3, 3)')
    if targets.shape != (len(rotations), 3):
        raise ValueError('target directions must have shape (N, 3)')
    if len(rotations) < 3:
        raise ValueError('at least three known poses are required')
    targets = targets / np.linalg.norm(targets, axis=1, keepdims=True)

    seeds = []
    for sign in (-1.0, 1.0):
        for axis_index in range(3):
            axis = np.zeros(3)
            axis[axis_index] = sign
            seeds.append(axis)

    best = None
    for seed in seeds:
        axis = seed.copy()
        for _ in range(50):
            measured = np.einsum('nij,j->ni', rotations, axis)
            alignment, _ = fit_body_alignment(measured, targets)
            estimates = np.einsum(
                'nji,nj->ni', rotations,
                (alignment.T @ targets.T).T)
            new_axis = np.mean(estimates, axis=0)
            new_axis /= np.linalg.norm(new_axis)
            change = np.linalg.norm(new_axis - axis)
            axis = new_axis
            if change < 1e-10:
                break

        measured = np.einsum('nij,j->ni', rotations, axis)
        alignment, errors = fit_body_alignment(measured, targets)
        rms = float(np.sqrt(np.mean(np.square(errors))))
        if best is None or rms < best[0]:
            best = (rms, alignment, axis, errors)

    _, alignment, axis, errors = best
    return SingleDirectionCalibrationResult(
        rotation_tx_to_body=alignment,
        axis_local=axis,
        errors_deg=errors,
    )


def fit_single_direction_with_known_axis(
        rotations_tx: np.ndarray,
        target_directions_body: np.ndarray,
        axis_local: np.ndarray,
        ) -> SingleDirectionCalibrationResult:
    """Fit frame alignment after an isolated motion identified the local axis."""
    rotations = np.asarray(rotations_tx, dtype=float)
    targets = np.asarray(target_directions_body, dtype=float)
    axis = np.asarray(axis_local, dtype=float)
    if rotations.ndim != 3 or rotations.shape[1:] != (3, 3):
        raise ValueError('rotations must have shape (N, 3, 3)')
    if targets.shape != (len(rotations), 3):
        raise ValueError('target directions must have shape (N, 3)')
    if axis.shape != (3,) or np.linalg.norm(axis) < 1e-9:
        raise ValueError('axis_local must be a nonzero 3-vector')
    axis = axis / np.linalg.norm(axis)

    best = None
    for signed_axis in (axis, -axis):
        measured = np.einsum('nij,j->ni', rotations, signed_axis)
        alignment, errors = fit_body_alignment(measured, targets)
        rms = float(np.sqrt(np.mean(np.square(errors))))
        if best is None or rms < best[0]:
            best = (rms, alignment, signed_axis.copy(), errors)
    _, alignment, signed_axis, errors = best
    return SingleDirectionCalibrationResult(
        rotation_tx_to_body=alignment,
        axis_local=signed_axis,
        errors_deg=errors,
    )


def fit_position_direction_calibration(positions_tx: np.ndarray,
                                       target_directions_body: np.ndarray,
                                       ) -> PositionDirectionCalibrationResult:
    """Fit ``p = centre + radius * R_body_to_tx * direction``.

    The upper-arm receiver is treated as a point moving approximately on a
    sphere around the shoulder.  This is deliberately independent of its
    skin-mounted orientation.
    """
    positions = np.asarray(positions_tx, dtype=float)
    targets = np.asarray(target_directions_body, dtype=float)
    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError('positions must have shape (N, 3)')
    if targets.shape != positions.shape:
        raise ValueError('target directions must have shape (N, 3)')
    if len(positions) < 3:
        raise ValueError('at least three known poses are required')
    if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(targets)):
        raise ValueError('position calibration contains non-finite values')
    targets = targets / np.linalg.norm(targets, axis=1, keepdims=True)

    targets_centered = targets - np.mean(targets, axis=0)
    positions_centered = positions - np.mean(positions, axis=0)
    U, _, Vt = np.linalg.svd(targets_centered.T @ positions_centered)
    rotation_tx_to_body = U @ Vt
    if np.linalg.det(rotation_tx_to_body) < 0.0:
        U[:, -1] *= -1.0
        rotation_tx_to_body = U @ Vt

    denominator = float(np.sum(targets_centered * targets_centered))
    if denominator < 1e-9:
        raise ValueError('reference position directions do not span space')
    radius = float(np.sum(
        (targets_centered @ rotation_tx_to_body) * positions_centered
    ) / denominator)
    if radius <= 0.0:
        raise ValueError('fitted upper receiver radius is not positive')
    centre = (
        np.mean(positions, axis=0)
        - radius * np.mean(targets, axis=0) @ rotation_tx_to_body)

    predicted_positions = (
        centre + radius * targets @ rotation_tx_to_body)
    position_errors = np.linalg.norm(predicted_positions - positions, axis=1)
    measured_directions = (
        rotation_tx_to_body @ (positions - centre).T).T
    measured_directions /= np.linalg.norm(
        measured_directions, axis=1, keepdims=True)
    cosines = np.sum(measured_directions * targets, axis=1)
    direction_errors = np.degrees(np.arccos(np.clip(cosines, -1.0, 1.0)))
    return PositionDirectionCalibrationResult(
        rotation_tx_to_body=rotation_tx_to_body,
        shoulder_center_tx_m=centre,
        receiver_radius_m=radius,
        direction_errors_deg=direction_errors,
        position_errors_m=position_errors,
    )


def affine_position_direction(position_tx: np.ndarray,
                              centre_tx_m: np.ndarray,
                              body_to_tx_matrix_m: np.ndarray,
                              ) -> Tuple[np.ndarray, float]:
    """Map one receiver position to a unit body direction and model error.

    ``model_error_m`` is the distance from the sample to the same radial
    direction on the calibrated ellipsoid.  For an isotropic matrix this is
    exactly the former absolute sphere-radius error.
    """
    position = np.asarray(position_tx, dtype=float)
    centre = np.asarray(centre_tx_m, dtype=float)
    matrix = np.asarray(body_to_tx_matrix_m, dtype=float)
    if position.shape != (3,) or centre.shape != (3,) or matrix.shape != (3, 3):
        raise ValueError('affine position model expects 3-vectors and a 3x3 matrix')
    if not (np.all(np.isfinite(position)) and np.all(np.isfinite(centre)) and
            np.all(np.isfinite(matrix))):
        raise ValueError('affine position model contains non-finite values')
    singular_values = np.linalg.svd(matrix, compute_uv=False)
    if singular_values[-1] < 1e-6:
        raise ValueError('affine position model matrix is singular')

    # Row-vector calibration uses p - c = d @ B.  The equivalent column form
    # is d = inv(B.T) @ (p - c).
    raw_direction = np.linalg.solve(matrix.T, position - centre)
    raw_norm = float(np.linalg.norm(raw_direction))
    if raw_norm < 1e-9:
        raise ValueError('receiver is at the affine position-model centre')
    direction = raw_direction / raw_norm
    expected_position = centre + direction @ matrix
    model_error_m = float(np.linalg.norm(position - expected_position))
    return direction, model_error_m


def fit_affine_position_direction_calibration(
        positions_tx: np.ndarray,
        target_directions_body: np.ndarray,
        ) -> AffinePositionDirectionCalibrationResult:
    """Fit an ellipsoidal position-to-upper-arm-direction calibration.

    The reference directions must span a full affine 3-D basis.  In
    particular, DOWN/FORWARD/SIDEWAYS alone are insufficient; the two
    intermediate forward-raise references make the centre and all matrix
    components observable.
    """
    positions = np.asarray(positions_tx, dtype=float)
    targets = np.asarray(target_directions_body, dtype=float)
    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError('positions must have shape (N, 3)')
    if targets.shape != positions.shape:
        raise ValueError('target directions must have shape (N, 3)')
    if len(positions) < 4:
        raise ValueError('at least four known poses are required')
    if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(targets)):
        raise ValueError('affine position calibration contains non-finite values')
    target_norms = np.linalg.norm(targets, axis=1, keepdims=True)
    if np.any(target_norms < 1e-9):
        raise ValueError('target directions must be nonzero')
    targets = targets / target_norms

    design = np.column_stack((targets, np.ones(len(targets))))
    coefficients, _, rank, singular_values = np.linalg.lstsq(
        design, positions, rcond=None)
    if rank < 4:
        raise ValueError(
            'reference directions do not span the affine position model; '
            'collect the intermediate forward-raise references')
    body_to_tx = coefficients[:3, :]
    centre = coefficients[3, :]
    matrix_singular_values = np.linalg.svd(body_to_tx, compute_uv=False)
    if matrix_singular_values[-1] < 1e-6:
        raise ValueError('fitted affine position matrix is singular')
    condition = float(matrix_singular_values[0] / matrix_singular_values[-1])
    equivalent_radius = float(np.cbrt(abs(np.linalg.det(body_to_tx))))

    predicted_positions = centre + targets @ body_to_tx
    position_errors = np.linalg.norm(predicted_positions - positions, axis=1)
    measured_directions = []
    for position in positions:
        direction, _ = affine_position_direction(
            position, centre, body_to_tx)
        measured_directions.append(direction)
    measured_directions = np.asarray(measured_directions)
    cosines = np.sum(measured_directions * targets, axis=1)
    direction_errors = np.degrees(np.arccos(np.clip(cosines, -1.0, 1.0)))
    return AffinePositionDirectionCalibrationResult(
        centre_tx_m=centre,
        body_to_tx_matrix_m=body_to_tx,
        direction_errors_deg=direction_errors,
        position_errors_m=position_errors,
        equivalent_radius_m=equivalent_radius,
        condition_number=condition,
    )


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


def dual_joint_angles_from_directions(upper_sensor_rotation_tx: np.ndarray,
                                     forearm_sensor_rotation_tx: np.ndarray,
                                     rotation_tx_to_body: np.ndarray,
                                     upper_axis_local: np.ndarray,
                                     forearm_axis_local: np.ndarray,
                                     forearm_sensor_mount: np.ndarray,
                                     previous_yaw: float = 0.0,
                                     previous_wrist: float = 0.0,
                                     previous_pitch: float = 0.0,
                                     min_pitch_observability: float = 0.2,
                                     min_yaw_observability: float = 0.55,
                                     pitch_override: Optional[float] = None,
                                     roll_override: Optional[float] = None,
                                     ) -> np.ndarray:
    """Map two segment directions and forearm twist to five arm angles."""
    alignment = np.asarray(rotation_tx_to_body, dtype=float)
    upper_aligned = alignment @ np.asarray(upper_sensor_rotation_tx, dtype=float)
    forearm_aligned = alignment @ np.asarray(forearm_sensor_rotation_tx, dtype=float)
    upper_direction = upper_aligned @ np.asarray(upper_axis_local, dtype=float)
    forearm_direction = forearm_aligned @ np.asarray(forearm_axis_local, dtype=float)
    return dual_joint_angles_from_segment_vectors(
        upper_direction,
        forearm_direction,
        forearm_aligned,
        forearm_sensor_mount,
        previous_yaw=previous_yaw,
        previous_wrist=previous_wrist,
        previous_pitch=previous_pitch,
        min_pitch_observability=min_pitch_observability,
        min_yaw_observability=min_yaw_observability,
        pitch_override=pitch_override,
        roll_override=roll_override,
    )


def dual_joint_angles_from_segment_vectors(
        upper_direction_body: np.ndarray,
        forearm_direction_body: np.ndarray,
        forearm_sensor_rotation_body: np.ndarray,
        forearm_sensor_mount: np.ndarray,
        previous_yaw: float = 0.0,
        previous_wrist: float = 0.0,
        min_yaw_observability: float = 0.55,
        previous_pitch: float = 0.0,
        min_pitch_observability: float = 0.2,
        pitch_override: Optional[float] = None,
        roll_override: Optional[float] = None) -> np.ndarray:
    """Map body-frame segment directions and forearm attitude to joints."""
    upper_direction = np.asarray(upper_direction_body, dtype=float).copy()
    forearm_direction = np.asarray(forearm_direction_body, dtype=float).copy()
    forearm_aligned = np.asarray(forearm_sensor_rotation_body, dtype=float)
    if np.linalg.norm(upper_direction) < 1e-9:
        raise ValueError('upper-arm direction is zero')
    if np.linalg.norm(forearm_direction) < 1e-9:
        raise ValueError('forearm direction is zero')
    upper_direction /= np.linalg.norm(upper_direction)
    forearm_direction /= np.linalg.norm(forearm_direction)

    measured_pitch, roll = shoulder_pitch_roll(
        upper_direction,
        fallback_pitch=previous_pitch,
        min_horizontal=min_pitch_observability)
    pitch = (measured_pitch if pitch_override is None
             else float(pitch_override))
    roll = (roll if roll_override is None else float(roll_override))
    yaw, yaw_observable = shoulder_yaw(
        upper_direction, forearm_direction, pitch, roll,
        fallback=previous_yaw,
        min_perpendicular=min_yaw_observability)
    elbow = math.asin(float(np.clip(
        np.dot(upper_direction, forearm_direction), -1.0, 1.0)))
    if yaw_observable:
        wrist = wrist_roll(
            forearm_aligned,
            forearm_sensor_mount,
            pitch,
            roll,
            yaw,
            elbow,
            fallback=previous_wrist,
        )
    else:
        # With collinear upper-arm and forearm directions, shoulder axial
        # rotation and forearm twist cannot be separated.  Holding both last
        # observable values avoids assigning the same physical rotation to an
        # arbitrary combination of yaw and wrist.
        wrist = previous_wrist
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
