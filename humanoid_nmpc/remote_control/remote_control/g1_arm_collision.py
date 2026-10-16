"""Lightweight G1 left-arm versus torso collision safety filter.

The MPC currently accepts arm joint references directly and its existing
collision term covers feet and knees.  This module provides a deterministic
reference filter that has no runtime dependency on MuJoCo or Pinocchio.  The
kinematic constants come from ``g1_23dof.urdf`` and the collision primitives
conservatively approximate the corresponding mesh bounds.
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

import numpy as np


# Fixed transforms from robot_models/unitree_g1/g1_description/urdf/
# g1_23dof.urdf.  Coordinates are expressed in torso_link.
SHOULDER_PITCH_ORIGIN = np.array([0.0039563, 0.10022, 0.23778])
SHOULDER_PITCH_RPY = np.array([0.27931, 5.4949e-05, -0.00019159])
SHOULDER_ROLL_ORIGIN = np.array([0.0, 0.038, -0.013831])
SHOULDER_ROLL_RPY = np.array([-0.27925, 0.0, 0.0])
SHOULDER_YAW_ORIGIN = np.array([0.0, 0.00624, -0.1032])
ELBOW_ORIGIN = np.array([0.015783, 0.0, -0.080518])
WRIST_ORIGIN = np.array([0.100, 0.00188791, -0.010])
HAND_LENGTH_M = 0.235

# Axis-aligned bounds of torso_link.STL in torso_link coordinates.  The arm
# radii and a configurable clearance margin expand this box during checks.
TORSO_MIN = np.array([-0.06665, -0.10772, -0.00995])
TORSO_MAX = np.array([0.08364, 0.10772, 0.30037])

# Conservative capsule radii for the distal upper arm, forearm and hand.
UPPER_ARM_RADIUS_M = 0.038
FOREARM_RADIUS_M = 0.036
HAND_RADIUS_M = 0.045

LEFT_SHOULDER_ROLL_LIMIT = (-1.5882, 2.2515)


@dataclass(frozen=True)
class LeftArmPoints:
    """Key left-arm points in the G1 torso frame."""

    shoulder_yaw: np.ndarray
    elbow: np.ndarray
    wrist: np.ndarray
    hand_tip: np.ndarray


@dataclass(frozen=True)
class ArmTorsoCollisionCheck:
    """Minimum surface clearance of the arm from the torso."""

    clearance_m: float
    limiting_link: str


@dataclass(frozen=True)
class ArmCollisionFilterResult:
    """Result of projecting one arm target into the safe workspace."""

    angles_rad: np.ndarray
    raw_clearance_m: float
    safe_clearance_m: float
    limiting_link: str
    active: bool
    held_previous: bool
    shoulder_roll_adjustment_rad: float


def _rotation_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rotation_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rotation_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rpy_rotation(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = map(float, rpy)
    return _rotation_z(yaw) @ _rotation_y(pitch) @ _rotation_x(roll)


def g1_left_arm_points(angles_rad: np.ndarray) -> LeftArmPoints:
    """Forward kinematics for the five controlled G1 left-arm joints."""
    q = np.asarray(angles_rad, dtype=float).reshape(-1)
    if q.size != 5 or not np.all(np.isfinite(q)):
        raise ValueError('five finite left-arm angles are required')
    pitch, roll, yaw, elbow, wrist = map(float, q)

    p_pitch = SHOULDER_PITCH_ORIGIN.copy()
    R_pitch = _rpy_rotation(SHOULDER_PITCH_RPY) @ _rotation_y(pitch)

    p_roll = p_pitch + R_pitch @ SHOULDER_ROLL_ORIGIN
    R_roll = R_pitch @ _rpy_rotation(SHOULDER_ROLL_RPY) @ _rotation_x(roll)

    p_yaw = p_roll + R_roll @ SHOULDER_YAW_ORIGIN
    R_yaw = R_roll @ _rotation_z(yaw)

    p_elbow = p_yaw + R_yaw @ ELBOW_ORIGIN
    R_elbow = R_yaw @ _rotation_y(elbow)

    p_wrist = p_elbow + R_elbow @ WRIST_ORIGIN
    R_wrist = R_elbow @ _rotation_x(wrist)
    p_hand_tip = p_wrist + R_wrist @ np.array([HAND_LENGTH_M, 0.0, 0.0])
    return LeftArmPoints(
        shoulder_yaw=p_yaw,
        elbow=p_elbow,
        wrist=p_wrist,
        hand_tip=p_hand_tip,
    )


def _point_aabb_distance_squared(point: np.ndarray,
                                 box_min: np.ndarray,
                                 box_max: np.ndarray) -> float:
    delta = np.maximum(np.maximum(box_min - point, point - box_max), 0.0)
    return float(np.dot(delta, delta))


def segment_aabb_distance(start: np.ndarray, end: np.ndarray,
                          box_min: np.ndarray, box_max: np.ndarray) -> float:
    """Exact Euclidean distance between a line segment and an AABB.

    The squared point-to-box distance is a convex piecewise quadratic along
    the segment.  Evaluating its breakpoints and the stationary point in each
    interval avoids a sampling gap in the safety check.
    """
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    box_min = np.asarray(box_min, dtype=float)
    box_max = np.asarray(box_max, dtype=float)
    if any(value.shape != (3,) for value in (start, end, box_min, box_max)):
        raise ValueError('segment and AABB inputs must be 3-vectors')
    direction = end - start
    breakpoints = [0.0, 1.0]
    for axis in range(3):
        if abs(float(direction[axis])) > 1e-12:
            for bound in (box_min[axis], box_max[axis]):
                value = float((bound - start[axis]) / direction[axis])
                if 0.0 < value < 1.0:
                    breakpoints.append(value)
    breakpoints = sorted(set(breakpoints))
    candidates = list(breakpoints)
    for lower, upper in zip(breakpoints[:-1], breakpoints[1:]):
        midpoint = 0.5 * (lower + upper)
        point = start + midpoint * direction
        active_axes = []
        bounds = []
        for axis in range(3):
            if point[axis] < box_min[axis]:
                active_axes.append(axis)
                bounds.append(float(box_min[axis]))
            elif point[axis] > box_max[axis]:
                active_axes.append(axis)
                bounds.append(float(box_max[axis]))
        denominator = sum(float(direction[i] ** 2) for i in active_axes)
        if denominator > 1e-18:
            numerator = sum(
                float(direction[i] * (start[i] - bound))
                for i, bound in zip(active_axes, bounds))
            stationary = float(np.clip(-numerator / denominator, lower, upper))
            candidates.append(stationary)
    minimum_squared = min(
        _point_aabb_distance_squared(start + value * direction,
                                     box_min, box_max)
        for value in candidates)
    return math.sqrt(max(0.0, minimum_squared))


def arm_torso_clearance(angles_rad: np.ndarray) -> ArmTorsoCollisionCheck:
    """Return physical surface clearance using conservative link capsules."""
    points = g1_left_arm_points(angles_rad)
    # The shoulder is attached next to the torso by design.  Starting the
    # upper-arm capsule 35% down the segment excludes that intended proximal
    # adjacency while protecting the portion that can swing into the chest.
    upper_start = points.shoulder_yaw + 0.35 * (
        points.elbow - points.shoulder_yaw)
    capsules = (
        ('upper_arm', upper_start, points.elbow, UPPER_ARM_RADIUS_M),
        ('forearm', points.elbow, points.wrist, FOREARM_RADIUS_M),
        ('hand', points.wrist, points.hand_tip, HAND_RADIUS_M),
    )
    clearances = [
        (segment_aabb_distance(start, end, TORSO_MIN, TORSO_MAX) - radius,
         name)
        for name, start, end, radius in capsules
    ]
    clearance, link = min(clearances, key=lambda item: item[0])
    return ArmTorsoCollisionCheck(float(clearance), link)


class G1ArmCollisionFilter:
    """Project unsafe arm targets outward using shoulder roll."""

    def __init__(self, clearance_margin_m: float = 0.025,
                 search_step_rad: float = math.radians(1.0),
                 max_roll_adjustment_rad: float = math.radians(45.0)):
        if clearance_margin_m < 0.0:
            raise ValueError('collision clearance margin must be non-negative')
        if search_step_rad <= 0.0 or max_roll_adjustment_rad <= 0.0:
            raise ValueError('collision search parameters must be positive')
        self.clearance_margin_m = float(clearance_margin_m)
        self.search_step_rad = float(search_step_rad)
        self.max_roll_adjustment_rad = float(max_roll_adjustment_rad)

    def check(self, angles_rad: np.ndarray) -> ArmTorsoCollisionCheck:
        return arm_torso_clearance(angles_rad)

    def _is_safe(self, angles_rad: np.ndarray) -> Tuple[bool, ArmTorsoCollisionCheck]:
        check = self.check(angles_rad)
        return check.clearance_m >= self.clearance_margin_m, check

    def filter(self, angles_rad: np.ndarray,
               previous_safe_rad: Optional[np.ndarray] = None,
               ) -> ArmCollisionFilterResult:
        target = np.asarray(angles_rad, dtype=float).reshape(-1)
        if target.size != 5 or not np.all(np.isfinite(target)):
            raise ValueError('five finite left-arm target angles are required')
        raw_check = self.check(target)
        if raw_check.clearance_m >= self.clearance_margin_m:
            return ArmCollisionFilterResult(
                target.copy(), raw_check.clearance_m, raw_check.clearance_m,
                raw_check.limiting_link, False, False, 0.0)

        original_roll = float(target[1])
        maximum_roll = min(
            LEFT_SHOULDER_ROLL_LIMIT[1],
            original_roll + self.max_roll_adjustment_rad)
        last_unsafe_roll = original_roll
        safe_roll = None
        safe_check = None
        roll = original_roll + self.search_step_rad
        while roll <= maximum_roll + 1e-12:
            candidate = target.copy()
            candidate[1] = min(roll, maximum_roll)
            safe, check = self._is_safe(candidate)
            if safe:
                safe_roll = float(candidate[1])
                safe_check = check
                break
            last_unsafe_roll = float(candidate[1])
            roll += self.search_step_rad

        if safe_roll is not None:
            # Refine the first safe interval to avoid visible one-degree steps.
            lower, upper = last_unsafe_roll, safe_roll
            for _ in range(18):
                middle = 0.5 * (lower + upper)
                candidate = target.copy()
                candidate[1] = middle
                safe, check = self._is_safe(candidate)
                if safe:
                    upper = middle
                    safe_check = check
                else:
                    lower = middle
            projected = target.copy()
            projected[1] = upper
            final_check = self.check(projected)
            return ArmCollisionFilterResult(
                projected, raw_check.clearance_m, final_check.clearance_m,
                raw_check.limiting_link, True, False,
                float(projected[1] - original_roll))

        if previous_safe_rad is not None:
            previous = np.asarray(previous_safe_rad, dtype=float).reshape(-1)
            if previous.size == 5 and np.all(np.isfinite(previous)):
                previous_safe, previous_check = self._is_safe(previous)
                if previous_safe:
                    return ArmCollisionFilterResult(
                        previous.copy(), raw_check.clearance_m,
                        previous_check.clearance_m, raw_check.limiting_link,
                        True, True, float(previous[1] - original_roll))

        # No safe target is available.  Returning the input is useful to
        # callers that run the filter in report-only mode; publishing callers
        # must suppress this result when safe_clearance is below the margin.
        return ArmCollisionFilterResult(
            target.copy(), raw_check.clearance_m, raw_check.clearance_m,
            raw_check.limiting_link, True, True, 0.0)
