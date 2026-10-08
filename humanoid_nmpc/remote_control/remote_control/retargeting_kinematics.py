"""Pure kinematics used by the EM arm retargeting and its simulator.

The functions in this module deliberately have no ROS dependencies.  Keeping
the geometry here makes it possible to regression-test the retargeting without
starting ROS or MuJoCo.
"""

import math
from typing import Dict, Tuple

import numpy as np


UPPER_ARM_ZERO_AXIS = np.array([0.0, 0.0, -1.0])
FOREARM_ZERO_AXIS = np.array([1.0, 0.0, 0.0])
SENSOR_FOREARM_AXIS = np.array([0.0, 1.0, 0.0])


def rotation_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, c, -s],
                     [0.0, s, c]])


def rotation_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s],
                     [0.0, 1.0, 0.0],
                     [-s, 0.0, c]])


def rotation_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0],
                     [s, c, 0.0],
                     [0.0, 0.0, 1.0]])


def arm_frames(pitch: float, roll: float, yaw: float,
               elbow: float) -> Tuple[np.ndarray, np.ndarray]:
    """Return the shoulder frame and pre-wrist forearm frame.

    This is the joint-axis model used by the retargeting:
    ``Ry(pitch) Rx(roll) Rz(yaw) Ry(elbow)``.  At zero, the upper arm
    points down and the forearm points forward.
    """
    shoulder = rotation_y(pitch) @ rotation_x(roll) @ rotation_z(yaw)
    forearm = shoulder @ rotation_y(elbow)
    return shoulder, forearm


def arm_directions(pitch: float, roll: float, yaw: float,
                   elbow: float) -> Tuple[np.ndarray, np.ndarray]:
    shoulder, forearm = arm_frames(pitch, roll, yaw, elbow)
    return shoulder @ UPPER_ARM_ZERO_AXIS, forearm @ FOREARM_ZERO_AXIS


def shoulder_pitch_roll(upper_arm_direction: np.ndarray) -> Tuple[float, float]:
    """Recover shoulder pitch and roll from the shoulder-to-elbow direction."""
    u = np.asarray(upper_arm_direction, dtype=float)
    norm = float(np.linalg.norm(u))
    if norm < 1e-9:
        raise ValueError('upper-arm direction is zero')
    u = u / norm

    horizontal = math.hypot(float(u[0]), float(u[2]))
    pitch = 0.0 if horizontal < 1e-8 else math.atan2(-float(u[0]), -float(u[2]))
    roll = math.asin(float(np.clip(u[1], -1.0, 1.0)))
    return pitch, roll


def shoulder_yaw(upper_arm_direction: np.ndarray,
                 forearm_direction: np.ndarray,
                 pitch: float,
                 roll: float,
                 yaw_offset: float = 0.0,
                 fallback: float = 0.0,
                 min_perpendicular: float = 0.12) -> Tuple[float, bool]:
    """Recover shoulder yaw from the elbow plane.

    The component of the forearm perpendicular to the upper arm identifies
    rotation around the upper-arm axis.  When the elbow is almost fully
    extended that component vanishes, so yaw is physically unobservable; the
    caller-provided fallback is returned in that case.
    """
    u = np.asarray(upper_arm_direction, dtype=float)
    f = np.asarray(forearm_direction, dtype=float)
    u /= np.linalg.norm(u)
    f /= np.linalg.norm(f)

    forearm_perp = f - float(np.dot(f, u)) * u
    magnitude = float(np.linalg.norm(forearm_perp))
    if magnitude < min_perpendicular:
        return fallback, False

    forearm_perp /= magnitude
    zero_yaw_frame = rotation_y(pitch) @ rotation_x(roll)
    zero_yaw_ref = zero_yaw_frame @ FOREARM_ZERO_AXIS
    zero_yaw_ref -= float(np.dot(zero_yaw_ref, u)) * u
    zero_yaw_ref /= np.linalg.norm(zero_yaw_ref)

    # The G1 yaw joint's positive axis is opposite the upper-arm direction.
    sine = -float(np.dot(np.cross(zero_yaw_ref, forearm_perp), u))
    cosine = float(np.dot(zero_yaw_ref, forearm_perp))
    angle = wrap_to_pi(math.atan2(sine, cosine) - yaw_offset)
    return unwrap_near(angle, fallback), True


def sensor_mount_from_reference(R_align: np.ndarray,
                                R_sensor_reference: np.ndarray,
                                reference_angles: Tuple[float, float, float, float] =
                                (-math.pi / 2.0, 0.0, 0.0, 0.0)) -> np.ndarray:
    """Return the fixed sensor-to-forearm rotation from calibration Step 3.

    Step 3 asks for upper arm forward, elbow bent 90 degrees, forearm up.
    In the retargeting convention this is ``pitch=-pi/2`` and all remaining
    arm joints zero.
    """
    _, reference_forearm = arm_frames(*reference_angles)
    aligned_reference = np.asarray(R_align) @ np.asarray(R_sensor_reference)
    return reference_forearm.T @ aligned_reference


def wrist_roll(R_sensor_aligned: np.ndarray,
               sensor_mount: np.ndarray,
               pitch: float,
               roll: float,
               yaw: float,
               elbow: float,
               fallback: float = 0.0) -> float:
    """Extract pronation/supination after removing shoulder and elbow motion."""
    _, forearm = arm_frames(pitch, roll, yaw, elbow)

    # R_sensor = R_forearm * Rx(wrist) * R_mount
    residual = forearm.T @ np.asarray(R_sensor_aligned) @ np.asarray(sensor_mount).T
    q = matrix_to_quaternion(residual)

    # Swing-twist decomposition around the forearm's local +X axis.  Projecting
    # the quaternion vector part onto X rejects small residual swing errors.
    twist_norm = math.hypot(float(q[0]), float(q[1]))
    if twist_norm < 1e-8:
        return fallback
    angle = wrap_to_pi(2.0 * math.atan2(float(q[1]), float(q[0])))
    return unwrap_near(angle, fallback)


def alignment_rotation_from_calibration(cal: Dict) -> Tuple[np.ndarray, str]:
    """Build the same transmitter-to-robot alignment used by retargeting."""
    down = np.array([cal.get('arm_down_hat_x', 0.0),
                     cal.get('arm_down_hat_y', 0.0),
                     cal.get('arm_down_hat_z', -1.0)], dtype=float)
    if np.linalg.norm(down) < 0.1:
        down = np.array([0.0, 0.0, -1.0])
    down /= np.linalg.norm(down)

    if cal.get('arm_forward_hat_x') is not None and cal.get('arm_sideways_hat_x') is not None:
        side = np.array([cal['arm_sideways_hat_x'],
                         cal['arm_sideways_hat_y'],
                         cal['arm_sideways_hat_z']], dtype=float)
        side /= np.linalg.norm(side)
        down_orthogonal = down - float(np.dot(down, side)) * side
        if np.linalg.norm(down_orthogonal) < 1e-8:
            raise ValueError('arm-down and arm-sideways calibration directions are parallel')
        e_down = down_orthogonal / np.linalg.norm(down_orthogonal)
        e_forward = np.cross(side, -e_down)
        e_forward /= np.linalg.norm(e_forward)
        source_frame = np.column_stack([e_forward, side, -e_down])
        return source_frame.T, 'side/down Gram-Schmidt'

    R1 = rotation_between(down, np.array([0.0, 0.0, -1.0]))
    if cal.get('arm_forward_hat_x') is not None:
        forward = np.array([cal['arm_forward_hat_x'],
                            cal['arm_forward_hat_y'],
                            cal['arm_forward_hat_z']], dtype=float)
        forward /= np.linalg.norm(forward)
        after_R1 = R1 @ forward
        xy_norm = float(np.linalg.norm(after_R1[:2]))
        if xy_norm > 0.1:
            theta = math.atan2(float(after_R1[1]), float(after_R1[0]))
            c, s = math.cos(theta), math.sin(theta)
            R2 = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
        else:
            R2 = np.eye(3)
        return R2 @ R1, 'down/forward two-step'
    return R1, 'arm-down only'


def calibration_alignment_errors(cal: Dict, R_align: np.ndarray) -> Dict[str, float]:
    """Return angular residuals, in degrees, for saved reference poses."""
    references = {
        'down': ('arm_down_hat', np.array([0.0, 0.0, -1.0])),
        'forward': ('arm_forward_hat', np.array([1.0, 0.0, 0.0])),
        'sideways': ('arm_sideways_hat', np.array([0.0, 1.0, 0.0])),
    }
    result = {}
    for label, (prefix, target) in references.items():
        if cal.get(prefix + '_x') is None:
            continue
        measured = np.array([cal[prefix + '_x'], cal[prefix + '_y'], cal[prefix + '_z']],
                            dtype=float)
        measured /= np.linalg.norm(measured)
        aligned = np.asarray(R_align) @ measured
        result[label] = math.degrees(math.acos(float(np.clip(np.dot(aligned, target), -1.0, 1.0))))
    return result


def quaternion_to_matrix(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        raise ValueError('quaternion norm is zero')
    w, x, y, z = q / norm
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Rotation matrix to normalized quaternion ``[w, x, y, z]``."""
    R = np.asarray(R, dtype=float)
    trace = float(np.trace(R))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = np.array([0.25 * s,
                      (R[2, 1] - R[1, 2]) / s,
                      (R[0, 2] - R[2, 0]) / s,
                      (R[1, 0] - R[0, 1]) / s])
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) * 2.0
            q = np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s,
                          (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
        elif i == 1:
            s = math.sqrt(max(0.0, 1.0 + R[1, 1] - R[0, 0] - R[2, 2])) * 2.0
            q = np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                          0.25 * s, (R[1, 2] + R[2, 1]) / s])
        else:
            s = math.sqrt(max(0.0, 1.0 + R[2, 2] - R[0, 0] - R[1, 1])) * 2.0
            q = np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                          (R[1, 2] + R[2, 1]) / s, 0.25 * s])
    q /= np.linalg.norm(q)
    return q if q[0] >= 0.0 else -q


def rotation_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    a /= np.linalg.norm(a)
    b /= np.linalg.norm(b)
    v = np.cross(a, b)
    s = float(np.linalg.norm(v))
    c = float(np.dot(a, b))
    if s < 1e-8:
        if c > 0.0:
            return np.eye(3)
        perpendicular = np.array([1.0, 0.0, 0.0])
        if abs(float(np.dot(perpendicular, a))) > 0.9:
            perpendicular = np.array([0.0, 1.0, 0.0])
        axis = np.cross(a, perpendicular)
        axis /= np.linalg.norm(axis)
        return 2.0 * np.outer(axis, axis) - np.eye(3)
    vx = np.array([[0.0, -v[2], v[1]],
                   [v[2], 0.0, -v[0]],
                   [-v[1], v[0], 0.0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / (s * s))


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def unwrap_near(angle: float, reference: float) -> float:
    return reference + wrap_to_pi(angle - reference)
