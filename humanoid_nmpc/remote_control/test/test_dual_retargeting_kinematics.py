import math
import unittest

import numpy as np

from remote_control.dual_retargeting_kinematics import (
    dual_joint_angles,
    fit_body_alignment,
    reference_mounts,
    solve_elbow_pivot,
    solve_shoulder_pivot,
)
from remote_control.retargeting_kinematics import (
    arm_frames,
    rotation_x,
    rotation_y,
    rotation_z,
)


def random_rotation(rng):
    return (rotation_z(rng.uniform(-2.5, 2.5)) @
            rotation_y(rng.uniform(-1.2, 1.2)) @
            rotation_x(rng.uniform(-2.5, 2.5)))


class DualRetargetingKinematicsTest(unittest.TestCase):

    def test_shoulder_pivot_recovers_sensor_offset_and_center(self):
        rng = np.random.default_rng(12)
        shoulder = np.array([0.04, -0.03, 0.08])
        sensor_to_shoulder = np.array([-0.015, 0.19, 0.012])
        rotations = np.array([random_rotation(rng) for _ in range(120)])
        positions = np.array([shoulder - R @ sensor_to_shoulder for R in rotations])

        result = solve_shoulder_pivot(positions, rotations)

        np.testing.assert_allclose(result.first_local_offset, sensor_to_shoulder, atol=1e-10)
        np.testing.assert_allclose(result.second_local_offset_or_center, shoulder, atol=1e-10)
        self.assertEqual(result.rank, 6)
        self.assertLess(result.rms_m, 1e-10)

    def test_shoulder_pivot_rejects_packet_outliers(self):
        rng = np.random.default_rng(91)
        shoulder = np.array([0.025, 0.015, -0.04])
        sensor_to_shoulder = np.array([0.01, -0.18, 0.02])
        rotations = np.array([random_rotation(rng) for _ in range(160)])
        positions = np.array([shoulder - R @ sensor_to_shoulder for R in rotations])
        positions += rng.normal(scale=0.0008, size=positions.shape)
        positions[[11, 73, 124]] += np.array([0.08, -0.06, 0.05])

        result = solve_shoulder_pivot(positions, rotations)

        np.testing.assert_allclose(result.first_local_offset, sensor_to_shoulder, atol=0.001)
        np.testing.assert_allclose(result.second_local_offset_or_center, shoulder, atol=0.001)
        self.assertLess(result.inlier_count, result.sample_count)
        self.assertLess(result.rms_m, 0.002)

    def test_elbow_pivot_recovers_both_sensor_offsets(self):
        rng = np.random.default_rng(33)
        upper_to_elbow = np.array([0.01, 0.11, -0.015])
        forearm_to_elbow = np.array([-0.02, -0.21, 0.01])
        upper_positions = []
        forearm_positions = []
        upper_rotations = []
        forearm_rotations = []
        for _ in range(150):
            R_u = random_rotation(rng)
            relative = rotation_y(rng.uniform(-1.0, 1.3)) @ rotation_x(rng.uniform(-1.2, 1.2))
            R_f = R_u @ relative
            p_u = rng.uniform(-0.2, 0.2, size=3)
            elbow = p_u + R_u @ upper_to_elbow
            p_f = elbow - R_f @ forearm_to_elbow
            upper_positions.append(p_u)
            forearm_positions.append(p_f)
            upper_rotations.append(R_u)
            forearm_rotations.append(R_f)

        result = solve_elbow_pivot(
            np.array(upper_positions), np.array(upper_rotations),
            np.array(forearm_positions), np.array(forearm_rotations))

        np.testing.assert_allclose(result.first_local_offset, upper_to_elbow, atol=1e-10)
        np.testing.assert_allclose(result.second_local_offset_or_center, forearm_to_elbow, atol=1e-10)
        self.assertEqual(result.rank, 6)
        self.assertLess(result.rms_m, 1e-10)

    def test_body_alignment_maps_reference_directions(self):
        wanted = np.array([[0.0, 0.0, -1.0],
                           [1.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0]])
        actual_rotation = rotation_z(0.6) @ rotation_y(-0.35) @ rotation_x(0.2)
        measured = (actual_rotation.T @ wanted.T).T

        recovered, errors = fit_body_alignment(measured, wanted)

        np.testing.assert_allclose(recovered, actual_rotation, atol=1e-12)
        np.testing.assert_allclose(errors, 0.0, atol=2e-6)

    def test_dual_orientation_retargeting_recovers_all_five_joints(self):
        alignment = rotation_z(-0.45) @ rotation_x(0.22)
        upper_mount = rotation_y(0.31) @ rotation_x(-0.18)
        forearm_mount = rotation_z(-math.pi / 2.0) @ rotation_x(0.27)

        # Verify the helper reproduces the same mounts at the specified zero pose.
        ref_shoulder, ref_forearm = arm_frames(-math.pi / 2.0, 0.0, 0.0, 0.0)
        ref_upper_tx = alignment.T @ ref_shoulder @ upper_mount
        ref_forearm_tx = alignment.T @ ref_forearm @ forearm_mount
        recovered_upper_mount, recovered_forearm_mount = reference_mounts(
            alignment, ref_upper_tx, ref_forearm_tx)
        np.testing.assert_allclose(recovered_upper_mount, upper_mount, atol=1e-12)
        np.testing.assert_allclose(recovered_forearm_mount, forearm_mount, atol=1e-12)

        cases = [
            (-0.4, 0.25, 0.65, 0.35, -0.8),
            (-1.1, -0.35, -0.75, -0.45, 0.9),
            (0.35, 0.55, 1.1, 0.7, 0.25),
        ]
        for wanted in cases:
            with self.subTest(wanted=wanted):
                pitch, roll, yaw, elbow, wrist = wanted
                shoulder, forearm = arm_frames(pitch, roll, yaw, elbow)
                upper_tx = alignment.T @ shoulder @ upper_mount
                forearm_tx = alignment.T @ forearm @ rotation_x(wrist) @ forearm_mount

                actual = dual_joint_angles(
                    upper_tx, forearm_tx, alignment, upper_mount, forearm_mount)

                np.testing.assert_allclose(actual, wanted, atol=1e-9)


if __name__ == '__main__':
    unittest.main()
