import math
import unittest

import numpy as np

from remote_control.dual_retargeting_kinematics import (
    ROTATION_TRANSPOSE_REPORTED,
    diagnose_rotation_convention,
    dual_joint_angles,
    dual_joint_angles_from_directions,
    dual_joint_angles_from_segment_vectors,
    estimate_local_rotation_axis,
    fit_body_alignment,
    fit_orientation_calibration,
    fit_position_direction_calibration,
    fit_segment_direction_calibration,
    fit_single_direction_calibration,
    fit_single_direction_with_known_axis,
    relative_rotation_errors,
    reference_mounts,
    solve_elbow_pivot,
    solve_shoulder_pivot,
)
from remote_control.retargeting_kinematics import (
    FOREARM_ZERO_AXIS,
    UPPER_ARM_ZERO_AXIS,
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

    def test_isolated_twist_recovers_receiver_local_forearm_axis(self):
        alignment = rotation_z(0.37) @ rotation_y(-0.22)
        mount = rotation_y(0.31) @ rotation_z(-0.48)
        forearm_frame = rotation_z(0.26) @ rotation_y(-0.19)
        wrist_angles = np.linspace(-1.35, 1.25, 120)
        rotations = np.array([
            alignment.T @ forearm_frame @ rotation_x(angle) @ mount
            for angle in wrist_angles
        ])

        result = estimate_local_rotation_axis(rotations)
        expected = mount.T @ FOREARM_ZERO_AXIS

        self.assertGreater(abs(float(np.dot(result.axis_local, expected))), 0.9999)
        self.assertLess(result.rms_error_deg, 1e-5)
        self.assertGreater(result.motion_range_deg, 140.0)

    def test_known_twist_axis_rejects_wrist_rotation_from_direction_fit(self):
        alignment = rotation_z(-0.38) @ rotation_x(0.21)
        mount = rotation_y(0.29) @ rotation_z(-0.17)
        references = [
            (0.0, 0.0, 0.0, 0.0),
            (-math.pi / 2.0, 0.0, 0.0, 0.0),
            (0.0, math.pi / 2.0, 0.0, math.pi / 2.0),
            (0.0, 0.0, 0.0, math.pi / 2.0),
        ]
        wrist_angles = [0.8, -1.1, 0.55, -0.7]
        forearm_frames = np.array([
            arm_frames(*angles)[1] for angles in references])
        sensors = np.array([
            alignment.T @ frame @ rotation_x(wrist) @ mount
            for frame, wrist in zip(forearm_frames, wrist_angles)
        ])
        targets = np.array([
            frame @ FOREARM_ZERO_AXIS for frame in forearm_frames])

        result = fit_single_direction_with_known_axis(
            sensors, targets, mount.T @ FOREARM_ZERO_AXIS)

        np.testing.assert_allclose(result.rotation_tx_to_body, alignment, atol=1e-9)
        self.assertGreater(float(np.dot(
            result.axis_local, mount.T @ FOREARM_ZERO_AXIS)), 0.999999)
        np.testing.assert_allclose(result.errors_deg, 0.0, atol=2e-6)

    def test_relative_rotation_errors_identify_reported_direction(self):
        alignment = rotation_z(0.42) @ rotation_y(-0.31) @ rotation_x(0.18)
        mount = rotation_x(-0.27) @ rotation_z(0.63)
        targets = np.array([
            np.eye(3),
            rotation_y(-math.pi / 2.0),
            rotation_z(math.pi / 2.0) @ rotation_x(math.pi / 2.0),
        ])
        reported = np.array([
            alignment.T @ target @ mount
            for target in targets
        ])

        direct = relative_rotation_errors(reported, targets, alignment)
        transposed = relative_rotation_errors(
            np.swapaxes(reported, 1, 2), targets, alignment)

        self.assertLess(float(np.max(direct)), 1e-6)
        self.assertGreater(float(np.sqrt(np.mean(np.square(transposed)))), 20.0)

    def test_straight_arm_holds_unobservable_yaw_and_wrist(self):
        upper_direction = np.array([0.0, 0.0, -1.0])
        forearm_direction = upper_direction.copy()
        previous_yaw = 0.37
        previous_wrist = -0.62

        angles = dual_joint_angles_from_segment_vectors(
            upper_direction,
            forearm_direction,
            rotation_z(1.1) @ rotation_y(0.8),
            np.eye(3),
            previous_yaw=previous_yaw,
            previous_wrist=previous_wrist,
        )

        self.assertAlmostEqual(angles[2], previous_yaw)
        self.assertAlmostEqual(angles[3], math.pi / 2.0)
        self.assertAlmostEqual(angles[4], previous_wrist)

    def test_nearly_straight_arm_holds_unreliable_yaw_and_wrist(self):
        previous_yaw = -0.21
        previous_wrist = 0.34
        shoulder, forearm = arm_frames(
            0.0, 0.0, 1.1, math.radians(75.0))
        upper_direction = shoulder @ UPPER_ARM_ZERO_AXIS
        forearm_direction = forearm @ FOREARM_ZERO_AXIS

        angles = dual_joint_angles_from_segment_vectors(
            upper_direction,
            forearm_direction,
            forearm @ rotation_x(0.9),
            np.eye(3),
            previous_yaw=previous_yaw,
            previous_wrist=previous_wrist,
        )

        self.assertAlmostEqual(angles[2], previous_yaw)
        self.assertAlmostEqual(angles[3], math.radians(75.0))
        self.assertAlmostEqual(angles[4], previous_wrist)

    def test_sixty_degree_elbow_holds_unreliable_yaw_and_wrist(self):
        previous_yaw = -0.31
        previous_wrist = 0.28
        shoulder, forearm = arm_frames(
            0.0, 0.0, 0.9, math.radians(60.0))
        upper_direction = shoulder @ UPPER_ARM_ZERO_AXIS
        forearm_direction = forearm @ FOREARM_ZERO_AXIS

        angles = dual_joint_angles_from_segment_vectors(
            upper_direction,
            forearm_direction,
            forearm @ rotation_x(-0.7),
            np.eye(3),
            previous_yaw=previous_yaw,
            previous_wrist=previous_wrist,
        )

        self.assertAlmostEqual(angles[2], previous_yaw)
        self.assertAlmostEqual(angles[3], math.radians(60.0))
        self.assertAlmostEqual(angles[4], previous_wrist)

    def test_bent_arm_keeps_yaw_and_wrist_observable(self):
        wanted = (0.12, -0.08, 0.47, math.radians(55.0), -0.36)
        pitch, roll, yaw, elbow, wrist = wanted
        shoulder, forearm = arm_frames(pitch, roll, yaw, elbow)
        upper_direction = shoulder @ UPPER_ARM_ZERO_AXIS
        forearm_direction = forearm @ FOREARM_ZERO_AXIS

        angles = dual_joint_angles_from_segment_vectors(
            upper_direction,
            forearm_direction,
            forearm @ rotation_x(wrist),
            np.eye(3),
            previous_yaw=-0.9,
            previous_wrist=0.8,
        )

        np.testing.assert_allclose(angles, wanted, atol=1e-9)

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

    def test_shoulder_pivot_detects_transposed_reported_rotations(self):
        rng = np.random.default_rng(123)
        shoulder = np.array([-0.03, 0.02, 0.07])
        sensor_to_shoulder = np.array([0.015, -0.17, 0.025])
        actual_rotations = np.array([random_rotation(rng) for _ in range(140)])
        positions = np.array([
            shoulder - rotation @ sensor_to_shoulder
            for rotation in actual_rotations
        ])
        reported_rotations = np.swapaxes(actual_rotations, 1, 2)

        selected, results = diagnose_rotation_convention(
            positions, reported_rotations)

        self.assertEqual(selected, ROTATION_TRANSPOSE_REPORTED)
        self.assertLess(results[selected].rms_m, 1e-10)
        self.assertGreater(results['as_reported'].rms_m, 0.03)

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

    def test_multi_pose_orientation_calibration_recovers_alignment_and_mount(self):
        alignment = rotation_z(-0.42) @ rotation_x(0.18)
        mount = rotation_y(0.33) @ rotation_z(-0.21)
        target_rotations = np.array([
            arm_frames(0.0, 0.0, 0.0, 0.0)[0],
            arm_frames(-math.pi / 2.0, 0.0, 0.0, 0.0)[0],
            arm_frames(0.0, math.pi / 2.0, 0.0, 0.0)[0],
        ])
        sensor_rotations = np.array([
            alignment.T @ target @ mount
            for target in target_rotations
        ])

        result = fit_orientation_calibration(
            sensor_rotations, target_rotations)

        np.testing.assert_allclose(result.rotation_tx_to_body, alignment, atol=1e-10)
        np.testing.assert_allclose(result.sensor_mount_rotation, mount, atol=1e-10)
        np.testing.assert_allclose(result.errors_deg, 0.0, atol=2e-6)

    def test_direction_calibration_recovers_alignment_and_both_axes(self):
        alignment = rotation_z(0.37) @ rotation_x(-0.24)
        upper_mount = rotation_y(-0.28) @ rotation_z(0.19)
        forearm_mount = rotation_x(0.31) @ rotation_z(-0.22)
        reference_angles = [
            (0.0, 0.0, 0.0, 0.0),
            (-math.pi / 2.0, 0.0, 0.0, 0.0),
            (0.0, math.pi / 2.0, 0.0, 0.0),
        ]
        upper_frames = np.array([arm_frames(*angles)[0] for angles in reference_angles])
        forearm_frames = np.array([arm_frames(*angles)[1] for angles in reference_angles])
        upper_sensors = np.array([
            alignment.T @ frame @ upper_mount for frame in upper_frames])
        forearm_sensors = np.array([
            alignment.T @ frame @ forearm_mount for frame in forearm_frames])

        result = fit_segment_direction_calibration(
            upper_sensors,
            forearm_sensors,
            np.array([frame @ UPPER_ARM_ZERO_AXIS for frame in upper_frames]),
            np.array([frame @ FOREARM_ZERO_AXIS for frame in forearm_frames]))

        np.testing.assert_allclose(result.rotation_tx_to_body, alignment, atol=1e-6)
        np.testing.assert_allclose(
            result.upper_axis_local,
            upper_mount.T @ UPPER_ARM_ZERO_AXIS,
            atol=1e-6)
        np.testing.assert_allclose(
            result.forearm_axis_local,
            forearm_mount.T @ FOREARM_ZERO_AXIS,
            atol=1e-6)
        np.testing.assert_allclose(result.upper_errors_deg, 0.0, atol=1e-4)
        np.testing.assert_allclose(result.forearm_errors_deg, 0.0, atol=1e-4)

    def test_position_direction_calibration_recovers_centre_and_alignment(self):
        alignment = rotation_z(0.41) @ rotation_x(-0.23)
        centre = np.array([0.22, -0.04, 0.08])
        radius = 0.16
        targets = np.array([
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ])
        positions = np.array([
            centre + radius * alignment.T @ direction
            for direction in targets
        ])

        result = fit_position_direction_calibration(positions, targets)

        np.testing.assert_allclose(result.rotation_tx_to_body, alignment, atol=1e-10)
        np.testing.assert_allclose(result.shoulder_center_tx_m, centre, atol=1e-10)
        self.assertAlmostEqual(result.receiver_radius_m, radius, places=10)
        np.testing.assert_allclose(result.direction_errors_deg, 0.0, atol=2e-6)
        np.testing.assert_allclose(result.position_errors_m, 0.0, atol=1e-10)

    def test_single_direction_calibration_recovers_forearm_axis(self):
        alignment = rotation_z(-0.38) @ rotation_x(0.21)
        mount = rotation_y(0.29) @ rotation_z(-0.17)
        references = [
            (0.0, 0.0, 0.0, 0.0),
            (-math.pi / 2.0, 0.0, 0.0, 0.0),
            (0.0, math.pi / 2.0, 0.0, math.pi / 2.0),
        ]
        wrist_angles = [0.25, -0.45, 0.65]
        forearm_frames = np.array([arm_frames(*angles)[1] for angles in references])
        sensors = np.array([
            alignment.T @ frame @ rotation_x(wrist) @ mount
            for frame, wrist in zip(forearm_frames, wrist_angles)
        ])
        targets = np.array([
            frame @ FOREARM_ZERO_AXIS for frame in forearm_frames])

        result = fit_single_direction_calibration(sensors, targets)

        np.testing.assert_allclose(result.rotation_tx_to_body, alignment, atol=0.03)
        np.testing.assert_allclose(
            result.axis_local, mount.T @ FOREARM_ZERO_AXIS, atol=0.03)
        self.assertLess(float(np.max(result.errors_deg)), 0.5)

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

    def test_direction_retargeting_recovers_all_five_joints(self):
        alignment = rotation_z(0.31) @ rotation_x(-0.17)
        upper_mount = rotation_y(-0.26) @ rotation_z(0.14)
        forearm_mount = rotation_z(0.23) @ rotation_x(-0.32)
        upper_axis = upper_mount.T @ UPPER_ARM_ZERO_AXIS
        forearm_axis = forearm_mount.T @ FOREARM_ZERO_AXIS
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
                forearm_tx = (
                    alignment.T @ forearm @ rotation_x(wrist) @ forearm_mount)

                actual = dual_joint_angles_from_directions(
                    upper_tx, forearm_tx, alignment,
                    upper_axis, forearm_axis, forearm_mount)

                np.testing.assert_allclose(actual, wanted, atol=1e-9)

    def test_segment_vector_retargeting_recovers_all_five_joints(self):
        forearm_mount = rotation_z(0.27) @ rotation_x(-0.19)
        cases = [
            (-0.4, 0.25, 0.65, 0.35, -0.8),
            (-1.1, -0.35, -0.75, -0.45, 0.9),
            (0.35, 0.55, 1.1, 0.7, 0.25),
        ]
        for wanted in cases:
            with self.subTest(wanted=wanted):
                pitch, roll, yaw, elbow, wrist = wanted
                shoulder, forearm = arm_frames(pitch, roll, yaw, elbow)
                upper_direction = shoulder @ UPPER_ARM_ZERO_AXIS
                forearm_direction = forearm @ FOREARM_ZERO_AXIS
                forearm_sensor_body = (
                    forearm @ rotation_x(wrist) @ forearm_mount)

                actual = dual_joint_angles_from_segment_vectors(
                    upper_direction,
                    forearm_direction,
                    forearm_sensor_body,
                    forearm_mount)

                np.testing.assert_allclose(actual, wanted, atol=1e-9)


if __name__ == '__main__':
    unittest.main()
