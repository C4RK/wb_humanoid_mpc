import math
import unittest

import numpy as np

from remote_control.retargeting_kinematics import (
    arm_directions,
    arm_frames,
    matrix_to_quaternion,
    quaternion_to_matrix,
    rotation_x,
    rotation_z,
    sensor_mount_from_reference,
    shoulder_pitch_roll,
    shoulder_yaw,
    wrist_roll,
)


class RetargetingKinematicsTest(unittest.TestCase):

    def test_shoulder_angles_recover_synthetic_arm(self):
        cases = [
            (0.0, 0.0, 0.0, 0.0),
            (-0.7, 0.35, 0.8, 0.2),
            (0.45, -0.4, -1.0, -0.35),
            (-1.1, 0.75, 1.25, 0.55),
        ]
        for pitch, roll, yaw, elbow in cases:
            with self.subTest(angles=(pitch, roll, yaw, elbow)):
                upper, forearm = arm_directions(pitch, roll, yaw, elbow)
                actual_pitch, actual_roll = shoulder_pitch_roll(upper)
                actual_yaw, observable = shoulder_yaw(
                    upper, forearm, actual_pitch, actual_roll)

                self.assertTrue(observable)
                self.assertAlmostEqual(actual_pitch, pitch, places=9)
                self.assertAlmostEqual(actual_roll, roll, places=9)
                self.assertAlmostEqual(actual_yaw, yaw, places=9)

    def test_shoulder_yaw_removes_calibrated_offset(self):
        pitch, roll, wanted_yaw, offset, elbow = -0.6, 0.25, 0.5, 0.17, 0.1
        upper, forearm = arm_directions(pitch, roll, wanted_yaw + offset, elbow)

        actual_yaw, observable = shoulder_yaw(
            upper, forearm, pitch, roll, yaw_offset=offset)

        self.assertTrue(observable)
        self.assertAlmostEqual(actual_yaw, wanted_yaw, places=9)

    def test_shoulder_yaw_holds_previous_value_near_full_extension(self):
        pitch, roll, yaw = -0.4, 0.3, 1.0
        upper, forearm = arm_directions(pitch, roll, yaw, math.radians(89.0))

        actual_yaw, observable = shoulder_yaw(
            upper, forearm, pitch, roll, fallback=0.42)

        self.assertFalse(observable)
        self.assertAlmostEqual(actual_yaw, 0.42)

    def test_wrist_roll_rejects_shoulder_and_elbow_motion(self):
        # This fixed mount maps the receiver's local +Y axis to the anatomical
        # forearm +X axis and includes an arbitrary 23-degree strap twist.
        mount = rotation_x(math.radians(23.0)) @ rotation_z(-math.pi / 2.0)
        _, reference_forearm = arm_frames(-math.pi / 2.0, 0.0, 0.0, 0.0)
        reference_sensor = reference_forearm @ mount
        recovered_mount = sensor_mount_from_reference(np.eye(3), reference_sensor)

        for wanted_wrist in [-1.2, -0.4, 0.0, 0.65, 1.4]:
            with self.subTest(wrist=wanted_wrist):
                pitch, roll, yaw, elbow = -0.85, 0.5, -0.7, 0.35
                _, forearm = arm_frames(pitch, roll, yaw, elbow)
                current_sensor = forearm @ rotation_x(wanted_wrist) @ mount

                actual_wrist = wrist_roll(
                    current_sensor, recovered_mount, pitch, roll, yaw, elbow)

                self.assertAlmostEqual(actual_wrist, wanted_wrist, places=9)

    def test_rotation_matrix_quaternion_round_trip(self):
        matrix = rotation_x(0.8) @ rotation_z(-1.1)
        quaternion = matrix_to_quaternion(matrix)
        recovered = quaternion_to_matrix(quaternion)
        np.testing.assert_allclose(recovered, matrix, atol=1e-12)


if __name__ == '__main__':
    unittest.main()
