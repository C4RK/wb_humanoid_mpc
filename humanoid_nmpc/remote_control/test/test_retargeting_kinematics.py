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
    shoulder_pitch_roll_continuous,
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

    def test_shoulder_pitch_holds_previous_value_near_sideways_singularity(self):
        upper, _ = arm_directions(1.7, math.radians(85.0), 0.0, 0.0)

        actual_pitch, actual_roll = shoulder_pitch_roll(
            upper, fallback_pitch=-0.42, min_horizontal=0.2)

        self.assertAlmostEqual(actual_pitch, -0.42)
        self.assertAlmostEqual(actual_roll, math.radians(85.0), places=9)

    def test_shoulder_pitch_uses_nearest_periodic_solution(self):
        upper, _ = arm_directions(math.radians(-179.0), 0.0, 0.0, 0.0)

        actual_pitch, _ = shoulder_pitch_roll(
            upper, fallback_pitch=math.radians(179.0))

        self.assertAlmostEqual(actual_pitch, math.radians(181.0), places=9)

    def test_shoulder_pitch_hysteresis_uses_canonical_side_solution(self):
        previous_pitch = 0.0
        locked = False

        upper, _ = arm_directions(0.0, 0.0, 0.0, 0.0)
        pitch, _, locked, observability = shoulder_pitch_roll_continuous(
            upper, previous_pitch, locked)
        self.assertFalse(locked)
        self.assertGreater(observability, 0.65)

        # The measured direction contains a large apparent pitch near a side
        # raise.  It must select the canonical side solution rather than keep
        # a path-dependent preceding value.
        upper, _ = arm_directions(2.0, math.radians(70.0), 0.0, 0.0)
        pitch, _, locked, observability = shoulder_pitch_roll_continuous(
            upper, pitch, locked)
        self.assertTrue(locked)
        self.assertLess(observability, 0.5)
        self.assertAlmostEqual(pitch, 0.0)

        # Remaining inside the hysteresis band keeps the canonical solution.
        upper, _ = arm_directions(-1.7, math.radians(55.0), 0.0, 0.0)
        pitch, _, locked, observability = shoulder_pitch_roll_continuous(
            upper, pitch, locked)
        self.assertTrue(locked)
        self.assertGreater(observability, 0.5)
        self.assertLess(observability, 0.65)
        self.assertAlmostEqual(pitch, 0.0)

        # Once clearly away from sideways, pitch is safely reacquired.
        upper, _ = arm_directions(-0.3, math.radians(40.0), 0.0, 0.0)
        pitch, _, locked, observability = shoulder_pitch_roll_continuous(
            upper, pitch, locked)
        self.assertFalse(locked)
        self.assertGreater(observability, 0.65)
        self.assertAlmostEqual(pitch, -0.3, places=9)

    def test_side_solution_does_not_depend_on_approach_pitch(self):
        upper, _ = arm_directions(1.9, math.radians(78.0), 0.0, 0.0)

        from_forward = shoulder_pitch_roll_continuous(
            upper, previous_pitch=-1.8, pitch_locked=False)
        from_down = shoulder_pitch_roll_continuous(
            upper, previous_pitch=0.4, pitch_locked=False)

        self.assertTrue(from_forward[2])
        self.assertTrue(from_down[2])
        self.assertAlmostEqual(from_forward[0], 0.0)
        self.assertAlmostEqual(from_down[0], 0.0)

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
