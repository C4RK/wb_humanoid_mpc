import math
import unittest

import numpy as np

from remote_control.g1_arm_collision import (
    G1ArmCollisionFilter,
    TORSO_MAX,
    TORSO_MIN,
    arm_torso_clearance,
    g1_left_arm_points,
    segment_aabb_distance,
)


class G1ArmCollisionTest(unittest.TestCase):

    def test_segment_aabb_distance_handles_intersection_and_separation(self):
        self.assertAlmostEqual(segment_aabb_distance(
            np.array([-2.0, 0.0, 0.0]), np.array([2.0, 0.0, 0.0]),
            np.array([-1.0, -1.0, -1.0]), np.array([1.0, 1.0, 1.0])), 0.0)
        self.assertAlmostEqual(segment_aabb_distance(
            np.array([2.0, -0.5, 0.0]), np.array([2.0, 0.5, 0.0]),
            np.array([-1.0, -1.0, -1.0]), np.array([1.0, 1.0, 1.0])), 1.0)
        self.assertAlmostEqual(segment_aabb_distance(
            np.array([2.0, 2.0, 0.0]), np.array([3.0, 3.0, 0.0]),
            np.array([-1.0, -1.0, -1.0]), np.array([1.0, 1.0, 1.0])),
            math.sqrt(2.0))

    def test_zero_pose_forward_kinematics_matches_urdf_chain(self):
        points = g1_left_arm_points(np.zeros(5))
        np.testing.assert_allclose(
            points.elbow - points.shoulder_yaw,
            np.array([0.015783, 0.0, -0.080518]), atol=2e-5)
        np.testing.assert_allclose(
            points.wrist - points.elbow,
            np.array([0.1, 0.00188791, -0.01]), atol=2e-5)
        self.assertGreater(points.shoulder_yaw[1], TORSO_MAX[1])
        self.assertGreater(points.elbow[1], TORSO_MAX[1])
        self.assertGreater(points.hand_tip[0], TORSO_MAX[0])
        self.assertGreater(points.shoulder_yaw[2], TORSO_MIN[2])

    def test_outward_roll_increases_clearance_for_arm_at_side(self):
        inward = np.radians([0.0, -15.0, 0.0, 0.0, 0.0])
        outward = np.radians([0.0, 20.0, 0.0, 0.0, 0.0])
        self.assertGreater(
            arm_torso_clearance(outward).clearance_m,
            arm_torso_clearance(inward).clearance_m)

    def test_filter_projects_inward_target_to_requested_margin(self):
        collision_filter = G1ArmCollisionFilter(clearance_margin_m=0.02)
        target = np.radians([0.0, -15.0, 0.0, 0.0, 0.0])

        result = collision_filter.filter(target)

        self.assertTrue(result.active)
        self.assertFalse(result.held_previous)
        self.assertGreater(result.shoulder_roll_adjustment_rad, 0.0)
        self.assertGreaterEqual(result.safe_clearance_m, 0.02 - 1e-7)
        np.testing.assert_allclose(
            result.angles_rad[[0, 2, 3, 4]],
            target[[0, 2, 3, 4]])

    def test_safe_side_raise_passes_through_unchanged(self):
        collision_filter = G1ArmCollisionFilter(clearance_margin_m=0.025)
        target = np.radians([0.0, 90.0, 0.0, 90.0, 0.0])

        result = collision_filter.filter(target)

        self.assertFalse(result.active)
        np.testing.assert_allclose(result.angles_rad, target)

    def test_filter_holds_previous_when_roll_projection_is_insufficient(self):
        collision_filter = G1ArmCollisionFilter(
            clearance_margin_m=0.08,
            max_roll_adjustment_rad=math.radians(1.0))
        target = np.radians([0.0, -15.0, 0.0, 0.0, 0.0])
        previous = np.radians([0.0, 60.0, 0.0, 0.0, 0.0])

        result = collision_filter.filter(target, previous_safe_rad=previous)

        self.assertTrue(result.active)
        self.assertTrue(result.held_previous)
        np.testing.assert_allclose(result.angles_rad, previous)


if __name__ == '__main__':
    unittest.main()
