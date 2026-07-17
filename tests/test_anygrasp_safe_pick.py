import math
import unittest

from geometry_msgs.msg import PoseStamped

from scripts.anygrasp_safe_pick import (
    SafetyConfig,
    build_pregrasp_pose,
    clamp,
    piperx_grasp_workspace_config,
    validate_candidate_center_pose,
    validate_grasp_pose,
)


def make_pose(x=0.45, y=0.0, z=0.16, q=(0.0, 0.0, 0.0, 1.0)):
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


class AnyGraspSafePickTest(unittest.TestCase):
    def test_accepts_pose_inside_workspace_above_table(self):
        decision = validate_grasp_pose(make_pose(), SafetyConfig())

        self.assertTrue(decision.accepted)
        self.assertEqual(decision.reason, "accepted")

    def test_default_workspace_accepts_low_tabletop_tcp_target_above_clearance(self):
        decision = validate_grasp_pose(make_pose(z=0.061), SafetyConfig())

        self.assertTrue(decision.accepted)

    def test_rejects_pose_below_table_clearance(self):
        cfg = SafetyConfig(table_z=0.03, min_table_clearance=0.04)

        decision = validate_grasp_pose(make_pose(z=0.055), cfg)

        self.assertFalse(decision.accepted)
        self.assertIn("below table clearance", decision.reason)

    def test_candidate_center_can_be_on_table_while_tcp_target_still_needs_clearance(self):
        cfg = SafetyConfig(table_z=0.0, min_table_clearance=0.06, min_z=0.08)
        table_object_center = make_pose(z=0.03)

        candidate_decision = validate_candidate_center_pose(table_object_center, cfg)
        tcp_target_decision = validate_grasp_pose(table_object_center, cfg)

        self.assertTrue(candidate_decision.accepted)
        self.assertFalse(tcp_target_decision.accepted)
        self.assertIn("below table clearance", tcp_target_decision.reason)

    def test_rejects_pose_outside_workspace(self):
        decision = validate_grasp_pose(make_pose(x=0.95), piperx_grasp_workspace_config())

        self.assertFalse(decision.accepted)
        self.assertIn("outside workspace x", decision.reason)

    def test_piperx_workspace_rejects_previous_far_anygrasp_result(self):
        decision = validate_grasp_pose(
            make_pose(x=0.866, y=0.322, z=0.037),
            piperx_grasp_workspace_config(),
        )

        self.assertFalse(decision.accepted)
        self.assertIn("outside workspace x", decision.reason)

    def test_piperx_workspace_accepts_conservative_front_grasp_area(self):
        decision = validate_grasp_pose(
            make_pose(x=0.42, y=0.05, z=0.16),
            piperx_grasp_workspace_config(),
        )

        self.assertTrue(decision.accepted)

    def test_rejects_invalid_quaternion(self):
        decision = validate_grasp_pose(make_pose(q=(0.0, 0.0, 0.0, 0.0)), SafetyConfig())

        self.assertFalse(decision.accepted)
        self.assertIn("invalid orientation", decision.reason)

    def test_pregrasp_keeps_orientation_and_adds_vertical_clearance(self):
        grasp = make_pose(z=0.12, q=(0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)))

        pregrasp = build_pregrasp_pose(grasp, vertical_standoff=0.08)

        self.assertEqual(pregrasp.header.frame_id, "world")
        self.assertAlmostEqual(pregrasp.pose.position.x, 0.45)
        self.assertAlmostEqual(pregrasp.pose.position.y, 0.0)
        self.assertAlmostEqual(pregrasp.pose.position.z, 0.20)
        self.assertAlmostEqual(pregrasp.pose.orientation.z, math.sqrt(0.5))
        self.assertAlmostEqual(pregrasp.pose.orientation.w, math.sqrt(0.5))

    def test_pregrasp_can_use_orientation_override(self):
        grasp = make_pose(z=0.12, q=(0.0, 0.0, 0.0, 1.0))
        current_tcp = make_pose(q=(0.1, 0.2, 0.3, 0.9))

        pregrasp = build_pregrasp_pose(
            grasp,
            vertical_standoff=0.08,
            orientation_source=current_tcp,
        )

        self.assertAlmostEqual(pregrasp.pose.position.z, 0.20)
        self.assertAlmostEqual(pregrasp.pose.orientation.x, 0.1)
        self.assertAlmostEqual(pregrasp.pose.orientation.y, 0.2)
        self.assertAlmostEqual(pregrasp.pose.orientation.z, 0.3)
        self.assertAlmostEqual(pregrasp.pose.orientation.w, 0.9)

    def test_clamp_limits_operator_parameters(self):
        self.assertEqual(clamp(-1.0, 0.02, 0.15), 0.02)
        self.assertEqual(clamp(0.08, 0.02, 0.15), 0.08)
        self.assertEqual(clamp(1.0, 0.02, 0.15), 0.15)


if __name__ == "__main__":
    unittest.main()
