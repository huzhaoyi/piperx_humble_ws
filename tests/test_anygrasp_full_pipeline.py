import unittest
from unittest.mock import patch

from geometry_msgs.msg import PoseStamped

from scripts.anygrasp_full_pipeline import (
    build_linear_waypoints,
    build_world_lift_pose,
    candidates_from_selection_snapshot,
    is_empty_grasp,
    offset_pose_along_local_z,
    parse_args,
    parse_float_list,
    selection_snapshot_from_plan,
)


def make_pose(x=0.40, y=0.0, z=0.20):
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = 1.0
    return pose


class GripperFeedback:
    def __init__(self, width, force):
        self.width = width
        self.force = force


class AnyGraspFullPipelineTest(unittest.TestCase):
    def test_offset_pose_along_local_z_moves_against_approach_axis(self):
        grasp = make_pose()

        pregrasp = offset_pose_along_local_z(grasp, -0.10)

        self.assertAlmostEqual(pregrasp.pose.position.x, 0.40)
        self.assertAlmostEqual(pregrasp.pose.position.y, 0.0)
        self.assertAlmostEqual(pregrasp.pose.position.z, 0.10)
        self.assertAlmostEqual(pregrasp.pose.orientation.w, 1.0)

    def test_build_linear_waypoints_interpolates_position_and_keeps_orientation(self):
        start = make_pose(z=0.10)
        goal = make_pose(z=0.20)

        waypoints = build_linear_waypoints(start, goal, step=0.04)

        self.assertEqual(len(waypoints), 3)
        self.assertAlmostEqual(waypoints[0].pose.position.z, 0.14)
        self.assertAlmostEqual(waypoints[1].pose.position.z, 0.18)
        self.assertAlmostEqual(waypoints[2].pose.position.z, 0.20)
        self.assertAlmostEqual(waypoints[2].pose.orientation.w, 1.0)

    def test_build_world_lift_pose_moves_up_in_world_z(self):
        grasp = make_pose(z=0.20)

        lift = build_world_lift_pose(grasp, 0.08)

        self.assertAlmostEqual(lift.pose.position.x, 0.40)
        self.assertAlmostEqual(lift.pose.position.z, 0.28)

    def test_is_empty_grasp_rejects_closed_low_force_feedback(self):
        status = GripperFeedback(0.0001, -0.05)

        self.assertTrue(is_empty_grasp(status, 0.003, 0.2))

    def test_is_empty_grasp_accepts_nonzero_width_or_contact_force(self):
        status = GripperFeedback(0.010, 0.05)

        self.assertFalse(is_empty_grasp(status, 0.003, 0.2))

        status = GripperFeedback(0.0001, 0.35)

        self.assertFalse(is_empty_grasp(status, 0.003, 0.2))

    def test_default_motion_parameters_are_conservative_for_real_grasp(self):
        with patch("sys.argv", ["anygrasp_full_pipeline.py"]):
            args = parse_args()

        self.assertLessEqual(args.velocity_scale, 0.05)
        self.assertLessEqual(args.acceleration_scale, 0.03)
        self.assertGreaterEqual(args.cartesian_step, 0.04)

    def test_default_gripper_open_width_uses_available_piper_range(self):
        with patch("sys.argv", ["anygrasp_full_pipeline.py"]):
            args = parse_args()

        self.assertGreaterEqual(args.open_width, 0.08)

    def test_default_tcp_grasp_offset_matches_piper_gripper_center(self):
        with patch("sys.argv", ["anygrasp_full_pipeline.py"]):
            args = parse_args()

        self.assertAlmostEqual(args.tcp_grasp_offset_z, 0.12)

    def test_candidates_from_selection_snapshot_freezes_selected_grasp_only(self):
        snapshot = {
            "frame_id": "world",
            "selected_index": 1,
            "selected_grasp": {
                "position": {"x": 0.31, "y": 0.04, "z": 0.13},
                "orientation": {"x": 0.1, "y": 0.2, "z": 0.3, "w": 0.9},
            },
            "candidates": [
                {
                    "position": {"x": 0.10, "y": 0.0, "z": 0.10},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                {
                    "position": {"x": 0.31, "y": 0.04, "z": 0.13},
                    "orientation": {"x": 0.1, "y": 0.2, "z": 0.3, "w": 0.9},
                },
            ],
        }

        candidates = candidates_from_selection_snapshot(snapshot)

        self.assertEqual(candidates.header.frame_id, "world")
        self.assertEqual(len(candidates.poses), 1)
        self.assertAlmostEqual(candidates.poses[0].position.x, 0.31)
        self.assertAlmostEqual(candidates.poses[0].orientation.w, 0.9)

    def test_selection_snapshot_from_plan_records_selected_raw_grasp(self):
        raw_grasp = make_pose(x=0.33, y=0.02, z=0.14)
        plan = {"index": 7, "raw_grasp": raw_grasp}

        snapshot = selection_snapshot_from_plan(plan)

        self.assertEqual(snapshot["frame_id"], "world")
        self.assertEqual(snapshot["selected_index"], 0)
        self.assertEqual(len(snapshot["candidates"]), 1)
        self.assertAlmostEqual(snapshot["selected_grasp"]["position"]["x"], 0.33)
        self.assertAlmostEqual(snapshot["selected_grasp"]["orientation"]["w"], 1.0)

    def test_parse_float_list_supports_offset_scan_arguments(self):
        self.assertEqual(parse_float_list("0.12,0.10, 0.08"), [0.12, 0.10, 0.08])
        self.assertEqual(parse_float_list(""), [])


if __name__ == "__main__":
    unittest.main()
