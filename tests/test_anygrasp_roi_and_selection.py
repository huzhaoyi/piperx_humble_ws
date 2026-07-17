import unittest

import numpy as np
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from sensor_msgs.msg import CameraInfo, RegionOfInterest

from scripts.anygrasp_ros_infer import build_point_cloud, normalize_roi
from scripts.anygrasp_safe_pick import (
    SafetyConfig,
    apply_tcp_axis_mapping,
    build_candidate_markers,
    build_frozen_selection_markers,
    build_pose_axes_markers,
    build_selection_snapshot,
    quaternion_to_matrix,
    select_pregrasp_by_index,
)


def camera_info():
    info = CameraInfo()
    info.width = 4
    info.height = 3
    info.k = [100.0, 0.0, 1.5, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0]
    return info


def roi(x_offset, y_offset, width, height):
    msg = RegionOfInterest()
    msg.x_offset = x_offset
    msg.y_offset = y_offset
    msg.width = width
    msg.height = height
    return msg


def pose(x, y, z):
    msg = Pose()
    msg.position.x = x
    msg.position.y = y
    msg.position.z = z
    msg.orientation.w = 1.0
    return msg


def stamped_pose(q):
    msg = PoseStamped()
    msg.header.frame_id = "world"
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]
    return msg


class AnyGraspRoiAndSelectionTest(unittest.TestCase):
    def test_normalize_roi_clips_to_image_bounds(self):
        clipped = normalize_roi(roi(2, 1, 8, 8), image_width=4, image_height=3)

        self.assertEqual(clipped, (2, 1, 2, 2))

    def test_build_point_cloud_only_uses_roi_pixels(self):
        color = np.full((3, 4, 3), 255, dtype=np.uint8)
        depth = np.full((3, 4), 200, dtype=np.uint16)

        points, colors = build_point_cloud(color, depth, camera_info(), 1000.0, roi(1, 1, 2, 1))

        self.assertEqual(points.shape, (2, 3))
        self.assertEqual(colors.shape, (2, 3))

    def test_build_candidate_markers_colors_safe_and_rejected_grasps(self):
        candidates = PoseArray()
        candidates.header.frame_id = "world"
        candidates.poses = [pose(0.42, 0.0, 0.16), pose(0.90, 0.0, 0.16)]

        markers = build_candidate_markers(candidates, SafetyConfig()).markers

        self.assertEqual(len(markers), 4)
        self.assertGreater(markers[0].color.g, markers[0].color.r)
        self.assertGreater(markers[2].color.r, markers[2].color.g)
        self.assertEqual(markers[1].text, "0")
        self.assertEqual(markers[3].text, "1")

    def test_select_pregrasp_by_index_returns_selected_safe_candidate(self):
        candidates = PoseArray()
        candidates.header.frame_id = "world"
        candidates.poses = [pose(0.42, 0.0, 0.16), pose(0.48, 0.0, 0.18)]

        selected = select_pregrasp_by_index(candidates, 1, SafetyConfig(), 0.10)

        self.assertIsNotNone(selected)
        self.assertIsInstance(selected, PoseStamped)
        self.assertAlmostEqual(selected.pose.position.x, 0.48)
        self.assertAlmostEqual(selected.pose.position.z, 0.28)

    def test_select_pregrasp_by_index_can_use_current_tcp_orientation(self):
        candidates = PoseArray()
        candidates.header.frame_id = "world"
        candidates.poses = [pose(0.42, 0.0, 0.16)]

        selected = select_pregrasp_by_index(
            candidates,
            0,
            SafetyConfig(),
            0.10,
            orientation_source=stamped_pose((0.1, 0.2, 0.3, 0.9)),
        )

        self.assertIsNotNone(selected)
        self.assertAlmostEqual(selected.pose.orientation.x, 0.1)
        self.assertAlmostEqual(selected.pose.orientation.y, 0.2)
        self.assertAlmostEqual(selected.pose.orientation.z, 0.3)
        self.assertAlmostEqual(selected.pose.orientation.w, 0.9)

    def test_apply_tcp_axis_mapping_maps_anygrasp_x_to_tcp_z(self):
        grasp = PoseStamped()
        grasp.header.frame_id = "world"
        grasp.pose = pose(0.42, 0.0, 0.16)

        mapped = apply_tcp_axis_mapping(grasp, "anygrasp_x_to_tcp_z")
        rotation = quaternion_to_matrix(mapped.pose.orientation)

        self.assertAlmostEqual(rotation[0][2], 1.0)
        self.assertAlmostEqual(rotation[1][2], 0.0)
        self.assertAlmostEqual(rotation[2][2], 0.0)
        self.assertAlmostEqual(rotation[0][0], 0.0)
        self.assertAlmostEqual(rotation[1][0], 1.0)
        self.assertAlmostEqual(rotation[2][0], 0.0)

    def test_select_pregrasp_by_index_can_map_grasp_frame_to_tcp_frame(self):
        candidates = PoseArray()
        candidates.header.frame_id = "world"
        candidates.poses = [pose(0.42, 0.0, 0.16)]

        selected = select_pregrasp_by_index(
            candidates,
            0,
            SafetyConfig(),
            0.10,
            tcp_axis_mapping="anygrasp_x_to_tcp_z",
        )
        rotation = quaternion_to_matrix(selected.pose.orientation)

        self.assertIsNotNone(selected)
        self.assertAlmostEqual(rotation[0][2], 1.0)
        self.assertAlmostEqual(rotation[1][0], 1.0)

    def test_select_pregrasp_by_index_offsets_tcp_behind_grasp_center(self):
        candidates = PoseArray()
        candidates.header.frame_id = "world"
        candidates.poses = [pose(0.42, 0.0, 0.16)]

        selected = select_pregrasp_by_index(
            candidates,
            0,
            SafetyConfig(),
            0.10,
            tcp_axis_mapping="anygrasp_x_to_tcp_z",
            tcp_grasp_offset_z=0.12,
        )

        self.assertIsNotNone(selected)
        self.assertAlmostEqual(selected.pose.position.x, 0.30)
        self.assertAlmostEqual(selected.pose.position.y, 0.0)
        self.assertAlmostEqual(selected.pose.position.z, 0.26)

    def test_select_pregrasp_by_index_rejects_unsafe_candidate(self):
        candidates = PoseArray()
        candidates.header.frame_id = "world"
        candidates.poses = [pose(0.90, 0.0, 0.16)]

        selected = select_pregrasp_by_index(candidates, 0, SafetyConfig(), 0.10)

        self.assertIsNone(selected)

    def test_build_selection_snapshot_freezes_candidates_and_selected_pose(self):
        candidates = PoseArray()
        candidates.header.frame_id = "world"
        candidates.poses = [pose(0.42, 0.0, 0.16), pose(0.48, 0.0, 0.18)]
        selected = select_pregrasp_by_index(candidates, 1, SafetyConfig(), 0.10)
        tcp = stamped_pose((0.1, 0.2, 0.3, 0.9))

        snapshot = build_selection_snapshot(candidates, 1, selected, tcp)
        candidates.poses[1].position.x = 0.90

        self.assertEqual(snapshot["selected_index"], 1)
        self.assertEqual(snapshot["frame_id"], "world")
        self.assertAlmostEqual(snapshot["candidates"][1]["position"]["x"], 0.48)
        self.assertAlmostEqual(snapshot["selected_pregrasp"]["position"]["z"], 0.28)
        self.assertAlmostEqual(snapshot["tcp_pose"]["orientation"]["w"], 0.9)

    def test_build_frozen_selection_markers_marks_selected_candidate(self):
        candidates = PoseArray()
        candidates.header.frame_id = "world"
        candidates.poses = [pose(0.42, 0.0, 0.16), pose(0.48, 0.0, 0.18)]

        markers = build_frozen_selection_markers(candidates, selected_index=1).markers

        self.assertEqual(len(markers), 5)
        self.assertEqual(markers[0].action, 3)
        self.assertEqual(markers[2].text, "0")
        self.assertEqual(markers[4].text, "EXEC 1")
        self.assertGreater(markers[3].color.b, markers[3].color.r)
        self.assertGreater(markers[3].scale.x, markers[1].scale.x)

    def test_build_frozen_selection_markers_can_include_diagnostic_axes(self):
        candidates = PoseArray()
        candidates.header.frame_id = "world"
        candidates.poses = [pose(0.42, 0.0, 0.16), pose(0.48, 0.0, 0.18)]

        markers = build_frozen_selection_markers(
            candidates,
            selected_index=1,
            show_diagnostic_axes=True,
        ).markers

        self.assertEqual(len(markers), 11)
        self.assertEqual(markers[6].text, "grasp +X")

    def test_build_pose_axes_markers_draws_local_xyz_axes(self):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "world"
        pose_msg.pose = pose(0.10, 0.20, 0.30)

        markers = build_pose_axes_markers(
            pose_msg,
            namespace="diagnostic",
            id_base=20,
            label_prefix="grasp",
            axis_length=0.05,
        ).markers

        self.assertEqual(len(markers), 6)
        self.assertEqual(markers[0].ns, "diagnostic_axes")
        self.assertEqual(markers[0].id, 20)
        self.assertAlmostEqual(markers[0].points[1].x, 0.15)
        self.assertAlmostEqual(markers[2].points[1].y, 0.25)
        self.assertAlmostEqual(markers[4].points[1].z, 0.35)
        self.assertGreater(markers[0].color.r, markers[0].color.g)
        self.assertGreater(markers[2].color.g, markers[2].color.b)
        self.assertGreater(markers[4].color.b, markers[4].color.r)
        self.assertEqual(markers[1].text, "grasp +X")


if __name__ == "__main__":
    unittest.main()
