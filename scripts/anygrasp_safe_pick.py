#!/usr/bin/env python3

import argparse
import json
import math
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseArray, PoseStamped
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Int32, String
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class SafetyConfig:
    base_frame: str = "world"
    min_x: float = 0.12
    max_x: float = 0.62
    min_y: float = -0.30
    max_y: float = 0.30
    min_z: float = 0.06
    max_z: float = 0.45
    min_radius: float = 0.14
    max_radius: float = 0.68
    table_z: float = 0.0
    min_table_clearance: float = 0.06
    min_quaternion_norm: float = 0.90
    max_quaternion_norm: float = 1.10


@dataclass(frozen=True)
class SafetyDecision:
    accepted: bool
    reason: str


def piperx_grasp_workspace_config(base_frame: str = "world") -> SafetyConfig:
    # Conservative envelope from PiperX URDF arm segments:
    # 0.28503m + 0.27364m + 0.07466m, leaving margin for wrist attitude,
    # gripper length, table clearance, and MoveIt IK/collision validation.
    return SafetyConfig(base_frame=base_frame)


def clamp(value: float, min_value: float, max_value: float) -> float:
    return min(max(value, min_value), max_value)


def _all_finite(values) -> bool:
    return all(math.isfinite(value) for value in values)


def validate_grasp_pose(pose_msg: PoseStamped, cfg: SafetyConfig) -> SafetyDecision:
    frame_id = pose_msg.header.frame_id
    if frame_id and frame_id != cfg.base_frame:
        return SafetyDecision(False, f"frame mismatch: {frame_id} != {cfg.base_frame}")

    p = pose_msg.pose.position
    q = pose_msg.pose.orientation
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    if not _all_finite(values):
        return SafetyDecision(False, "pose contains non-finite value")

    quat_norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if quat_norm < cfg.min_quaternion_norm or quat_norm > cfg.max_quaternion_norm:
        return SafetyDecision(False, f"invalid orientation quaternion norm: {quat_norm:.3f}")

    if p.x < cfg.min_x or p.x > cfg.max_x:
        return SafetyDecision(False, f"outside workspace x: {p.x:.3f}")
    if p.y < cfg.min_y or p.y > cfg.max_y:
        return SafetyDecision(False, f"outside workspace y: {p.y:.3f}")

    min_safe_z = cfg.table_z + cfg.min_table_clearance
    if p.z < min_safe_z:
        return SafetyDecision(False, f"below table clearance: z={p.z:.3f}, min={min_safe_z:.3f}")

    if p.z < cfg.min_z or p.z > cfg.max_z:
        return SafetyDecision(False, f"outside workspace z: {p.z:.3f}")

    radius = math.hypot(p.x, p.y)
    if radius < cfg.min_radius or radius > cfg.max_radius:
        return SafetyDecision(False, f"outside radial reach: {radius:.3f}")

    return SafetyDecision(True, "accepted")


def validate_candidate_center_pose(pose_msg: PoseStamped, cfg: SafetyConfig) -> SafetyDecision:
    frame_id = pose_msg.header.frame_id
    if frame_id and frame_id != cfg.base_frame:
        return SafetyDecision(False, f"frame mismatch: {frame_id} != {cfg.base_frame}")

    p = pose_msg.pose.position
    q = pose_msg.pose.orientation
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    if not _all_finite(values):
        return SafetyDecision(False, "pose contains non-finite value")

    quat_norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if quat_norm < cfg.min_quaternion_norm or quat_norm > cfg.max_quaternion_norm:
        return SafetyDecision(False, f"invalid orientation quaternion norm: {quat_norm:.3f}")

    if p.x < cfg.min_x or p.x > cfg.max_x:
        return SafetyDecision(False, f"outside workspace x: {p.x:.3f}")
    if p.y < cfg.min_y or p.y > cfg.max_y:
        return SafetyDecision(False, f"outside workspace y: {p.y:.3f}")
    if p.z > cfg.max_z:
        return SafetyDecision(False, f"outside workspace z: {p.z:.3f}")

    radius = math.hypot(p.x, p.y)
    if radius < cfg.min_radius or radius > cfg.max_radius:
        return SafetyDecision(False, f"outside radial reach: {radius:.3f}")

    return SafetyDecision(True, "accepted")


def build_pregrasp_pose(
    grasp_msg: PoseStamped,
    vertical_standoff: float,
    orientation_source: PoseStamped = None,
    tcp_axis_mapping: str = "none",
    tcp_grasp_offset_z: float = 0.0,
) -> PoseStamped:
    pregrasp = PoseStamped()
    pregrasp.header = grasp_msg.header
    pregrasp.pose.position.x = grasp_msg.pose.position.x
    pregrasp.pose.position.y = grasp_msg.pose.position.y
    pregrasp.pose.position.z = grasp_msg.pose.position.z + vertical_standoff
    if orientation_source is None:
        pregrasp.pose.orientation = grasp_msg.pose.orientation
    else:
        pregrasp.pose.orientation = orientation_source.pose.orientation
    pregrasp = apply_tcp_axis_mapping(pregrasp, tcp_axis_mapping)
    apply_tcp_grasp_offset(pregrasp, tcp_grasp_offset_z)
    return pregrasp


def pose_array_item_to_stamped(candidates: PoseArray, index: int) -> PoseStamped:
    pose_msg = PoseStamped()
    pose_msg.header = candidates.header
    pose_msg.pose = candidates.poses[index]
    return pose_msg


def select_pregrasp_by_index(
    candidates: PoseArray,
    index: int,
    cfg: SafetyConfig,
    vertical_standoff: float,
    orientation_source: PoseStamped = None,
    tcp_axis_mapping: str = "none",
    tcp_grasp_offset_z: float = 0.0,
):
    if index < 0 or index >= len(candidates.poses):
        return None

    grasp = pose_array_item_to_stamped(candidates, index)
    if not validate_candidate_center_pose(grasp, cfg).accepted:
        return None

    pregrasp = build_pregrasp_pose(
        grasp,
        vertical_standoff,
        orientation_source,
        tcp_axis_mapping,
        tcp_grasp_offset_z,
    )
    if not validate_grasp_pose(pregrasp, cfg).accepted:
        return None

    return pregrasp


def build_candidate_markers(candidates: PoseArray, cfg: SafetyConfig) -> MarkerArray:
    marker_array = MarkerArray()
    for index, pose in enumerate(candidates.poses):
        stamped = pose_array_item_to_stamped(candidates, index)
        accepted = validate_candidate_center_pose(stamped, cfg).accepted

        arrow = Marker()
        arrow.header = candidates.header
        arrow.ns = "anygrasp_candidates"
        arrow.id = index * 2
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = pose
        arrow.scale.x = 0.08
        arrow.scale.y = 0.012
        arrow.scale.z = 0.012
        arrow.color.a = 0.85
        arrow.color.r = 0.10 if accepted else 0.90
        arrow.color.g = 0.80 if accepted else 0.10
        arrow.color.b = 0.10
        marker_array.markers.append(arrow)

        text = Marker()
        text.header = candidates.header
        text.ns = "anygrasp_candidate_labels"
        text.id = index * 2 + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose = pose
        text.pose.position.z += 0.04
        text.scale.z = 0.035
        text.color.a = 0.95
        text.color.r = arrow.color.r
        text.color.g = arrow.color.g
        text.color.b = arrow.color.b
        text.text = str(index)
        marker_array.markers.append(text)

    return marker_array


def pose_to_dict(pose_msg) -> dict:
    return {
        "position": {
            "x": float(pose_msg.position.x),
            "y": float(pose_msg.position.y),
            "z": float(pose_msg.position.z),
        },
        "orientation": {
            "x": float(pose_msg.orientation.x),
            "y": float(pose_msg.orientation.y),
            "z": float(pose_msg.orientation.z),
            "w": float(pose_msg.orientation.w),
        },
    }


def build_selection_snapshot(
    candidates: PoseArray,
    selected_index: int,
    selected_pregrasp: PoseStamped,
    tcp_pose: PoseStamped = None,
) -> dict:
    frozen_candidates = deepcopy(candidates)
    snapshot = {
        "frame_id": frozen_candidates.header.frame_id,
        "stamp": {
            "sec": int(frozen_candidates.header.stamp.sec),
            "nanosec": int(frozen_candidates.header.stamp.nanosec),
        },
        "selected_index": int(selected_index),
        "candidates": [pose_to_dict(pose) for pose in frozen_candidates.poses],
        "selected_pregrasp": pose_to_dict(selected_pregrasp.pose),
    }
    if 0 <= selected_index < len(frozen_candidates.poses):
        snapshot["selected_grasp"] = pose_to_dict(frozen_candidates.poses[selected_index])
    if tcp_pose is not None:
        snapshot["tcp_pose"] = pose_to_dict(tcp_pose.pose)
    return snapshot


def quaternion_to_matrix(q) -> list:
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]

    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]


def matrix_to_quaternion(matrix: list):
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2][1] - matrix[1][2]) / scale
        y = (matrix[0][2] - matrix[2][0]) / scale
        z = (matrix[1][0] - matrix[0][1]) / scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        w = (matrix[2][1] - matrix[1][2]) / scale
        x = 0.25 * scale
        y = (matrix[0][1] + matrix[1][0]) / scale
        z = (matrix[0][2] + matrix[2][0]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        w = (matrix[0][2] - matrix[2][0]) / scale
        x = (matrix[0][1] + matrix[1][0]) / scale
        y = 0.25 * scale
        z = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        w = (matrix[1][0] - matrix[0][1]) / scale
        x = (matrix[0][2] + matrix[2][0]) / scale
        y = (matrix[1][2] + matrix[2][1]) / scale
        z = 0.25 * scale

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    q = type("QuaternionValue", (), {})()
    q.x = x / norm
    q.y = y / norm
    q.z = z / norm
    q.w = w / norm
    return q


def multiply_matrix(a: list, b: list) -> list:
    return [
        [
            a[row][0] * b[0][col]
            + a[row][1] * b[1][col]
            + a[row][2] * b[2][col]
            for col in range(3)
        ]
        for row in range(3)
    ]


def apply_tcp_axis_mapping(pose_msg: PoseStamped, mapping: str) -> PoseStamped:
    mapped = deepcopy(pose_msg)
    if mapping == "none":
        return mapped
    if mapping != "anygrasp_x_to_tcp_z":
        raise ValueError(f"unsupported tcp axis mapping: {mapping}")

    # Columns define tcp axes in the AnyGrasp frame:
    # tcp +X = grasp +Y, tcp +Y = grasp +Z, tcp +Z = grasp +X.
    grasp_r_tcp = [
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ]
    base_r_grasp = quaternion_to_matrix(pose_msg.pose.orientation)
    base_r_tcp = multiply_matrix(base_r_grasp, grasp_r_tcp)
    q = matrix_to_quaternion(base_r_tcp)
    mapped.pose.orientation.x = q.x
    mapped.pose.orientation.y = q.y
    mapped.pose.orientation.z = q.z
    mapped.pose.orientation.w = q.w
    return mapped


def apply_tcp_grasp_offset(pose_msg: PoseStamped, tcp_grasp_offset_z: float) -> None:
    if tcp_grasp_offset_z == 0.0:
        return
    rotation = quaternion_to_matrix(pose_msg.pose.orientation)
    tcp_z = [rotation[0][2], rotation[1][2], rotation[2][2]]
    pose_msg.pose.position.x -= tcp_z[0] * tcp_grasp_offset_z
    pose_msg.pose.position.y -= tcp_z[1] * tcp_grasp_offset_z
    pose_msg.pose.position.z -= tcp_z[2] * tcp_grasp_offset_z


def _point(x: float, y: float, z: float) -> Point:
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def build_pose_axes_markers(
    pose_msg: PoseStamped,
    namespace: str,
    id_base: int,
    label_prefix: str,
    axis_length: float = 0.08,
) -> MarkerArray:
    marker_array = MarkerArray()
    p = pose_msg.pose.position
    rotation = quaternion_to_matrix(pose_msg.pose.orientation)
    axes = [
        ("X", (1.0, 0.05, 0.05)),
        ("Y", (0.05, 0.85, 0.05)),
        ("Z", (0.05, 0.20, 1.0)),
    ]

    for axis_index, (axis_name, color) in enumerate(axes):
        direction = [
            rotation[0][axis_index],
            rotation[1][axis_index],
            rotation[2][axis_index],
        ]
        end = _point(
            p.x + direction[0] * axis_length,
            p.y + direction[1] * axis_length,
            p.z + direction[2] * axis_length,
        )

        arrow = Marker()
        arrow.header = pose_msg.header
        arrow.ns = f"{namespace}_axes"
        arrow.id = id_base + axis_index * 2
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.points = [_point(p.x, p.y, p.z), end]
        arrow.scale.x = 0.008
        arrow.scale.y = 0.018
        arrow.scale.z = 0.018
        arrow.color.a = 0.95
        arrow.color.r = color[0]
        arrow.color.g = color[1]
        arrow.color.b = color[2]
        marker_array.markers.append(arrow)

        text = Marker()
        text.header = pose_msg.header
        text.ns = f"{namespace}_axis_labels"
        text.id = id_base + axis_index * 2 + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position = _point(end.x, end.y, end.z)
        text.pose.position.z += 0.015
        text.pose.orientation.w = 1.0
        text.scale.z = 0.025
        text.color.a = 0.95
        text.color.r = color[0]
        text.color.g = color[1]
        text.color.b = color[2]
        text.text = f"{label_prefix} +{axis_name}"
        marker_array.markers.append(text)

    return marker_array


def build_frozen_selection_markers(
    candidates: PoseArray,
    selected_index: int,
    selected_pregrasp: PoseStamped = None,
    current_tcp_pose: PoseStamped = None,
    show_diagnostic_axes: bool = False,
) -> MarkerArray:
    marker_array = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    marker_array.markers.append(clear)

    frozen_candidates = deepcopy(candidates)
    for index, pose in enumerate(frozen_candidates.poses):
        selected = index == selected_index

        arrow = Marker()
        arrow.header = frozen_candidates.header
        arrow.ns = "anygrasp_frozen_candidates"
        arrow.id = index * 2
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = pose
        arrow.scale.x = 0.11 if selected else 0.07
        arrow.scale.y = 0.018 if selected else 0.010
        arrow.scale.z = 0.018 if selected else 0.010
        arrow.color.a = 0.95 if selected else 0.45
        arrow.color.r = 0.10 if selected else 0.55
        arrow.color.g = 0.35 if selected else 0.55
        arrow.color.b = 1.00 if selected else 0.55
        marker_array.markers.append(arrow)

        text = Marker()
        text.header = frozen_candidates.header
        text.ns = "anygrasp_frozen_labels"
        text.id = index * 2 + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose = pose
        text.pose.position.z += 0.025 if selected else 0.020
        text.scale.z = 0.035 if selected else 0.025
        text.color.a = arrow.color.a
        text.color.r = arrow.color.r
        text.color.g = arrow.color.g
        text.color.b = arrow.color.b
        text.text = f"EXEC {index}" if selected else str(index)
        marker_array.markers.append(text)

    if show_diagnostic_axes and 0 <= selected_index < len(frozen_candidates.poses):
        selected_grasp = pose_array_item_to_stamped(frozen_candidates, selected_index)
        marker_array.markers.extend(
            build_pose_axes_markers(
                selected_grasp,
                "anygrasp_frozen_grasp",
                1000,
                "grasp",
            ).markers
        )

    if show_diagnostic_axes and selected_pregrasp is not None:
        marker_array.markers.extend(
            build_pose_axes_markers(
                selected_pregrasp,
                "anygrasp_frozen_pregrasp",
                1100,
                "pregrasp",
            ).markers
        )

    if show_diagnostic_axes and current_tcp_pose is not None:
        marker_array.markers.extend(
            build_pose_axes_markers(
                current_tcp_pose,
                "anygrasp_current_tcp",
                1200,
                "tcp",
            ).markers
        )

    return marker_array


class AnyGraspSafePick(Node):
    def __init__(self, args):
        super().__init__("anygrasp_safe_pick")
        self.args = args
        base_cfg = piperx_grasp_workspace_config(args.base_frame)
        self.cfg = SafetyConfig(
            base_frame=base_cfg.base_frame,
            min_x=args.min_x if args.min_x is not None else base_cfg.min_x,
            max_x=args.max_x if args.max_x is not None else base_cfg.max_x,
            min_y=args.min_y if args.min_y is not None else base_cfg.min_y,
            max_y=args.max_y if args.max_y is not None else base_cfg.max_y,
            min_z=args.min_z if args.min_z is not None else base_cfg.min_z,
            max_z=args.max_z if args.max_z is not None else base_cfg.max_z,
            min_radius=args.min_radius if args.min_radius is not None else base_cfg.min_radius,
            max_radius=args.max_radius if args.max_radius is not None else base_cfg.max_radius,
            table_z=args.table_z,
            min_table_clearance=args.min_table_clearance,
        )
        self.vertical_standoff = clamp(
            args.vertical_standoff,
            args.min_vertical_standoff,
            args.max_vertical_standoff,
        )
        self.latest_candidates = None
        self.latest_tcp_pose = None
        self.selected_pregrasp = None
        self.frozen_candidates = None
        self.frozen_selected_index = None

        self.create_subscription(PoseStamped, args.grasp_topic, self.grasp_callback, 10)
        self.create_subscription(PoseStamped, args.tcp_pose_topic, self.tcp_callback, 10)
        self.create_subscription(PoseArray, args.candidates_topic, self.candidates_callback, 10)
        self.create_subscription(Int32, args.select_topic, self.select_callback, 10)
        self.safe_grasp_pub = self.create_publisher(PoseStamped, args.safe_grasp_topic, 10)
        self.pregrasp_pub = self.create_publisher(PoseStamped, args.pregrasp_topic, 10)
        self.selected_pregrasp_pub = self.create_publisher(
            PoseStamped, args.selected_pregrasp_topic, 10
        )
        self.marker_pub = self.create_publisher(MarkerArray, args.marker_topic, 10)
        self.frozen_marker_pub = self.create_publisher(
            MarkerArray, args.frozen_marker_topic, 10
        )
        self.status_pub = self.create_publisher(String, args.status_topic, 10)
        self.selected_timer = self.create_timer(0.5, self.publish_selected_pregrasp)
        self.frozen_marker_timer = self.create_timer(1.0, self.publish_frozen_markers)

        self.get_logger().info(
            "AnyGrasp safety gate ready: input=%s safe=%s pregrasp=%s workspace=x[%.2f, %.2f] y[%.2f, %.2f] z[%.2f, %.2f]"
            % (
                args.grasp_topic,
                args.safe_grasp_topic,
                args.pregrasp_topic,
                self.cfg.min_x,
                self.cfg.max_x,
                self.cfg.min_y,
                self.cfg.max_y,
                self.cfg.min_z,
                self.cfg.max_z,
            )
        )

    def candidates_callback(self, msg: PoseArray):
        self.latest_candidates = msg
        self.marker_pub.publish(build_candidate_markers(msg, self.cfg))

    def tcp_callback(self, msg: PoseStamped):
        self.latest_tcp_pose = msg

    def select_callback(self, msg: Int32):
        if self.latest_candidates is None:
            self.publish_status(False, "no candidates available")
            return

        selected = select_pregrasp_by_index(
            self.latest_candidates,
            int(msg.data),
            self.cfg,
            self.vertical_standoff,
            orientation_source=self.latest_tcp_pose
            if self.args.pregrasp_orientation_mode == "current_tcp"
            else None,
            tcp_axis_mapping=self.args.tcp_axis_mapping
            if self.args.pregrasp_orientation_mode == "grasp"
            else "none",
            tcp_grasp_offset_z=self.args.tcp_grasp_offset_z
            if self.args.pregrasp_orientation_mode == "grasp"
            else 0.0,
        )
        if selected is None:
            reason = f"candidate {int(msg.data)} unavailable or unsafe"
            self.get_logger().warn(reason)
            self.publish_status(False, reason)
            return

        self.selected_pregrasp_pub.publish(selected)
        self.selected_pregrasp = selected
        self.frozen_candidates = deepcopy(self.latest_candidates)
        self.frozen_selected_index = int(msg.data)
        self.publish_frozen_markers()
        snapshot_path = self.save_selection_snapshot(
            self.frozen_candidates,
            self.frozen_selected_index,
            selected,
        )
        self.publish_status(True, f"selected candidate {int(msg.data)}")
        self.get_logger().info(
            "frozen candidate %d saved to %s"
            % (self.frozen_selected_index, snapshot_path)
        )

    def publish_selected_pregrasp(self):
        if self.selected_pregrasp is not None:
            self.selected_pregrasp_pub.publish(self.selected_pregrasp)

    def publish_frozen_markers(self):
        if self.frozen_candidates is None or self.frozen_selected_index is None:
            return
        self.frozen_marker_pub.publish(
            build_frozen_selection_markers(
                self.frozen_candidates,
                self.frozen_selected_index,
                self.selected_pregrasp,
                self.latest_tcp_pose,
                self.args.show_diagnostic_axes,
            )
        )

    def save_selection_snapshot(
        self,
        candidates: PoseArray,
        selected_index: int,
        selected_pregrasp: PoseStamped,
    ) -> str:
        log_dir = Path(self.args.selection_log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        now = self.get_clock().now().to_msg()
        filename = "anygrasp_selection_%d_%09d.json" % (now.sec, now.nanosec)
        path = log_dir / filename
        snapshot = build_selection_snapshot(
            candidates,
            selected_index,
            selected_pregrasp,
            self.latest_tcp_pose,
        )
        path.write_text(json.dumps(snapshot, indent=2, sort_keys=True), encoding="utf-8")
        return str(path)

    def publish_status(self, accepted: bool, reason: str):
        msg = String()
        msg.data = ("ACCEPT " if accepted else "REJECT ") + reason
        self.status_pub.publish(msg)

    def grasp_callback(self, msg: PoseStamped):
        grasp_decision = validate_candidate_center_pose(msg, self.cfg)
        if not grasp_decision.accepted:
            self.get_logger().warn(grasp_decision.reason)
            self.publish_status(False, grasp_decision.reason)
            return

        pregrasp = build_pregrasp_pose(
            msg,
            self.vertical_standoff,
            tcp_axis_mapping=self.args.tcp_axis_mapping,
            tcp_grasp_offset_z=self.args.tcp_grasp_offset_z,
        )
        pregrasp_decision = validate_grasp_pose(pregrasp, self.cfg)
        if not pregrasp_decision.accepted:
            reason = "pregrasp rejected: " + pregrasp_decision.reason
            self.get_logger().warn(reason)
            self.publish_status(False, reason)
            return

        self.safe_grasp_pub.publish(msg)
        self.pregrasp_pub.publish(pregrasp)
        self.publish_status(True, grasp_decision.reason)
        self.get_logger().info(
            "safe grasp accepted: grasp=(%.3f, %.3f, %.3f), pregrasp_z=%.3f"
            % (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                pregrasp.pose.position.z,
            )
        )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--grasp-topic", default="/anygrasp/best_grasp_base")
    parser.add_argument("--tcp-pose-topic", default="/feedback/tcp_pose")
    parser.add_argument("--candidates-topic", default="/anygrasp/grasp_candidates_base")
    parser.add_argument("--select-topic", default="/anygrasp/select_index")
    parser.add_argument("--safe-grasp-topic", default="/anygrasp/safe_grasp_base")
    parser.add_argument("--pregrasp-topic", default="/anygrasp/pregrasp_base")
    parser.add_argument("--selected-pregrasp-topic", default="/anygrasp/selected_pregrasp_base")
    parser.add_argument("--marker-topic", default="/anygrasp/grasp_markers")
    parser.add_argument("--frozen-marker-topic", default="/anygrasp/frozen_grasp_markers")
    parser.add_argument("--status-topic", default="/anygrasp/safety_status")
    parser.add_argument("--selection-log-dir", default="logs/anygrasp_runs")
    parser.add_argument("--show-diagnostic-axes", action="store_true")
    parser.add_argument(
        "--pregrasp-orientation-mode",
        choices=["current_tcp", "grasp"],
        default="current_tcp",
    )
    parser.add_argument(
        "--tcp-axis-mapping",
        choices=["none", "anygrasp_x_to_tcp_z"],
        default="anygrasp_x_to_tcp_z",
    )
    parser.add_argument("--tcp-grasp-offset-z", type=float, default=0.12)
    parser.add_argument("--base-frame", default="world")
    parser.add_argument("--min-x", type=float, default=None)
    parser.add_argument("--max-x", type=float, default=None)
    parser.add_argument("--min-y", type=float, default=None)
    parser.add_argument("--max-y", type=float, default=None)
    parser.add_argument("--min-z", type=float, default=None)
    parser.add_argument("--max-z", type=float, default=None)
    parser.add_argument("--min-radius", type=float, default=None)
    parser.add_argument("--max-radius", type=float, default=None)
    parser.add_argument("--table-z", type=float, default=0.0)
    parser.add_argument("--min-table-clearance", type=float, default=0.06)
    parser.add_argument("--vertical-standoff", type=float, default=0.10)
    parser.add_argument("--min-vertical-standoff", type=float, default=0.04)
    parser.add_argument("--max-vertical-standoff", type=float, default=0.18)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = AnyGraspSafePick(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RCLError:
            pass


if __name__ == "__main__":
    main()
