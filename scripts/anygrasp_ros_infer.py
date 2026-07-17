#!/usr/bin/env python3

import argparse
import json
import os
import sys
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest


def image_to_array(msg: Image) -> np.ndarray:
    if msg.encoding in ("rgb8", "bgr8"):
        channels = 3
        dtype = np.uint8
    elif msg.encoding in ("mono8", "8UC1"):
        channels = 1
        dtype = np.uint8
    elif msg.encoding in ("mono16", "16UC1"):
        channels = 1
        dtype = np.uint16
    elif msg.encoding == "32FC1":
        channels = 1
        dtype = np.float32
    else:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    data = np.frombuffer(msg.data, dtype=dtype)
    if channels == 1:
        return data.reshape((msg.height, msg.width))

    image = data.reshape((msg.height, msg.width, channels))
    if msg.encoding == "bgr8":
        image = image[:, :, ::-1]
    return image


def rotate_camera_info_180(info_msg: CameraInfo) -> CameraInfo:
    rotated = CameraInfo()
    rotated.header = info_msg.header
    rotated.height = info_msg.height
    rotated.width = info_msg.width
    rotated.distortion_model = info_msg.distortion_model
    rotated.d = list(info_msg.d)
    rotated.k = list(info_msg.k)
    rotated.r = list(info_msg.r)
    rotated.p = list(info_msg.p)

    width = float(rotated.width)
    height = float(rotated.height)
    rotated.k[2] = width - 1.0 - rotated.k[2]
    rotated.k[5] = height - 1.0 - rotated.k[5]
    rotated.p[2] = width - 1.0 - rotated.p[2]
    rotated.p[6] = height - 1.0 - rotated.p[6]
    return rotated


def pose_to_matrix(pose_msg: PoseStamped) -> np.ndarray:
    q = pose_msg.pose.orientation
    p = pose_msg.pose.position
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    mat[:3, 3] = [p.x, p.y, p.z]
    return mat


def transform_from_json(path: str) -> np.ndarray:
    with open(path, "r", encoding="utf-8") as result_file:
        data = json.load(result_file)

    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = Rotation.from_quat(data["orientation"]).as_matrix()
    mat[:3, 3] = data["position"]
    return mat


def matrix_to_pose_stamped(mat: np.ndarray, frame_id: str, stamp) -> PoseStamped:
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = frame_id
    pose_msg.header.stamp = stamp

    quat = Rotation.from_matrix(mat[:3, :3]).as_quat()
    pose_msg.pose.position.x = float(mat[0, 3])
    pose_msg.pose.position.y = float(mat[1, 3])
    pose_msg.pose.position.z = float(mat[2, 3])
    pose_msg.pose.orientation.x = float(quat[0])
    pose_msg.pose.orientation.y = float(quat[1])
    pose_msg.pose.orientation.z = float(quat[2])
    pose_msg.pose.orientation.w = float(quat[3])
    return pose_msg


def normalize_roi(roi_msg: RegionOfInterest, image_width: int, image_height: int):
    if roi_msg is None or roi_msg.width == 0 or roi_msg.height == 0:
        return None

    x0 = max(0, min(int(roi_msg.x_offset), image_width))
    y0 = max(0, min(int(roi_msg.y_offset), image_height))
    x1 = max(x0, min(int(roi_msg.x_offset + roi_msg.width), image_width))
    y1 = max(y0, min(int(roi_msg.y_offset + roi_msg.height), image_height))
    width = x1 - x0
    height = y1 - y0
    if width == 0 or height == 0:
        return None
    return x0, y0, width, height


def build_point_cloud(
    color: np.ndarray,
    depth: np.ndarray,
    info: CameraInfo,
    depth_scale: float,
    roi_msg: RegionOfInterest = None,
):
    if color.shape[:2] != depth.shape[:2]:
        raise ValueError(
            f"color/depth shape mismatch: color={color.shape[:2]}, depth={depth.shape[:2]}"
        )

    fx = float(info.k[0])
    fy = float(info.k[4])
    cx = float(info.k[2])
    cy = float(info.k[5])
    if fx == 0.0 or fy == 0.0:
        raise ValueError("invalid camera intrinsics: fx/fy is zero")

    if depth.dtype == np.float32 or depth.dtype == np.float64:
        points_z = depth.astype(np.float32)
    else:
        points_z = depth.astype(np.float32) / float(depth_scale)

    xmap, ymap = np.meshgrid(np.arange(depth.shape[1]), np.arange(depth.shape[0]))
    points_x = (xmap.astype(np.float32) - cx) / fx * points_z
    points_y = (ymap.astype(np.float32) - cy) / fy * points_z

    mask = np.isfinite(points_z) & (points_z > 0.05) & (points_z < 1.2)
    roi = normalize_roi(roi_msg, depth.shape[1], depth.shape[0])
    if roi is not None:
        x0, y0, width, height = roi
        roi_mask = np.zeros_like(mask, dtype=bool)
        roi_mask[y0 : y0 + height, x0 : x0 + width] = True
        mask = mask & roi_mask

    points = np.stack([points_x, points_y, points_z], axis=-1)[mask].astype(np.float32)
    colors = (color.astype(np.float32) / 255.0)[mask].astype(np.float32)
    return points, colors


def matrix_to_pose_array(mats, frame_id: str, stamp) -> PoseArray:
    pose_array = PoseArray()
    pose_array.header.frame_id = frame_id
    pose_array.header.stamp = stamp
    for mat in mats:
        pose_array.poses.append(matrix_to_pose_stamped(mat, frame_id, stamp).pose)
    return pose_array


def grasp_to_matrix(grasp) -> np.ndarray:
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = np.asarray(grasp.rotation_matrix, dtype=np.float64)
    mat[:3, 3] = np.asarray(grasp.translation, dtype=np.float64)
    return mat


class AnyGraspRosInfer(Node):
    def __init__(self, args):
        super().__init__("anygrasp_ros_infer")
        self.args = args
        self.latest_color = None
        self.latest_depth = None
        self.latest_info = None
        self.latest_tcp_pose = None
        self.latest_roi = None
        self.tcp_t_camera = transform_from_json(args.handeye_result)

        sys.path.insert(0, args.anygrasp_detection_dir)
        from gsnet import AnyGrasp

        cfg = SimpleNamespace(
            checkpoint_path=args.checkpoint_path,
            max_gripper_width=args.max_gripper_width,
            gripper_height=args.gripper_height,
            top_down_grasp=args.top_down_grasp,
            debug=False,
        )
        self.anygrasp = AnyGrasp(cfg)
        self.anygrasp.load_net()

        self.create_subscription(Image, args.color_topic, self.color_callback, 5)
        self.create_subscription(Image, args.depth_topic, self.depth_callback, 5)
        self.create_subscription(CameraInfo, args.camera_info_topic, self.info_callback, 5)
        self.create_subscription(PoseStamped, args.tcp_pose_topic, self.tcp_callback, 5)
        self.create_subscription(RegionOfInterest, args.roi_topic, self.roi_callback, 5)

        self.camera_pose_pub = self.create_publisher(PoseStamped, args.camera_grasp_topic, 5)
        self.base_pose_pub = self.create_publisher(PoseStamped, args.base_grasp_topic, 5)
        self.camera_candidates_pub = self.create_publisher(
            PoseArray, args.camera_candidates_topic, 5
        )
        self.base_candidates_pub = self.create_publisher(PoseArray, args.base_candidates_topic, 5)
        self.timer = self.create_timer(args.period, self.infer_once)
        self.busy = False

        self.get_logger().info(
            "AnyGrasp node ready: color=%s depth=%s info=%s"
            % (args.color_topic, args.depth_topic, args.camera_info_topic)
        )

    def color_callback(self, msg):
        self.latest_color = msg

    def depth_callback(self, msg):
        self.latest_depth = msg

    def info_callback(self, msg):
        self.latest_info = msg

    def tcp_callback(self, msg):
        self.latest_tcp_pose = msg

    def roi_callback(self, msg):
        self.latest_roi = msg

    def infer_once(self):
        if self.busy:
            return
        if self.latest_color is None or self.latest_depth is None or self.latest_info is None:
            self.get_logger().warn("waiting for color/depth/camera_info")
            return

        self.busy = True
        try:
            color = image_to_array(self.latest_color)
            depth = image_to_array(self.latest_depth)
            camera_info = self.latest_info
            if self.args.rotate_180:
                color = np.rot90(color, 2)
                depth = np.rot90(depth, 2)
                camera_info = rotate_camera_info_180(self.latest_info)
            points, colors = build_point_cloud(
                color, depth, camera_info, self.args.depth_scale, self.latest_roi
            )
            if points.shape[0] < self.args.min_points:
                self.get_logger().warn(f"not enough valid depth points: {points.shape[0]}")
                return

            lims = [
                self.args.xmin,
                self.args.xmax,
                self.args.ymin,
                self.args.ymax,
                self.args.zmin,
                self.args.zmax,
            ]
            gg, _ = self.anygrasp.get_grasp(
                points,
                colors,
                lims=lims,
                apply_object_mask=True,
                dense_grasp=False,
                collision_detection=True,
            )
            if gg is None or len(gg) == 0:
                self.get_logger().warn("no grasp detected")
                return

            sorted_gg = gg.nms().sort_by_score()
            top_count = min(len(sorted_gg), self.args.top_n)
            camera_t_grasps = [grasp_to_matrix(sorted_gg[index]) for index in range(top_count)]
            best = sorted_gg[0]
            camera_t_grasp = camera_t_grasps[0]

            stamp = self.latest_color.header.stamp
            camera_frame = self.latest_color.header.frame_id or self.args.camera_frame
            self.camera_pose_pub.publish(
                matrix_to_pose_stamped(camera_t_grasp, camera_frame, stamp)
            )
            self.camera_candidates_pub.publish(
                matrix_to_pose_array(camera_t_grasps, camera_frame, stamp)
            )

            if self.latest_tcp_pose is not None:
                base_t_tcp = pose_to_matrix(self.latest_tcp_pose)
                base_t_grasps = [
                    base_t_tcp @ self.tcp_t_camera @ camera_t_grasp
                    for camera_t_grasp in camera_t_grasps
                ]
                base_t_grasp = base_t_grasps[0]
                self.base_pose_pub.publish(
                    matrix_to_pose_stamped(base_t_grasp, self.args.base_frame, stamp)
                )
                self.base_candidates_pub.publish(
                    matrix_to_pose_array(base_t_grasps, self.args.base_frame, stamp)
                )

            self.get_logger().info(
                "best grasp score=%.3f width=%.3f points=%d"
                % (float(best.score), float(best.width), points.shape[0])
            )
        except Exception as exc:
            self.get_logger().error(f"AnyGrasp inference failed: {exc}")
        finally:
            self.busy = False


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--anygrasp-detection-dir", default="/home/joey/anygrasp_sdk/grasp_detection")
    parser.add_argument(
        "--checkpoint-path",
        default="/home/joey/anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar",
    )
    parser.add_argument(
        "--handeye-result",
        default="/home/joey/piperx_humble_ws/calibration_results/2026-07-15_15-21-25_calibration.json",
    )
    parser.add_argument("--color-topic", default="/stereo/left/image_rect_color")
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--camera-info-topic", default="/stereo/left/camera_info")
    parser.add_argument("--tcp-pose-topic", default="/feedback/tcp_pose")
    parser.add_argument("--camera-grasp-topic", default="/anygrasp/best_grasp_camera")
    parser.add_argument("--base-grasp-topic", default="/anygrasp/best_grasp_base")
    parser.add_argument("--camera-candidates-topic", default="/anygrasp/grasp_candidates_camera")
    parser.add_argument("--base-candidates-topic", default="/anygrasp/grasp_candidates_base")
    parser.add_argument("--roi-topic", default="/anygrasp/roi")
    parser.add_argument("--camera-frame", default="dabai_dc1_color_optical_frame")
    parser.add_argument("--base-frame", default="world")
    parser.add_argument("--depth-scale", type=float, default=1000.0)
    parser.add_argument("--max-gripper-width", type=float, default=0.08)
    parser.add_argument("--gripper-height", type=float, default=0.03)
    parser.add_argument("--top-down-grasp", action="store_true")
    parser.add_argument("--rotate-180", action="store_true")
    parser.add_argument("--period", type=float, default=2.0)
    parser.add_argument("--top-n", type=int, default=5)
    parser.add_argument("--min-points", type=int, default=1000)
    parser.add_argument("--xmin", type=float, default=-0.35)
    parser.add_argument("--xmax", type=float, default=0.35)
    parser.add_argument("--ymin", type=float, default=-0.30)
    parser.add_argument("--ymax", type=float, default=0.30)
    parser.add_argument("--zmin", type=float, default=0.05)
    parser.add_argument("--zmax", type=float, default=0.80)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = AnyGraspRosInfer(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RCLError:
            pass


if __name__ == "__main__":
    main()
