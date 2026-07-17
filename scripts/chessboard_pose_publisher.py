#!/usr/bin/env python3

import argparse

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image


def create_object_points(cols: int, rows: int, square_size: float) -> np.ndarray:
    points = np.zeros((rows * cols, 3), dtype=np.float32)
    for row in range(rows):
        for col in range(cols):
            index = row * cols + col
            points[index, 0] = float(col) * square_size
            points[index, 1] = float(row) * square_size
    return points


def camera_info_to_matrices(msg: CameraInfo):
    camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
    dist_coeffs = np.array(msg.d, dtype=np.float64)
    return camera_matrix, dist_coeffs


def rotate_camera_matrix_180(camera_matrix: np.ndarray, width: int, height: int) -> np.ndarray:
    rotated = camera_matrix.copy()
    rotated[0, 2] = float(width) - 1.0 - rotated[0, 2]
    rotated[1, 2] = float(height) - 1.0 - rotated[1, 2]
    return rotated


class ChessboardPosePublisher(Node):
    def __init__(self, args):
        super().__init__("chessboard_pose_publisher")
        self.pattern_size = (args.cols, args.rows)
        self.object_points = create_object_points(args.cols, args.rows, args.square_size)
        self.bridge = CvBridge()
        self.rotate_180 = args.rotate_180
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame = None

        self.pose_pub = self.create_publisher(PoseStamped, args.pose_topic, 10)
        self.create_subscription(CameraInfo, args.info_topic, self.camera_info_callback, 10)
        self.create_subscription(Image, args.image_topic, self.image_callback, 10)
        self.get_logger().info(
            f"Detecting chessboard {args.cols}x{args.rows} inner corners, "
            f"square {args.square_size:.4f} m, rotate_180={self.rotate_180}"
        )

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix, self.dist_coeffs = camera_info_to_matrices(msg)
        if self.rotate_180:
            self.camera_matrix = rotate_camera_matrix_180(
                self.camera_matrix, msg.width, msg.height
            )
        self.camera_frame = msg.header.frame_id

    def image_callback(self, msg: Image):
        if self.camera_matrix is None or self.dist_coeffs is None:
            return

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.rotate_180:
            image = cv2.rotate(image, cv2.ROTATE_180)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, self.pattern_size, flags)
        if not found:
            return

        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )
        refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        ok, rvec, tvec = cv2.solvePnP(
            self.object_points,
            refined,
            self.camera_matrix,
            self.dist_coeffs,
        )
        if not ok:
            self.get_logger().warn("solvePnP failed for detected chessboard")
            return

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quaternion = Rotation.from_matrix(rotation_matrix).as_quat()

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.camera_frame or msg.header.frame_id
        pose.pose.position.x = float(tvec[0, 0])
        pose.pose.position.y = float(tvec[1, 0])
        pose.pose.position.z = float(tvec[2, 0])
        pose.pose.orientation.x = float(quaternion[0])
        pose.pose.orientation.y = float(quaternion[1])
        pose.pose.orientation.z = float(quaternion[2])
        pose.pose.orientation.w = float(quaternion[3])
        self.pose_pub.publish(pose)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cols", type=int, default=8, help="Inner corner columns")
    parser.add_argument("--rows", type=int, default=11, help="Inner corner rows")
    parser.add_argument("--square-size", type=float, default=0.02)
    parser.add_argument("--image-topic", default="/stereo/left/image_rect_color")
    parser.add_argument("--info-topic", default="/stereo/left/camera_info")
    parser.add_argument("--pose-topic", default="/aruco_single/pose")
    parser.add_argument("--rotate-180", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = ChessboardPosePublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
