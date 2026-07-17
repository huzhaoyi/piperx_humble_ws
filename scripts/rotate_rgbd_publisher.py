#!/usr/bin/env python3

import argparse

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


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


class RotateRgbdPublisher(Node):
    def __init__(self, args):
        super().__init__("rotate_rgbd_publisher")
        self.bridge = CvBridge()
        self.color_frame_id = args.color_frame_id
        self.depth_frame_id = args.depth_frame_id

        self.color_pub = self.create_publisher(Image, args.color_out, 10)
        self.depth_pub = self.create_publisher(Image, args.depth_out, 10)
        self.color_info_pub = self.create_publisher(CameraInfo, args.color_info_out, 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, args.depth_info_out, 10)

        self.create_subscription(Image, args.color_in, self.color_callback, 10)
        self.create_subscription(Image, args.depth_in, self.depth_callback, 10)
        self.create_subscription(CameraInfo, args.color_info_in, self.color_info_callback, 10)
        self.create_subscription(CameraInfo, args.depth_info_in, self.depth_info_callback, 10)

        self.get_logger().info(
            f"Publishing rotated RGB-D: {args.color_out}, {args.depth_out}"
        )

    def rotate_image_msg(self, msg: Image, desired_encoding: str) -> Image:
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=desired_encoding)
        rotated = cv2.rotate(image, cv2.ROTATE_180)
        out_msg = self.bridge.cv2_to_imgmsg(rotated, encoding=desired_encoding)
        out_msg.header = msg.header
        return out_msg

    def color_callback(self, msg: Image):
        out_msg = self.rotate_image_msg(msg, "rgb8")
        if self.color_frame_id:
            out_msg.header.frame_id = self.color_frame_id
        self.color_pub.publish(out_msg)

    def depth_callback(self, msg: Image):
        out_msg = self.rotate_image_msg(msg, "16UC1")
        if self.depth_frame_id:
            out_msg.header.frame_id = self.depth_frame_id
        self.depth_pub.publish(out_msg)

    def color_info_callback(self, msg: CameraInfo):
        rotated = rotate_camera_info_180(msg)
        if self.color_frame_id:
            rotated.header.frame_id = self.color_frame_id
        self.color_info_pub.publish(rotated)

    def depth_info_callback(self, msg: CameraInfo):
        rotated = rotate_camera_info_180(msg)
        if self.depth_frame_id:
            rotated.header.frame_id = self.depth_frame_id
        self.depth_info_pub.publish(rotated)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--color-in", default="/camera/color/image_raw")
    parser.add_argument("--depth-in", default="/camera/depth/image_raw")
    parser.add_argument("--color-info-in", default="/camera/color/camera_info")
    parser.add_argument("--depth-info-in", default="/camera/depth/camera_info")
    parser.add_argument("--color-out", default="/camera_rotated/color/image_raw")
    parser.add_argument("--depth-out", default="/camera_rotated/depth/image_raw")
    parser.add_argument("--color-info-out", default="/camera_rotated/color/camera_info")
    parser.add_argument("--depth-info-out", default="/camera_rotated/depth/camera_info")
    parser.add_argument("--color-frame-id", default="camera_color_optical_frame")
    parser.add_argument("--depth-frame-id", default="camera_color_optical_frame")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = RotateRgbdPublisher(args)
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
