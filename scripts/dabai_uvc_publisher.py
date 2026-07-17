#!/usr/bin/env python3

import argparse
import time

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def load_camera_info(path: str, frame_id: str) -> CameraInfo:
    with open(path, "r", encoding="utf-8") as info_file:
        data = yaml.safe_load(info_file)

    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = int(data["image_width"])
    msg.height = int(data["image_height"])
    msg.distortion_model = data.get("distortion_model", "plumb_bob")
    msg.d = [float(value) for value in data["distortion_coefficients"]["data"]]
    msg.k = [float(value) for value in data["camera_matrix"]["data"]]
    msg.r = [float(value) for value in data["rectification_matrix"]["data"]]
    msg.p = [float(value) for value in data["projection_matrix"]["data"]]
    return msg


def rotate_camera_info_180(info_msg: CameraInfo) -> None:
    width = float(info_msg.width)
    height = float(info_msg.height)

    info_msg.k[2] = width - 1.0 - info_msg.k[2]
    info_msg.k[5] = height - 1.0 - info_msg.k[5]
    info_msg.p[2] = width - 1.0 - info_msg.p[2]
    info_msg.p[6] = height - 1.0 - info_msg.p[6]


class DabaiUvcPublisher(Node):
    def __init__(self, args):
        super().__init__("dabai_uvc_publisher")
        self.frame_id = args.frame_id
        self.rotate_180 = args.rotate_180
        self.bridge = CvBridge()
        self.camera_info = load_camera_info(args.camera_info, self.frame_id)
        if self.rotate_180:
            rotate_camera_info_180(self.camera_info)
        self.image_pub = self.create_publisher(Image, args.image_topic, 10)
        self.info_pub = self.create_publisher(CameraInfo, args.info_topic, 10)

        self.cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera device {args.device}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        self.cap.set(cv2.CAP_PROP_FPS, args.fps)

        period = 1.0 / float(args.fps)
        self.timer = self.create_timer(period, self.publish_frame)
        self.get_logger().info(
            f"Publishing {args.device} at {args.width}x{args.height}@{args.fps} "
            f"to {args.image_topic}, rotate_180={self.rotate_180}"
        )

    def publish_frame(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn("Failed to read camera frame")
            return
        if self.rotate_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)

        stamp = self.get_clock().now().to_msg()
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.frame_id

        info_msg = CameraInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = self.frame_id
        info_msg.width = self.camera_info.width
        info_msg.height = self.camera_info.height
        info_msg.distortion_model = self.camera_info.distortion_model
        info_msg.d = list(self.camera_info.d)
        info_msg.k = list(self.camera_info.k)
        info_msg.r = list(self.camera_info.r)
        info_msg.p = list(self.camera_info.p)

        self.image_pub.publish(image_msg)
        self.info_pub.publish(info_msg)

    def destroy_node(self):
        if hasattr(self, "cap"):
            self.cap.release()
        super().destroy_node()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/video2")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--frame-id", default="dabai_dc1_color_optical_frame")
    parser.add_argument(
        "--camera-info",
        default="/home/joey/piperx_humble_ws/config/dabai_dc1_camera_info.yaml",
    )
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--info-topic", default="/camera/color/camera_info")
    parser.add_argument("--rotate-180", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = DabaiUvcPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
