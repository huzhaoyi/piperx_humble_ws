#!/usr/bin/env python3

import argparse

import cv2
import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, RegionOfInterest


def image_to_bgr(msg: Image) -> np.ndarray:
    if msg.encoding == "bgr8":
        channels = 3
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)
        return image.copy()
    if msg.encoding == "rgb8":
        channels = 3
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)
        return image[:, :, ::-1].copy()
    if msg.encoding in ("mono8", "8UC1"):
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"unsupported image encoding: {msg.encoding}")


def roi_from_points(start, end, width: int, height: int) -> RegionOfInterest:
    x0 = max(0, min(start[0], end[0], width - 1))
    y0 = max(0, min(start[1], end[1], height - 1))
    x1 = max(0, min(max(start[0], end[0]), width))
    y1 = max(0, min(max(start[1], end[1]), height))

    roi = RegionOfInterest()
    roi.x_offset = int(x0)
    roi.y_offset = int(y0)
    roi.width = int(max(0, x1 - x0))
    roi.height = int(max(0, y1 - y0))
    roi.do_rectify = False
    return roi


class AnyGraspRoiSelector(Node):
    def __init__(self, args):
        super().__init__("anygrasp_roi_selector")
        self.args = args
        self.window_name = args.window_name
        self.latest_image = None
        self.dragging = False
        self.drag_start = None
        self.drag_end = None
        self.current_roi = None
        self.last_roi_publish_time = self.get_clock().now()

        self.create_subscription(Image, args.image_topic, self.image_callback, 5)
        self.roi_pub = self.create_publisher(RegionOfInterest, args.roi_topic, 5)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        self.timer = self.create_timer(0.03, self.draw_once)
        self.get_logger().info(
            "ROI selector ready: image=%s roi=%s" % (args.image_topic, args.roi_topic)
        )

    def image_callback(self, msg: Image):
        try:
            self.latest_image = image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc))

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.drag_start = (x, y)
            self.drag_end = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            self.drag_end = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.dragging:
            self.dragging = False
            self.drag_end = (x, y)
            if self.latest_image is None:
                return
            height, width = self.latest_image.shape[:2]
            roi = roi_from_points(self.drag_start, self.drag_end, width, height)
            if roi.width > 2 and roi.height > 2:
                self.current_roi = roi
                self.roi_pub.publish(roi)
                self.get_logger().info(
                    "published ROI x=%d y=%d w=%d h=%d"
                    % (roi.x_offset, roi.y_offset, roi.width, roi.height)
                )

    def publish_clear_roi(self):
        roi = RegionOfInterest()
        self.current_roi = None
        self.roi_pub.publish(roi)
        self.get_logger().info("cleared ROI")

    def draw_once(self):
        if self.latest_image is None:
            return

        view = self.latest_image.copy()
        if self.current_roi is not None:
            x0 = self.current_roi.x_offset
            y0 = self.current_roi.y_offset
            x1 = x0 + self.current_roi.width
            y1 = y0 + self.current_roi.height
            cv2.rectangle(view, (x0, y0), (x1, y1), (0, 255, 0), 2)

        if self.dragging and self.drag_start is not None and self.drag_end is not None:
            cv2.rectangle(view, self.drag_start, self.drag_end, (0, 200, 255), 1)

        cv2.imshow(self.window_name, view)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("c"):
            self.publish_clear_roi()
        elif key == ord("q"):
            raise KeyboardInterrupt

        if self.current_roi is not None:
            now = self.get_clock().now()
            if (now - self.last_roi_publish_time).nanoseconds > 1_000_000_000:
                self.roi_pub.publish(self.current_roi)
                self.last_roi_publish_time = now


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image-topic", default="/camera_rotated/color/image_raw")
    parser.add_argument("--roi-topic", default="/anygrasp/roi")
    parser.add_argument("--window-name", default="AnyGrasp ROI Selector")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = AnyGraspRoiSelector(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RCLError:
            pass


if __name__ == "__main__":
    main()
