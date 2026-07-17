#!/usr/bin/env python3

import argparse
from copy import deepcopy

import rclpy
from agx_arm_msgs.msg import AgxArmStatus, GripperStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty


class PiperXGraspPreflight(Node):
    def __init__(self, args):
        super().__init__("piperx_grasp_preflight")
        self.args = args
        self.ik_client = self.create_client(GetPositionIK, args.ik_service)
        self.home_client = self.create_client(Empty, args.home_service)

    def wait_for_message(self, msg_type, topic):
        box = {"msg": None}

        def callback(msg):
            box["msg"] = msg

        sub = self.create_subscription(msg_type, topic, callback, 1)
        deadline = self.get_clock().now().nanoseconds + int(self.args.topic_timeout * 1e9)
        while rclpy.ok() and box["msg"] is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds > deadline:
                self.destroy_subscription(sub)
                raise RuntimeError(f"topic timeout: {topic}")
        self.destroy_subscription(sub)
        return box["msg"]

    def arm_joint_state(self, joint_state):
        filtered = JointState()
        filtered.header = joint_state.header
        for name, position in zip(joint_state.name, joint_state.position):
            if name.startswith("joint"):
                filtered.name.append(name)
                filtered.position.append(position)
        return filtered

    def assert_arm_status_safe(self, status):
        joint_limited = any(status.joint_angle_limit)
        comm_error = any(status.communication_status_joint)
        if status.err_status != 0 or joint_limited or comm_error:
            raise RuntimeError(
                "unsafe arm status: err_status=%d joint_limited=%s comm_error=%s"
                % (status.err_status, str(joint_limited).lower(), str(comm_error).lower())
            )

    def assert_gripper_status_safe(self, status):
        driver_error = (
            status.voltage_too_low
            or status.motor_overheating
            or status.driver_overcurrent
            or status.driver_overheating
            or status.driver_error_status
        )
        if driver_error:
            raise RuntimeError("unsafe gripper driver status")

    def call_ik(self, pose, joint_state, avoid_collisions):
        if not self.ik_client.wait_for_service(timeout_sec=self.args.service_timeout):
            raise RuntimeError(f"IK service not available: {self.args.ik_service}")

        request = GetPositionIK.Request()
        request.ik_request.group_name = self.args.group_name
        request.ik_request.robot_state.joint_state = self.arm_joint_state(joint_state)
        request.ik_request.avoid_collisions = avoid_collisions
        request.ik_request.ik_link_name = self.args.ik_link_name
        request.ik_request.pose_stamped = pose
        request.ik_request.timeout = Duration(sec=self.args.ik_timeout_sec)

        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=self.args.service_timeout,
        )
        if not future.done():
            raise RuntimeError(f"IK service timeout: {self.args.ik_service}")
        return future.result().error_code.val

    def call_move_home(self):
        if not self.home_client.wait_for_service(timeout_sec=self.args.service_timeout):
            raise RuntimeError(f"home service not available: {self.args.home_service}")

        future = self.home_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=self.args.home_timeout,
        )
        if not future.done():
            raise RuntimeError(f"home service timeout: {self.args.home_service}")

    def run(self):
        arm_status = self.wait_for_message(AgxArmStatus, self.args.arm_status_topic)
        gripper_status = self.wait_for_message(
            GripperStatus,
            self.args.gripper_status_topic,
        )
        tcp_pose = self.wait_for_message(PoseStamped, self.args.tcp_pose_topic)
        joint_state = self.wait_for_message(JointState, self.args.joint_state_topic)

        self.assert_arm_status_safe(arm_status)
        self.assert_gripper_status_safe(gripper_status)

        self.get_logger().info(
            "arm safe: err_status=%d motion_status=%d"
            % (arm_status.err_status, arm_status.motion_status)
        )
        self.get_logger().info(
            "tcp pose: frame=%s xyz=(%.4f, %.4f, %.4f)"
            % (
                tcp_pose.header.frame_id,
                tcp_pose.pose.position.x,
                tcp_pose.pose.position.y,
                tcp_pose.pose.position.z,
            )
        )
        self.get_logger().info(
            "gripper: width=%.4f force=%.2f enabled=%s"
            % (
                gripper_status.width,
                gripper_status.force,
                str(gripper_status.driver_enable_status).lower(),
            )
        )

        shifted_pose = deepcopy(tcp_pose)
        shifted_pose.pose.position.z += self.args.ik_probe_offset_z
        exact_code = self.call_ik(tcp_pose, joint_state, avoid_collisions=False)
        shifted_code = self.call_ik(shifted_pose, joint_state, avoid_collisions=True)
        self.get_logger().info("current tcp IK code=%d" % exact_code)
        self.get_logger().info(
            "z-offset tcp IK code=%d offset_z=%.3f"
            % (shifted_code, self.args.ik_probe_offset_z)
        )

        if shifted_code != 1:
            raise RuntimeError("MoveIt IK probe failed for shifted tcp pose")

        if self.args.move_home:
            self.get_logger().warn("calling move_home service")
            self.call_move_home()
            self.get_logger().info("move_home service returned")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm-status-topic", default="/feedback/arm_status")
    parser.add_argument("--gripper-status-topic", default="/feedback/gripper_status")
    parser.add_argument("--tcp-pose-topic", default="/feedback/tcp_pose")
    parser.add_argument("--joint-state-topic", default="/feedback/joint_states")
    parser.add_argument("--ik-service", default="/compute_ik")
    parser.add_argument("--home-service", default="/move_home")
    parser.add_argument("--group-name", default="arm")
    parser.add_argument("--ik-link-name", default="tcp_link")
    parser.add_argument("--topic-timeout", type=float, default=3.0)
    parser.add_argument("--service-timeout", type=float, default=5.0)
    parser.add_argument("--home-timeout", type=float, default=20.0)
    parser.add_argument("--ik-timeout-sec", type=int, default=2)
    parser.add_argument("--ik-probe-offset-z", type=float, default=0.08)
    parser.add_argument("--move-home", action="store_true")
    return parser.parse_args()


def main():
    rclpy.init()
    node = PiperXGraspPreflight(parse_args())
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
