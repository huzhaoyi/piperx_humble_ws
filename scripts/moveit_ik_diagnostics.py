#!/usr/bin/env python3

import argparse
from copy import deepcopy

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState


class MoveItIkDiagnostics(Node):
    def __init__(self, args):
        super().__init__("moveit_ik_diagnostics")
        self.args = args
        self.ik_client = self.create_client(GetPositionIK, args.ik_service)
        self.fk_client = self.create_client(GetPositionFK, args.fk_service)

    def wait_for_message(self, msg_type, topic):
        box = {"msg": None}

        def callback(msg):
            box["msg"] = msg

        sub = self.create_subscription(msg_type, topic, callback, 1)
        while rclpy.ok() and box["msg"] is None:
            rclpy.spin_once(self, timeout_sec=0.1)
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

    def get_target_pose(self):
        pose = deepcopy(self.wait_for_message(PoseStamped, self.args.tcp_pose_topic))
        if not pose.header.frame_id:
            pose.header.frame_id = self.args.base_frame
        pose.pose.position.x += self.args.offset_x
        pose.pose.position.y += self.args.offset_y
        pose.pose.position.z += self.args.offset_z
        return pose

    def call_ik(self, target, joint_state, avoid_collisions):
        if not self.ik_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"IK service not available: {self.args.ik_service}")

        request = GetPositionIK.Request()
        request.ik_request.group_name = self.args.group_name
        request.ik_request.robot_state.joint_state = self.arm_joint_state(joint_state)
        request.ik_request.avoid_collisions = avoid_collisions
        request.ik_request.ik_link_name = self.args.ik_link_name
        request.ik_request.pose_stamped = target
        request.ik_request.timeout = Duration(sec=self.args.ik_timeout_sec)

        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=self.args.service_timeout,
        )
        if not future.done():
            raise RuntimeError(f"IK service timeout: {self.args.ik_service}")
        response = future.result()
        return response.error_code.val, response.solution.joint_state

    def call_fk(self, joint_state):
        if not self.fk_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"FK service not available: {self.args.fk_service}")

        request = GetPositionFK.Request()
        request.header.frame_id = self.args.base_frame
        request.fk_link_names = [self.args.ik_link_name]
        request.robot_state = RobotState()
        request.robot_state.joint_state = self.arm_joint_state(joint_state)

        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=self.args.service_timeout,
        )
        if not future.done():
            raise RuntimeError(f"FK service timeout: {self.args.fk_service}")
        response = future.result()
        return response.error_code.val, response.pose_stamped

    def run(self):
        target = self.get_target_pose()
        joint_state = self.wait_for_message(JointState, self.args.joint_state_topic)
        arm_state = self.arm_joint_state(joint_state)

        self.get_logger().info(
            "target tcp pose frame=%s xyz=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)"
            % (
                target.header.frame_id,
                target.pose.position.x,
                target.pose.position.y,
                target.pose.position.z,
                target.pose.orientation.x,
                target.pose.orientation.y,
                target.pose.orientation.z,
                target.pose.orientation.w,
            )
        )
        self.get_logger().info(
            "seed joints: %s"
            % ", ".join(
                f"{name}={position:.4f}"
                for name, position in zip(arm_state.name, arm_state.position)
            )
        )

        try:
            fk_code, fk_poses = self.call_fk(joint_state)
            if fk_poses:
                pose = fk_poses[0].pose
                self.get_logger().info(
                    "fk result code=%d xyz=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)"
                    % (
                        fk_code,
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    )
                )
            else:
                self.get_logger().warn("fk result code=%d returned no pose" % fk_code)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))

        for avoid_collisions in (False, True):
            code, solution = self.call_ik(target, joint_state, avoid_collisions)
            self.get_logger().info(
                "ik avoid_collisions=%s code=%d solution_joints=%s"
                % (
                    str(avoid_collisions).lower(),
                    code,
                    ", ".join(
                        f"{name}={position:.4f}"
                        for name, position in zip(solution.name, solution.position)
                        if name.startswith("joint")
                    )
                    or "<none>",
                )
            )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tcp-pose-topic", default="/feedback/tcp_pose")
    parser.add_argument("--joint-state-topic", default="/feedback/joint_states")
    parser.add_argument("--ik-service", default="/compute_ik")
    parser.add_argument("--fk-service", default="/compute_fk")
    parser.add_argument("--group-name", default="arm")
    parser.add_argument("--ik-link-name", default="tcp_link")
    parser.add_argument("--base-frame", default="world")
    parser.add_argument("--ik-timeout-sec", type=int, default=2)
    parser.add_argument("--service-timeout", type=float, default=5.0)
    parser.add_argument("--offset-x", type=float, default=0.0)
    parser.add_argument("--offset-y", type=float, default=0.0)
    parser.add_argument("--offset-z", type=float, default=0.0)
    return parser.parse_args()


def main():
    rclpy.init()
    node = MoveItIkDiagnostics(parse_args())
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
