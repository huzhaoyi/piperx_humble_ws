#!/usr/bin/env python3

import argparse
from copy import deepcopy

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, RobotState
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


class AnyGraspMoveItPregrasp(Node):
    def __init__(self, args):
        super().__init__("anygrasp_moveit_pregrasp")
        self.args = args
        self.ik_client = self.create_client(GetPositionIK, args.ik_service)
        self.plan_client = self.create_client(GetMotionPlan, args.plan_service)
        self.execute_client = ActionClient(
            self,
            FollowJointTrajectory,
            args.follow_joint_trajectory_action,
        )

    def wait_for_inputs(self):
        self.get_logger().info("waiting for target pose and joint feedback")
        selected = self.get_target_pose()
        joint_state = self.wait_for_message(JointState, self.args.joint_state_topic)
        return selected, joint_state

    def get_target_pose(self):
        if self.args.target_source == "current_tcp":
            target = self.wait_for_message(PoseStamped, self.args.tcp_pose_topic)
            if not target.header.frame_id:
                target.header.frame_id = self.args.base_frame
        else:
            target = self.wait_for_message(PoseStamped, self.args.selected_pregrasp_topic)

        target = deepcopy(target)
        target.pose.position.x += self.args.offset_x
        target.pose.position.y += self.args.offset_y
        target.pose.position.z += self.args.offset_z
        if target.pose.position.z < self.args.min_target_z:
            raise RuntimeError(
                "target z below minimum: %.3f < %.3f"
                % (target.pose.position.z, self.args.min_target_z)
            )
        return target

    def wait_for_message(self, msg_type, topic):
        box = {"msg": None}

        def callback(msg):
            box["msg"] = msg

        sub = self.create_subscription(msg_type, topic, callback, 1)
        while rclpy.ok() and box["msg"] is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        return box["msg"]

    def compute_ik(self, selected: PoseStamped, joint_state: JointState):
        if not self.ik_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"IK service not available: {self.args.ik_service}")

        request = GetPositionIK.Request()
        request.ik_request.group_name = self.args.group_name
        request.ik_request.robot_state.joint_state = self.arm_joint_state(joint_state)
        request.ik_request.avoid_collisions = True
        request.ik_request.ik_link_name = self.args.ik_link_name
        request.ik_request.pose_stamped = selected
        request.ik_request.timeout = Duration(sec=2)

        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.error_code.val != 1:
            raise RuntimeError(f"IK failed: MoveItErrorCodes={response.error_code.val}")
        return response.solution.joint_state

    def plan(self, start_state: JointState, goal_state: JointState):
        if not self.plan_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"Plan service not available: {self.args.plan_service}")

        request = GetMotionPlan.Request()
        request.motion_plan_request = MotionPlanRequest()
        request.motion_plan_request.workspace_parameters.header.frame_id = self.args.base_frame
        request.motion_plan_request.start_state = RobotState()
        request.motion_plan_request.start_state.joint_state = self.arm_joint_state(start_state)
        request.motion_plan_request.goal_constraints = [
            self.goal_constraints_from_joint_state(goal_state)
        ]
        request.motion_plan_request.pipeline_id = self.args.pipeline_id
        request.motion_plan_request.group_name = self.args.group_name
        request.motion_plan_request.num_planning_attempts = self.args.planning_attempts
        request.motion_plan_request.allowed_planning_time = self.args.allowed_planning_time
        request.motion_plan_request.max_velocity_scaling_factor = self.args.velocity_scale
        request.motion_plan_request.max_acceleration_scaling_factor = self.args.acceleration_scale

        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result().motion_plan_response
        if response.error_code.val != 1:
            raise RuntimeError(f"planning failed: MoveItErrorCodes={response.error_code.val}")
        return response.trajectory

    def execute(self, trajectory):
        if not self.execute_client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError(
                f"trajectory action not available: {self.args.follow_joint_trajectory_action}"
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory
        future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("trajectory goal rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"trajectory execution failed: code={result.error_code} {result.error_string}"
            )

    def arm_joint_state(self, joint_state: JointState):
        filtered = JointState()
        filtered.header = joint_state.header
        for name, position in zip(joint_state.name, joint_state.position):
            if name.startswith("joint"):
                filtered.name.append(name)
                filtered.position.append(position)
        return filtered

    def goal_constraints_from_joint_state(self, joint_state: JointState):
        constraints = Constraints()
        for name, position in zip(joint_state.name, joint_state.position):
            if not name.startswith("joint"):
                continue
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = self.args.goal_tolerance
            constraint.tolerance_below = self.args.goal_tolerance
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        return constraints

    def run(self):
        selected, joint_state = self.wait_for_inputs()
        self.get_logger().info(
            "target pose: frame=%s xyz=(%.3f, %.3f, %.3f)"
            % (
                selected.header.frame_id,
                selected.pose.position.x,
                selected.pose.position.y,
                selected.pose.position.z,
            )
        )
        goal_state = self.compute_ik(selected, joint_state)
        trajectory = self.plan(joint_state, goal_state)
        point_count = len(trajectory.joint_trajectory.points)
        self.get_logger().info(f"plan-only success: trajectory points={point_count}")
        if self.args.execute:
            self.get_logger().warn("executing planned pregrasp trajectory")
            self.execute(trajectory)
            self.get_logger().info("execution success")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--selected-pregrasp-topic", default="/anygrasp/selected_pregrasp_base")
    parser.add_argument("--tcp-pose-topic", default="/feedback/tcp_pose")
    parser.add_argument("--joint-state-topic", default="/feedback/joint_states")
    parser.add_argument("--ik-service", default="/compute_ik")
    parser.add_argument("--plan-service", default="/plan_kinematic_path")
    parser.add_argument(
        "--follow-joint-trajectory-action",
        default="/arm_controller/follow_joint_trajectory",
    )
    parser.add_argument("--group-name", default="arm")
    parser.add_argument("--ik-link-name", default="tcp_link")
    parser.add_argument("--base-frame", default="world")
    parser.add_argument("--pipeline-id", default="ompl")
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--allowed-planning-time", type=float, default=5.0)
    parser.add_argument("--velocity-scale", type=float, default=0.1)
    parser.add_argument("--acceleration-scale", type=float, default=0.1)
    parser.add_argument("--goal-tolerance", type=float, default=0.01)
    parser.add_argument(
        "--target-source",
        choices=["selected_pregrasp", "current_tcp"],
        default="selected_pregrasp",
    )
    parser.add_argument("--offset-x", type=float, default=0.0)
    parser.add_argument("--offset-y", type=float, default=0.0)
    parser.add_argument("--offset-z", type=float, default=0.0)
    parser.add_argument("--min-target-z", type=float, default=0.12)
    parser.add_argument("--execute", action="store_true")
    return parser.parse_args()


def main():
    rclpy.init()
    node = AnyGraspMoveItPregrasp(parse_args())
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
