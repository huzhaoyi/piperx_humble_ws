#!/usr/bin/env python3

import argparse
import json
import math
import sys
from copy import deepcopy
from dataclasses import dataclass, asdict
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseArray, PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, RobotState
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.anygrasp_safe_pick import (
    SafetyConfig,
    apply_tcp_axis_mapping,
    apply_tcp_grasp_offset,
    pose_array_item_to_stamped,
    pose_to_dict,
    quaternion_to_matrix,
    validate_candidate_center_pose,
    validate_grasp_pose,
)


@dataclass
class CandidateReport:
    index: int
    accepted: bool
    reason: str
    tcp_grasp_offset_z: float = 0.0
    pregrasp_points: int = 0
    approach_steps: int = 0
    lift_steps: int = 0


def parse_float_list(value: str) -> list:
    if not value:
        return []
    return [float(item.strip()) for item in value.split(",") if item.strip()]


def offset_pose_along_local_z(pose_msg: PoseStamped, distance: float) -> PoseStamped:
    result = deepcopy(pose_msg)
    rotation = quaternion_to_matrix(result.pose.orientation)
    tcp_z = [rotation[0][2], rotation[1][2], rotation[2][2]]
    result.pose.position.x += tcp_z[0] * distance
    result.pose.position.y += tcp_z[1] * distance
    result.pose.position.z += tcp_z[2] * distance
    return result


def build_world_lift_pose(pose_msg: PoseStamped, distance: float) -> PoseStamped:
    result = deepcopy(pose_msg)
    result.pose.position.z += distance
    return result


def is_empty_grasp(gripper_status, empty_width_threshold: float, contact_force_threshold: float) -> bool:
    return (
        abs(float(gripper_status.width)) <= empty_width_threshold
        and abs(float(gripper_status.force)) < contact_force_threshold
    )


def build_linear_waypoints(
    start: PoseStamped,
    goal: PoseStamped,
    step: float,
) -> list:
    dx = goal.pose.position.x - start.pose.position.x
    dy = goal.pose.position.y - start.pose.position.y
    dz = goal.pose.position.z - start.pose.position.z
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)
    if distance == 0.0:
        return [deepcopy(goal)]

    direction = [dx / distance, dy / distance, dz / distance]
    full_steps = int(distance // step)
    waypoints = []
    for index in range(1, full_steps + 1):
        travelled = min(distance, index * step)
        waypoint = deepcopy(goal)
        waypoint.pose.position.x = start.pose.position.x + direction[0] * travelled
        waypoint.pose.position.y = start.pose.position.y + direction[1] * travelled
        waypoint.pose.position.z = start.pose.position.z + direction[2] * travelled
        waypoint.pose.orientation = goal.pose.orientation
        waypoints.append(waypoint)
    if not waypoints or waypoints[-1].pose.position != goal.pose.position:
        waypoints.append(deepcopy(goal))
    return waypoints


def transform_grasp_to_tcp_target(
    grasp: PoseStamped,
    tcp_axis_mapping: str,
    tcp_grasp_offset_z: float,
) -> PoseStamped:
    target = apply_tcp_axis_mapping(grasp, tcp_axis_mapping)
    apply_tcp_grasp_offset(target, tcp_grasp_offset_z)
    return target


def pose_from_dict(data: dict):
    pose = PoseStamped().pose
    pose.position.x = float(data["position"]["x"])
    pose.position.y = float(data["position"]["y"])
    pose.position.z = float(data["position"]["z"])
    pose.orientation.x = float(data["orientation"]["x"])
    pose.orientation.y = float(data["orientation"]["y"])
    pose.orientation.z = float(data["orientation"]["z"])
    pose.orientation.w = float(data["orientation"]["w"])
    return pose


def candidates_from_selection_snapshot(snapshot: dict) -> PoseArray:
    if "selected_grasp" in snapshot:
        selected_grasp = snapshot["selected_grasp"]
    else:
        selected_index = int(snapshot["selected_index"])
        selected_grasp = snapshot["candidates"][selected_index]

    candidates = PoseArray()
    candidates.header.frame_id = snapshot.get("frame_id", "world")
    candidates.poses.append(pose_from_dict(selected_grasp))
    return candidates


def load_frozen_candidates(path: str) -> PoseArray:
    snapshot_path = Path(path)
    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    return candidates_from_selection_snapshot(snapshot)


def selection_snapshot_from_plan(plan: dict) -> dict:
    raw_grasp = plan["raw_grasp"]
    selected_grasp = pose_to_dict(raw_grasp.pose)
    snapshot = {
        "frame_id": raw_grasp.header.frame_id,
        "selected_index": 0,
        "source_candidate_index": int(plan["index"]),
        "candidates": [selected_grasp],
        "selected_grasp": selected_grasp,
    }
    if "tcp_grasp_offset_z" in plan:
        snapshot["tcp_grasp_offset_z"] = float(plan["tcp_grasp_offset_z"])
    return snapshot


class AnyGraspFullPipeline(Node):
    def __init__(self, args):
        super().__init__("anygrasp_full_pipeline")
        self.args = args
        self.ik_client = self.create_client(GetPositionIK, args.ik_service)
        self.plan_client = self.create_client(GetMotionPlan, args.plan_service)
        self.execute_client = ActionClient(
            self,
            FollowJointTrajectory,
            args.follow_joint_trajectory_action,
        )
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            args.gripper_follow_joint_trajectory_action,
        )
        self.control_enable_client = self.create_client(SetBool, args.control_enable_service)
        self.cfg = SafetyConfig(
            base_frame=args.base_frame,
            min_x=args.min_x,
            max_x=args.max_x,
            min_y=args.min_y,
            max_y=args.max_y,
            min_z=args.min_z,
            max_z=args.max_z,
            min_radius=args.min_radius,
            max_radius=args.max_radius,
            table_z=args.table_z,
            min_table_clearance=args.min_table_clearance,
        )

    def wait_for_message(self, msg_type, topic):
        box = {"msg": None}

        def callback(msg):
            box["msg"] = msg

        sub = self.create_subscription(msg_type, topic, callback, 1)
        while rclpy.ok() and box["msg"] is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        return box["msg"]

    def arm_joint_state(self, joint_state: JointState):
        filtered = JointState()
        filtered.header = joint_state.header
        for name, position in zip(joint_state.name, joint_state.position):
            if name.startswith("joint"):
                filtered.name.append(name)
                filtered.position.append(position)
        return filtered

    def compute_ik(self, pose_msg: PoseStamped, joint_state: JointState):
        if not self.ik_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"IK service not available: {self.args.ik_service}")

        request = GetPositionIK.Request()
        request.ik_request.group_name = self.args.group_name
        request.ik_request.robot_state.joint_state = self.arm_joint_state(joint_state)
        request.ik_request.avoid_collisions = True
        request.ik_request.ik_link_name = self.args.ik_link_name
        request.ik_request.pose_stamped = pose_msg
        request.ik_request.timeout = Duration(sec=2)

        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.error_code.val != 1:
            raise RuntimeError(f"IK failed: MoveItErrorCodes={response.error_code.val}")
        return response.solution.joint_state

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

    def plan_to_joint_state(self, start_state: JointState, goal_state: JointState):
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

    def validate_waypoint_chain(self, waypoints: list, joint_state: JointState) -> tuple:
        current = joint_state
        trajectories = []
        for waypoint in waypoints:
            goal_state = self.compute_ik(waypoint, current)
            trajectory = self.plan_to_joint_state(current, goal_state)
            trajectories.append(trajectory)
            current = goal_state
        return current, trajectories

    def evaluate_candidate(
        self,
        candidates: PoseArray,
        index: int,
        start_joint_state: JointState,
        tcp_grasp_offset_z: float = None,
    ) -> tuple:
        if tcp_grasp_offset_z is None:
            tcp_grasp_offset_z = self.args.tcp_grasp_offset_z

        if index >= len(candidates.poses):
            return None, CandidateReport(index, False, "missing candidate", tcp_grasp_offset_z)

        raw_grasp = pose_array_item_to_stamped(candidates, index)
        raw_decision = validate_candidate_center_pose(raw_grasp, self.cfg)
        if not raw_decision.accepted:
            return None, CandidateReport(index, False, raw_decision.reason, tcp_grasp_offset_z)

        grasp = transform_grasp_to_tcp_target(
            raw_grasp,
            self.args.tcp_axis_mapping,
            tcp_grasp_offset_z,
        )
        grasp_decision = validate_grasp_pose(grasp, self.cfg)
        if not grasp_decision.accepted:
            return None, CandidateReport(
                index,
                False,
                "grasp target: " + grasp_decision.reason,
                tcp_grasp_offset_z,
            )

        pregrasp = offset_pose_along_local_z(grasp, -self.args.approach_distance)
        pregrasp_decision = validate_grasp_pose(pregrasp, self.cfg)
        if not pregrasp_decision.accepted:
            return None, CandidateReport(
                index,
                False,
                "pregrasp: " + pregrasp_decision.reason,
                tcp_grasp_offset_z,
            )

        lift = build_world_lift_pose(grasp, self.args.lift_distance)
        lift_decision = validate_grasp_pose(lift, self.cfg)
        if not lift_decision.accepted:
            return None, CandidateReport(
                index,
                False,
                "lift: " + lift_decision.reason,
                tcp_grasp_offset_z,
            )

        try:
            pregrasp_state = self.compute_ik(pregrasp, start_joint_state)
            pregrasp_traj = self.plan_to_joint_state(start_joint_state, pregrasp_state)
            approach_waypoints = build_linear_waypoints(
                pregrasp,
                grasp,
                self.args.cartesian_step,
            )
            grasp_state, approach_trajectories = self.validate_waypoint_chain(
                approach_waypoints,
                pregrasp_state,
            )
            lift_waypoints = build_linear_waypoints(grasp, lift, self.args.cartesian_step)
            _, lift_trajectories = self.validate_waypoint_chain(lift_waypoints, grasp_state)
        except RuntimeError as exc:
            return None, CandidateReport(index, False, str(exc), tcp_grasp_offset_z)

        report = CandidateReport(
            index=index,
            accepted=True,
            reason="accepted",
            tcp_grasp_offset_z=tcp_grasp_offset_z,
            pregrasp_points=len(pregrasp_traj.joint_trajectory.points),
            approach_steps=len(approach_waypoints),
            lift_steps=len(lift_waypoints),
        )
        plan = {
            "index": index,
            "tcp_grasp_offset_z": tcp_grasp_offset_z,
            "raw_grasp": raw_grasp,
            "pregrasp": pregrasp,
            "grasp": grasp,
            "lift": lift,
            "pregrasp_trajectory": pregrasp_traj,
            "approach_trajectories": approach_trajectories,
            "lift_trajectories": lift_trajectories,
        }
        return plan, report

    def call_control_enable(self, enabled: bool):
        if not self.control_enable_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"control enable service not available: {self.args.control_enable_service}")

        request = SetBool.Request()
        request.data = enabled
        future = self.control_enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            raise RuntimeError("control enable service timed out")
        if not future.result().success:
            raise RuntimeError(f"control enable failed: {future.result().message}")

    def send_joint_trajectory(self, action_client, trajectory, label: str):
        if not action_client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError(f"{label} trajectory action not available")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory
        future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError(f"{label} trajectory goal rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"{label} trajectory execution failed: code={result.error_code} {result.error_string}"
            )

    def build_gripper_trajectory(self, width: float, duration_sec: int):
        trajectory = type("TrajectoryBox", (), {})()
        trajectory.joint_trajectory = JointTrajectory()
        trajectory.joint_trajectory.joint_names = [self.args.gripper_joint_name]
        point = JointTrajectoryPoint()
        point.positions = [width]
        point.velocities = [0.0]
        point.time_from_start.sec = duration_sec
        trajectory.joint_trajectory.points = [point]
        return trajectory

    def move_gripper(self, width: float, duration_sec: int, label: str):
        self.call_control_enable(True)
        try:
            self.send_joint_trajectory(
                self.gripper_client,
                self.build_gripper_trajectory(width, duration_sec),
                label,
            )
        finally:
            try:
                self.call_control_enable(False)
            except RuntimeError as exc:
                self.get_logger().warn(f"failed to close control gate: {exc}")

    def execute_plan(self, plan: dict):
        self.get_logger().warn("EXECUTE enabled: opening gripper")
        self.move_gripper(self.args.open_width, self.args.gripper_open_duration, "open gripper")

        self.get_logger().warn("executing pregrasp trajectory")
        self.send_joint_trajectory(
            self.execute_client,
            plan["pregrasp_trajectory"],
            "pregrasp",
        )

        self.get_logger().warn("executing approach trajectories")
        for index, trajectory in enumerate(plan["approach_trajectories"]):
            self.send_joint_trajectory(self.execute_client, trajectory, f"approach {index}")

        self.get_logger().warn("closing gripper")
        self.move_gripper(self.args.close_width, self.args.gripper_close_duration, "close gripper")

        self.get_logger().warn("executing lift trajectories")
        for index, trajectory in enumerate(plan["lift_trajectories"]):
            self.send_joint_trajectory(self.execute_client, trajectory, f"lift {index}")

    def run(self):
        if self.args.frozen_selection_path:
            candidates = load_frozen_candidates(self.args.frozen_selection_path)
            self.get_logger().info(
                "loaded frozen grasp candidate from %s" % self.args.frozen_selection_path
            )
        else:
            candidates = self.wait_for_message(PoseArray, self.args.candidates_topic)
        joint_state = self.wait_for_message(JointState, self.args.joint_state_topic)
        reports = []
        accepted_plan = None
        offsets = parse_float_list(self.args.tcp_grasp_offset_candidates)
        if not offsets:
            offsets = [self.args.tcp_grasp_offset_z]

        for offset in offsets:
            max_count = min(len(candidates.poses), self.args.max_candidates)
            for index in range(max_count):
                plan, report = self.evaluate_candidate(candidates, index, joint_state, offset)
                reports.append(report)
                self.get_logger().info(
                    "offset %.3f candidate %d: %s (%s)"
                    % (
                        offset,
                        index,
                        "ACCEPT" if report.accepted else "REJECT",
                        report.reason,
                    )
                )
                if report.accepted and accepted_plan is None:
                    accepted_plan = plan
                    if self.args.stop_at_first:
                        break
            if accepted_plan is not None and self.args.stop_at_first:
                break

        self.write_report(reports)
        if accepted_plan is None:
            raise RuntimeError("no complete grasp candidate accepted")

        self.get_logger().info(
            "selected offset %.3f candidate %d: pregrasp=(%.3f, %.3f, %.3f), grasp=(%.3f, %.3f, %.3f), lift=(%.3f, %.3f, %.3f)"
            % (
                accepted_plan["tcp_grasp_offset_z"],
                accepted_plan["index"],
                accepted_plan["pregrasp"].pose.position.x,
                accepted_plan["pregrasp"].pose.position.y,
                accepted_plan["pregrasp"].pose.position.z,
                accepted_plan["grasp"].pose.position.x,
                accepted_plan["grasp"].pose.position.y,
                accepted_plan["grasp"].pose.position.z,
                accepted_plan["lift"].pose.position.x,
                accepted_plan["lift"].pose.position.y,
                accepted_plan["lift"].pose.position.z,
            )
        )
        self.write_selected_snapshot(accepted_plan)
        if self.args.execute:
            self.execute_plan(accepted_plan)

    def write_report(self, reports: list):
        if not self.args.report_path:
            return
        path = Path(self.args.report_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps([asdict(report) for report in reports], indent=2),
            encoding="utf-8",
        )

    def write_selected_snapshot(self, plan: dict):
        if not self.args.save_selected_path:
            return
        path = Path(self.args.save_selected_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(selection_snapshot_from_plan(plan), indent=2, sort_keys=True),
            encoding="utf-8",
        )
        self.get_logger().info("saved selected frozen grasp to %s" % path)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidates-topic", default="/anygrasp/grasp_candidates_base")
    parser.add_argument("--joint-state-topic", default="/feedback/joint_states")
    parser.add_argument("--ik-service", default="/compute_ik")
    parser.add_argument("--plan-service", default="/plan_kinematic_path")
    parser.add_argument(
        "--follow-joint-trajectory-action",
        default="/arm_controller/follow_joint_trajectory",
    )
    parser.add_argument(
        "--gripper-follow-joint-trajectory-action",
        default="/gripper_controller/follow_joint_trajectory",
    )
    parser.add_argument("--control-enable-service", default="/control_enable")
    parser.add_argument("--group-name", default="arm")
    parser.add_argument("--ik-link-name", default="tcp_link")
    parser.add_argument("--base-frame", default="world")
    parser.add_argument("--pipeline-id", default="ompl")
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--allowed-planning-time", type=float, default=5.0)
    parser.add_argument("--velocity-scale", type=float, default=0.05)
    parser.add_argument("--acceleration-scale", type=float, default=0.03)
    parser.add_argument("--goal-tolerance", type=float, default=0.01)
    parser.add_argument("--tcp-axis-mapping", default="anygrasp_x_to_tcp_z")
    parser.add_argument("--tcp-grasp-offset-z", type=float, default=0.12)
    parser.add_argument("--tcp-grasp-offset-candidates", default="")
    parser.add_argument("--approach-distance", type=float, default=0.10)
    parser.add_argument("--lift-distance", type=float, default=0.08)
    parser.add_argument("--cartesian-step", type=float, default=0.04)
    parser.add_argument("--max-candidates", type=int, default=20)
    parser.add_argument("--stop-at-first", action="store_true")
    parser.add_argument("--report-path", default="logs/anygrasp_runs/full_pipeline_report.json")
    parser.add_argument("--frozen-selection-path")
    parser.add_argument("--save-selected-path")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--gripper-joint-name", default="gripper")
    parser.add_argument("--open-width", type=float, default=0.08)
    parser.add_argument("--close-width", type=float, default=0.0)
    parser.add_argument("--gripper-open-duration", type=int, default=3)
    parser.add_argument("--gripper-close-duration", type=int, default=5)
    parser.add_argument("--min-x", type=float, default=0.12)
    parser.add_argument("--max-x", type=float, default=0.62)
    parser.add_argument("--min-y", type=float, default=-0.30)
    parser.add_argument("--max-y", type=float, default=0.30)
    parser.add_argument("--min-z", type=float, default=0.06)
    parser.add_argument("--max-z", type=float, default=0.45)
    parser.add_argument("--min-radius", type=float, default=0.14)
    parser.add_argument("--max-radius", type=float, default=0.68)
    parser.add_argument("--table-z", type=float, default=0.0)
    parser.add_argument("--min-table-clearance", type=float, default=0.06)
    return parser.parse_args()


def main():
    rclpy.init()
    node = AnyGraspFullPipeline(parse_args())
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
