import rclpy
from grasp_interfaces.action import ExecuteGrasp
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from std_srvs.srv import Trigger

from grasp_task_manager.pipeline_runner import (
    GraspPipelineRunner,
    PipelineStageError,
    build_pipeline_config,
)
from grasp_task_manager.state_machine import GraspEvent, GraspState, GraspTaskStateMachine


class GraspTaskManagerNode(Node):
    def __init__(self):
        super().__init__("grasp_task_manager")
        self.declare_parameter("require_home_before_detection", True)
        self.declare_parameter(
            "frozen_selection_path",
            "logs/anygrasp_runs/frozen_selected_latest.json",
        )
        self.declare_parameter(
            "plan_report_path",
            "logs/anygrasp_runs/full_pipeline_report_latest.json",
        )
        self.declare_parameter(
            "execute_report_path",
            "logs/anygrasp_runs/frozen_execute_report.json",
        )
        self.declare_parameter("tcp_grasp_offset_candidates", "0.12,0.10,0.08,0.06")
        self.declare_parameter("max_candidates", 50)
        self.declare_parameter("move_home_before_execute", True)
        require_home = (
            self.get_parameter("require_home_before_detection")
            .get_parameter_value()
            .bool_value
        )
        self.state_machine = GraspTaskStateMachine(
            require_home_before_detection=require_home
        )
        self.create_service(Trigger, "~/start", self.handle_start)
        self.create_service(Trigger, "~/reset", self.handle_reset)
        self.create_service(Trigger, "~/abort", self.handle_abort)
        self.action_server = ActionServer(
            self,
            ExecuteGrasp,
            "execute_grasp",
            self.handle_execute_grasp,
        )
        self.get_logger().info(
            "grasp_task_manager ready: state=%s require_home_before_detection=%s"
            % (self.state_machine.state.name, str(require_home).lower())
        )

    def handle_start(self, _request, response):
        state = self.state_machine.handle(GraspEvent.START_REQUESTED)
        response.success = True
        response.message = state.name
        self.get_logger().info("start requested: state=%s" % state.name)
        return response

    def handle_reset(self, _request, response):
        state = self.state_machine.handle(GraspEvent.RESET_REQUESTED)
        response.success = True
        response.message = state.name
        self.get_logger().info("reset requested: state=%s" % state.name)
        return response

    def handle_abort(self, _request, response):
        state = self.state_machine.handle(GraspEvent.ABORT_REQUESTED)
        response.success = True
        response.message = state.name
        self.get_logger().warn("abort requested: state=%s" % state.name)
        return response

    def handle_execute_grasp(self, goal_handle):
        result = ExecuteGrasp.Result()
        if goal_handle.request.enable_place:
            goal_handle.abort()
            result.success = False
            result.error_code = 10
            result.message = "place stage is not implemented in this closed loop"
            return result

        self.state_machine.handle(GraspEvent.RESET_REQUESTED)
        self.state_machine.handle(GraspEvent.START_REQUESTED)
        if self.state_machine.state == GraspState.HOMING:
            self.state_machine.handle(GraspEvent.HOME_REACHED)
            self.state_machine.handle(GraspEvent.START_REQUESTED)

        config = build_pipeline_config(self.pipeline_parameters(), execute=True)
        runner = GraspPipelineRunner(
            config,
            feedback=lambda stage, progress, description: self.publish_action_feedback(
                goal_handle,
                stage,
                progress,
                description,
            ),
        )

        try:
            runner.run()
        except PipelineStageError as exc:
            self.state_machine.handle(GraspEvent.FAULT_DETECTED)
            goal_handle.abort()
            result.success = False
            result.error_code = 20
            result.message = str(exc)
            return result

        self.state_machine.handle(GraspEvent.CANDIDATE_LOCKED)
        self.state_machine.handle(GraspEvent.PLAN_SUCCEEDED)
        self.state_machine.handle(GraspEvent.EXECUTE_REQUESTED)
        self.state_machine.handle(GraspEvent.PREGRASP_REACHED)
        self.state_machine.handle(GraspEvent.APPROACH_COMPLETE)
        self.state_machine.handle(GraspEvent.GRIPPER_CLOSED)
        self.state_machine.handle(GraspEvent.GRASP_VERIFIED)
        self.state_machine.handle(GraspEvent.LIFT_COMPLETE)

        goal_handle.succeed()
        result.success = True
        result.error_code = 0
        result.message = "grasp closed loop completed"
        return result

    def publish_action_feedback(self, goal_handle, stage: str, progress: float, description: str):
        feedback = ExecuteGrasp.Feedback()
        feedback.state = min(255, self.state_machine.state.value)
        feedback.progress = float(progress)
        feedback.description = f"{stage}: {description}"
        goal_handle.publish_feedback(feedback)

    def pipeline_parameters(self) -> dict:
        return {
            "frozen_selection_path": self.get_parameter("frozen_selection_path").value,
            "plan_report_path": self.get_parameter("plan_report_path").value,
            "execute_report_path": self.get_parameter("execute_report_path").value,
            "tcp_grasp_offset_candidates": self.get_parameter(
                "tcp_grasp_offset_candidates"
            ).value,
            "max_candidates": self.get_parameter("max_candidates").value,
            "move_home_before_execute": self.get_parameter("move_home_before_execute").value,
        }


def main(args=None):
    rclpy.init(args=args)
    node = GraspTaskManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except RCLError:
            pass


if __name__ == "__main__":
    main()
