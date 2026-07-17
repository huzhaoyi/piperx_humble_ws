import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from std_srvs.srv import Trigger

from grasp_task_manager.state_machine import GraspEvent, GraspTaskStateMachine


class GraspTaskManagerNode(Node):
    def __init__(self):
        super().__init__("grasp_task_manager")
        self.declare_parameter("require_home_before_detection", True)
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
