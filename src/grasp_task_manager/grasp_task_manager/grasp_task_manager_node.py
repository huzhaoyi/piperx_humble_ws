import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError

from grasp_task_manager.state_machine import GraspTaskStateMachine


class GraspTaskManagerNode(Node):
    def __init__(self):
        super().__init__("grasp_task_manager")
        self.state_machine = GraspTaskStateMachine()
        self.get_logger().info("grasp_task_manager ready")


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
