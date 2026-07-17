#!/usr/bin/env python3

import argparse

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class AnyGraspApplyScene(Node):
    def __init__(self, args):
        super().__init__("anygrasp_apply_scene")
        self.args = args
        self.client = self.create_client(ApplyPlanningScene, args.apply_scene_service)

    def build_table_object(self) -> CollisionObject:
        table = CollisionObject()
        table.header.frame_id = self.args.base_frame
        table.id = self.args.table_id
        table.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [
            self.args.table_size_x,
            self.args.table_size_y,
            self.args.table_thickness,
        ]

        pose = Pose()
        pose.position.x = self.args.table_center_x
        pose.position.y = self.args.table_center_y
        pose.position.z = self.args.table_z - self.args.table_thickness * 0.5
        pose.orientation.w = 1.0

        table.primitives.append(box)
        table.primitive_poses.append(pose)
        return table

    def build_remove_object(self) -> CollisionObject:
        obj = CollisionObject()
        obj.header.frame_id = self.args.base_frame
        obj.id = self.args.table_id
        obj.operation = CollisionObject.REMOVE
        return obj

    def run(self):
        if not self.client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(
                f"planning scene service not available: {self.args.apply_scene_service}"
            )

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(
            self.build_remove_object() if self.args.remove else self.build_table_object()
        )

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if not response.success:
            raise RuntimeError("failed to apply planning scene")

        action = "removed" if self.args.remove else "applied"
        self.get_logger().info(
            "%s table collision object '%s' in frame '%s'"
            % (action, self.args.table_id, self.args.base_frame)
        )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--apply-scene-service", default="/apply_planning_scene")
    parser.add_argument("--base-frame", default="world")
    parser.add_argument("--table-id", default="anygrasp_table")
    parser.add_argument("--table-z", type=float, default=0.0)
    parser.add_argument("--table-center-x", type=float, default=0.38)
    parser.add_argument("--table-center-y", type=float, default=0.0)
    parser.add_argument("--table-size-x", type=float, default=0.90)
    parser.add_argument("--table-size-y", type=float, default=0.80)
    parser.add_argument("--table-thickness", type=float, default=0.04)
    parser.add_argument("--remove", action="store_true")
    return parser.parse_args()


def main():
    rclpy.init()
    node = AnyGraspApplyScene(parse_args())
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
