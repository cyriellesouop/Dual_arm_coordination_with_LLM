#!/usr/bin/env python3
"""Minimal test to debug the 99999 error."""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotState,
)
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
import time


def main():
    rclpy.init()
    node = Node("plan_test")

    cli = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    node.get_logger().info("Waiting for service...")
    cli.wait_for_service(timeout_sec=10.0)
    node.get_logger().info("Service available!")

    # Build the simplest possible request
    req = GetMotionPlan.Request()
    mp = req.motion_plan_request

    mp.group_name = "arm"
    mp.pipeline_id = "ompl"
    mp.num_planning_attempts = 10
    mp.allowed_planning_time = 10.0
    mp.max_velocity_scaling_factor = 0.3
    mp.max_acceleration_scaling_factor = 0.3

    # Start from current state
    mp.start_state.is_diff = True

    # Workspace
    mp.workspace_parameters.header.frame_id = "base_link"
    mp.workspace_parameters.min_corner.x = -1.0
    mp.workspace_parameters.min_corner.y = -1.0
    mp.workspace_parameters.min_corner.z = -1.0
    mp.workspace_parameters.max_corner.x = 1.0
    mp.workspace_parameters.max_corner.y = 1.0
    mp.workspace_parameters.max_corner.z = 1.0

    # ONLY a position constraint, no orientation
    constraints = Constraints()

    pos_c = PositionConstraint()
    pos_c.header.frame_id = "base_link"
    pos_c.link_name = "tool_frame"
    pos_c.weight = 1.0

    bv = BoundingVolume()
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [0.01]

    sphere_pose = Pose()
    sphere_pose.position.x = 0.3
    sphere_pose.position.y = 0.0
    sphere_pose.position.z = 0.4
    sphere_pose.orientation.w = 1.0

    bv.primitives.append(sphere)
    bv.primitive_poses.append(sphere_pose)
    pos_c.constraint_region = bv

    constraints.position_constraints.append(pos_c)
    mp.goal_constraints.append(constraints)

    node.get_logger().info("Sending plan request (position only)...")
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)

    if future.result() is None:
        node.get_logger().error("Service call returned None")
    else:
        code = future.result().motion_plan_response.error_code.val
        node.get_logger().info(f"Result error code: {code}")
        if code == 1:
            node.get_logger().info("SUCCESS! Position-only planning works.")
        else:
            node.get_logger().error(f"Failed with code: {code}")

            # Also try printing the raw response
            node.get_logger().info(
                f"Response planning time: "
                f"{future.result().motion_plan_response.planning_time}"
            )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()