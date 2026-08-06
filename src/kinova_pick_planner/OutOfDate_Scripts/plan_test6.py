#!/usr/bin/env python3
"""
Test 6: Use MoveGroup action with pose_stamped goal field
instead of manual constraints. This matches how RViz sends goals.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotState,
)
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive

import threading
import time


def wait_for_future(future, timeout_sec=10.0):
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    if event.wait(timeout=timeout_sec):
        return future.result()
    return None


def main():
    rclpy.init()
    node = Node("plan_test6")

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    from rclpy.callback_groups import ReentrantCallbackGroup
    cb = ReentrantCallbackGroup()

    ac = ActionClient(node, MoveGroup, "/move_action", callback_group=cb)
    node.get_logger().info("Waiting for /move_action...")
    ac.wait_for_server(timeout_sec=10.0)
    node.get_logger().info("Connected!")

    # === Test: Plan to a Cartesian pose using the SAME method RViz uses ===
    goal = MoveGroup.Goal()

    mp = goal.request
    mp.group_name = "arm"
    mp.num_planning_attempts = 10
    mp.allowed_planning_time = 10.0
    mp.max_velocity_scaling_factor = 0.3
    mp.max_acceleration_scaling_factor = 0.3
    mp.pipeline_id = "ompl"
    mp.planner_id = "RRTConnect"  # explicit planner

    mp.start_state.is_diff = True

    mp.workspace_parameters.header.frame_id = "base_link"
    mp.workspace_parameters.min_corner.x = -1.0
    mp.workspace_parameters.min_corner.y = -1.0
    mp.workspace_parameters.min_corner.z = -1.0
    mp.workspace_parameters.max_corner.x = 1.0
    mp.workspace_parameters.max_corner.y = 1.0
    mp.workspace_parameters.max_corner.z = 1.0

    # Position constraint with LARGER tolerance
    constraints = Constraints()

    pos_c = PositionConstraint()
    pos_c.header.frame_id = "base_link"
    pos_c.link_name = "tool_frame"
    pos_c.weight = 1.0

    bv = BoundingVolume()
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [0.02]  # 2cm tolerance (was 1cm)

    target_pose = Pose()
    target_pose.position.x = 0.40
    target_pose.position.y = 0.05
    target_pose.position.z = 0.15  # safe height above table
    target_pose.orientation.w = 1.0

    bv.primitives.append(sphere)
    bv.primitive_poses.append(target_pose)
    pos_c.constraint_region = bv
    constraints.position_constraints.append(pos_c)

    # Orientation — VERY loose, just roughly downward
    orient_c = OrientationConstraint()
    orient_c.header.frame_id = "base_link"
    orient_c.link_name = "tool_frame"
    orient_c.orientation.x = 1.0  # gripper down
    orient_c.orientation.y = 0.0
    orient_c.orientation.z = 0.0
    orient_c.orientation.w = 0.0
    orient_c.absolute_x_axis_tolerance = 0.8  # ~45 degrees
    orient_c.absolute_y_axis_tolerance = 0.8  # ~45 degrees
    orient_c.absolute_z_axis_tolerance = 3.14159  # free rotation
    orient_c.weight = 1.0
    constraints.orientation_constraints.append(orient_c)

    mp.goal_constraints.append(constraints)

    goal.planning_options.plan_only = False  # plan AND execute
    goal.planning_options.replan = True
    goal.planning_options.replan_attempts = 3

    node.get_logger().info("Sending MoveGroup goal (plan+execute)...")
    future = ac.send_goal_async(goal)
    goal_handle = wait_for_future(future, timeout_sec=5.0)

    if goal_handle is None:
        node.get_logger().error("Failed to send goal")
    elif not goal_handle.accepted:
        node.get_logger().error("Goal rejected")
    else:
        node.get_logger().info("Goal accepted, waiting...")
        result_future = goal_handle.get_result_async()
        result = wait_for_future(result_future, timeout_sec=30.0)
        if result:
            code = result.result.error_code.val
            node.get_logger().info(f"Result: {code}")
            if code == 1:
                node.get_logger().info("SUCCESS!")
            else:
                node.get_logger().error(f"Failed: {code}")
        else:
            node.get_logger().error("Timed out waiting for result")

    time.sleep(1.0)
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()