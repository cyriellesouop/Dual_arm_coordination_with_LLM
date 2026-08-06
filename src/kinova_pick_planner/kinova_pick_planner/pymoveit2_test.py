#!/usr/bin/env python3
"""Quick test of pymoveit2 with Gen3 Lite."""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import time

from pymoveit2 import MoveIt2, GripperInterface


def main():
    rclpy.init()
    node = Node("pymoveit2_test")
    
    cb = ReentrantCallbackGroup()
    
    arm = MoveIt2(
        node=node,
        joint_names=["joint_1", "joint_2", "joint_3", 
                     "joint_4", "joint_5", "joint_6"],
        base_link_name="base_link",
        end_effector_name="tool_frame",
        group_name="arm",
        callback_group=cb,
    )
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    time.sleep(2.0)
    node.get_logger().info("pymoveit2 initialized!")
    
    # Test: get current pose
    current = arm.compute_fk()
    if current:
        p = current.pose.position
        o = current.pose.orientation
        node.get_logger().info(
            f"Current pose: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) "
            f"quat: ({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})"
        )
    else:
        node.get_logger().error("compute_fk failed")
    
    # Test: small Cartesian move (plan only, check if it works)
    node.get_logger().info("Testing Cartesian move to (0.4, 0.0, 0.2)...")
    arm.move_to_pose(
        position=(0.4, 0.0, 0.2),
        quat_xyzw=(1.0, 0.0, 0.0, 0.0),
        cartesian=False,
    )
    success = arm.wait_until_executed()
    node.get_logger().info(f"Move result: {success}")
    
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()