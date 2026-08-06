#!/usr/bin/env python3
"""
Camera Integration Example
============================
Shows how your teammate's camera code feeds coordinates into the planner.
Replace get_camera_data() with actual detection output.
"""

import rclpy
from kinova_pick_planner.kinova_pick_planner import (
    KinovaPickPlanner,
    TargetPose,
    SceneObject,
    build_scene_from_camera_data,
    TABLE_POSITION,
)


def get_camera_data() -> dict:
    """
    STUB: Replace with actual camera detection output.
    All coordinates must already be in the robot's base_link frame.
    """
    return {
        "target": {
            "x": 0.40,
            "y": 0.05,
            "z": TABLE_POSITION["z"] + TABLE_POSITION["thickness"] / 2,
            "type": "foam_ball",
            "label": "red_ball",
        },
        "obstacles": [
            {
                "x": 0.35, "y": 0.15,
                "z": TABLE_POSITION["z"] + TABLE_POSITION["thickness"] / 2,
                "type": "coffee_cup",
                "id": "cup_1",
            },
            {
                "x": 0.45, "y": -0.10,
                "z": TABLE_POSITION["z"] + TABLE_POSITION["thickness"] / 2,
                "type": "water_bottle",
                "id": "bottle_1",
            },
        ],
    }


def main():
    rclpy.init()
    node = KinovaPickPlanner()

    try:
        camera_data = get_camera_data()
        node.get_logger().info(
            f"Camera detected target '{camera_data['target'].get('label', 'unknown')}' "
            f"and {len(camera_data['obstacles'])} obstacles"
        )

        target = build_scene_from_camera_data(
            node=node,
            target_coord=(
                camera_data["target"]["x"],
                camera_data["target"]["y"],
                camera_data["target"]["z"],
            ),
            obstacle_coords=camera_data["obstacles"],
        )

        node.get_logger().info("Planning approach to target...")
        success = node.move_to_approach(target, execute=False)

        if success:
            node.get_logger().info("Arrived at target! Closing gripper...")
            node.close_gripper()
        else:
            node.get_logger().error("Motion planning failed!")

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()