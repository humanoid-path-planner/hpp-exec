#!/usr/bin/env python3
"""
hpp_planner API Example
=======================

Shows how hpp_planner simplifies trajectory execution.

BEFORE (40+ lines of ROS2 boilerplate):
    import rclpy
    from rclpy.action import ActionClient
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration

    rclpy.init()
    node = rclpy.create_node('my_node')
    action_client = ActionClient(node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
    action_client.wait_for_server(timeout_sec=10.0)

    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    for i, wp in enumerate(waypoints):
        point = JointTrajectoryPoint()
        point.positions = list(wp[:7])
        point.velocities = [0.0] * 7
        t = i * dt
        point.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
        trajectory.points.append(point)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory
    future = action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    rclpy.shutdown()

AFTER (3 lines with hpp_planner):
    from hpp_planner import send_trajectory

    success = send_trajectory(
        waypoints,
        times,
        joint_names=["joint1", "joint2", ...],
        joint_indices=[0, 1, 2, ...],  # optional: extract subset from waypoints
        max_velocity=1.0,              # optional: time scaling
    )
"""

# Example: HPP planning + hpp_planner execution

import numpy as np
from hpp_planner import send_trajectory

# Assume you have waypoints from HPP planning
# waypoints = [np.array([q1, q2, ...]), ...]
# times = [0.0, 0.1, 0.2, ...]

# Example FR3 usage:
FR3_JOINTS = [f"fr3_joint{i}" for i in range(1, 8)]

# send_trajectory(
#     waypoints,
#     times,
#     joint_names=FR3_JOINTS,
#     joint_indices=list(range(7)),  # FR3 has 9 DOF, we want joints 0-6
# )

# Example UR5 usage:
UR5_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# send_trajectory(
#     waypoints,
#     times,
#     joint_names=UR5_JOINTS,
#     max_velocity=0.5,
# )

print("See tutorial_mock.py for a complete working example.")
