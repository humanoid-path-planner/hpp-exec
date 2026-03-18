"""
Send trajectories to ros2_control.

Simple API for executing HPP-generated trajectories on ROS2 robots.

Example:
    from hpp_exec import send_trajectory

    # Your HPP script generates waypoints...
    waypoints = [np.array([0, 0, 0, 0, 0, 0]), np.array([1, 1, 1, 1, 1, 1])]
    times = [0.0, 2.0]

    # Execute on robot
    send_trajectory(
        waypoints, times,
        joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    )
"""

from typing import List, Optional
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory

from hpp_exec.trajectory_utils import (
    waypoints_to_joint_trajectory,
    add_time_parameterization,
)


class _TrajectorySenderNode(Node):
    """Internal node for sending trajectories."""

    def __init__(
        self,
        controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
    ):
        super().__init__("hpp_trajectory_sender")
        self.client = ActionClient(self, FollowJointTrajectory, controller_topic)
        self._result = None

    def send_and_wait(self, trajectory, timeout_sec: float = 60.0) -> bool:
        """Send trajectory and wait for execution to complete."""
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Trajectory controller not available")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        # Compute expected duration from last trajectory point
        last_point = trajectory.points[-1]
        duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9

        self.get_logger().info(
            f"Sending trajectory: {len(trajectory.points)} points, "
            f"{len(trajectory.joint_names)} joints, {duration:.1f}s"
        )

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected")
            return False

        self.get_logger().info("Trajectory accepted, executing...")

        # Wait for execution to complete
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)

        result = result_future.result()
        if result is None:
            self.get_logger().error("Trajectory execution timed out")
            return False

        self.get_logger().info("Trajectory execution complete")
        return True


def send_trajectory(
    waypoints: List[np.ndarray],
    times: List[float],
    joint_names: List[str],
    controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
    max_velocity: Optional[float] = None,
    max_acceleration: Optional[float] = None,
    joint_indices: Optional[List[int]] = None,
) -> bool:
    """
    Send a trajectory to ros2_control.

    IMPORTANT: HPP path parameters are NOT real time. If you extract waypoints
    from an HPP path using path(t), the times are path parameters (arc length),
    not seconds. You must either:

      1. Use max_velocity to rescale (simple, good enough for testing):
           send_trajectory(waypoints, times, joints, max_velocity=1.0)

      2. Time-parameterize inside HPP first (proper, recommended for production):
           ps = problem.pathOptimizer("SimpleTimeParameterization")
           timed_path = ps.optimize(path)
           # Now times from timed_path are real seconds, no rescaling needed
           send_trajectory(waypoints, times, joints)

    If you pass raw path parameters without max_velocity, the trajectory will
    execute at wrong speeds (typically way too fast).

    Args:
        waypoints: List of configuration vectors (numpy arrays).
        times: List of timestamps in seconds. If these are HPP path parameters,
               you MUST pass max_velocity to rescale them.
        joint_names: ROS2 joint names in order.
        controller_topic: FollowJointTrajectory action topic.
        max_velocity: Rescale times so no joint moves faster than this (rad/s).
            Required when times are raw HPP path parameters.
        max_acceleration: Max joint acceleration for rescaling (rad/s^2, default 0.5).
        joint_indices: Indices to extract from each waypoint (default: 0..len(joint_names)).

    Returns:
        True if trajectory executed successfully.

    Example:
        # From your HPP script:
        path = planner.solve()
        waypoints = [np.array(path(t)[0]) for t in np.linspace(0, path.length(), 100)]
        times = list(np.linspace(0, path.length(), 100))

        # times are path parameters — must rescale:
        send_trajectory(
            waypoints, times,
            joint_names=["shoulder_pan", "shoulder_lift", "elbow", ...],
            max_velocity=1.0,  # Scale path parameter to real time
        )
    """
    # Scale times if velocity/acceleration limits provided
    if max_velocity is not None:
        times = add_time_parameterization(
            waypoints, times,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration or 0.5,
        )

    # Convert to ROS2 message
    trajectory = waypoints_to_joint_trajectory(
        waypoints, times, joint_names,
        joint_indices=joint_indices,
    )

    # Initialize ROS2 if needed
    if not rclpy.ok():
        rclpy.init()

    node = _TrajectorySenderNode(controller_topic)
    try:
        return node.send_and_wait(trajectory)
    finally:
        node.destroy_node()


def send_trajectory_async(
    waypoints: List[np.ndarray],
    times: List[float],
    joint_names: List[str],
    controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
    max_velocity: Optional[float] = None,
    max_acceleration: Optional[float] = None,
    joint_indices: Optional[List[int]] = None,
):
    """
    Send trajectory without waiting for completion.

    Returns the goal handle for later status checking.
    Caller is responsible for ROS2 lifecycle (rclpy.init/shutdown).
    """
    if max_velocity is not None:
        times = add_time_parameterization(
            waypoints, times,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration or 0.5,
        )

    trajectory = waypoints_to_joint_trajectory(
        waypoints, times, joint_names,
        joint_indices=joint_indices,
    )

    if not rclpy.ok():
        rclpy.init()

    node = _TrajectorySenderNode(controller_topic)

    if not node.client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error("Trajectory controller not available")
        return None

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory

    future = node.client.send_goal_async(goal)
    return future, node
