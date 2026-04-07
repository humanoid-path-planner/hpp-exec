"""
Send trajectories to ros2_control.

Simple API for executing HPP-generated trajectories on ROS2 robots.

Example:
    from hpp_exec import send_trajectory, execute_segments, Segment

    # Your HPP script generates configs...
    configs = [np.array([0, 0, 0, 0, 0, 0]), np.array([1, 1, 1, 1, 1, 1])]
    times = [0.0, 2.0]

    # Simple execution (no actions between segments):
    send_trajectory(
        configs, times,
        joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    )

    # With pre/post actions between segments:
    segments = [
        Segment(0, 150),
        Segment(150, 300, pre_actions=[gripper.close]),
        Segment(300, 462, pre_actions=[gripper.open]),
    ]
    execute_segments(segments, configs, times, joint_names=[...])
"""

import logging
from dataclasses import dataclass, field
from typing import Callable, List, Optional

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node

from hpp_exec.trajectory_utils import (
    add_time_parameterization,
    configs_to_joint_trajectory,
)

logger = logging.getLogger(__name__)


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
        duration = (
            last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9
        )

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
    configs: List[np.ndarray],
    times: List[float],
    joint_names: List[str],
    controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
    max_velocity: Optional[float] = None,
    max_acceleration: Optional[float] = None,
    joint_indices: Optional[List[int]] = None,
) -> bool:
    """
    Send a trajectory to ros2_control.

    Args:
        configs: List of configuration vectors (numpy arrays).
        times: List of timestamps in seconds. If these are HPP path parameters,
               you MUST pass max_velocity to rescale them.
        joint_names: ROS2 joint names in order.
        controller_topic: FollowJointTrajectory action topic.
        max_velocity: Rescale times so no joint moves faster than this (rad/s).
            Required when times are raw HPP path parameters.
        max_acceleration: Max joint acceleration for rescaling (rad/s^2, default 0.5).
        joint_indices: Indices to extract from each config (default: 0..len(joint_names)).

    Returns:
        True if trajectory executed successfully.

    Example:
        # From your HPP script:
        path = planner.solve()
        configs = [np.array(path(t)[0]) for t in np.linspace(0, path.length(), 100)]
        times = list(np.linspace(0, path.length(), 100))

        # times are path parameters — must rescale:
        send_trajectory(
            configs, times,
            joint_names=["shoulder_pan", "shoulder_lift", "elbow", ...],
            max_velocity=1.0,  # Scale path parameter to real time
        )
    """
    # Scale times if velocity/acceleration limits provided
    if max_velocity is not None:
        times = add_time_parameterization(
            configs,
            times,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration or 0.5,
        )

    # Convert to ROS2 message
    trajectory = configs_to_joint_trajectory(
        configs,
        times,
        joint_names,
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
    configs: List[np.ndarray],
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
            configs,
            times,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration or 0.5,
        )

    trajectory = configs_to_joint_trajectory(
        configs,
        times,
        joint_names,
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


# ---------------------------------------------------------------------------
# Segment-based execution with pre/post action hooks
# ---------------------------------------------------------------------------


@dataclass
class Segment:
    """A trajectory segment with optional pre/post actions.

    Actions are callables that return True on success, False on failure.
    Any callable works: bound methods (gripper.close), lambdas, functions.

    Example:
        Segment(0, 150)                                     # no actions
        Segment(150, 300, pre_actions=[gripper.close])       # close gripper before
        Segment(300, 462, post_actions=[sensor.trigger])     # trigger sensor after
    """

    start_index: int
    """Start config index (inclusive)."""

    end_index: int
    """End config index (exclusive)."""

    pre_actions: list[Callable[[], bool]] = field(default_factory=list)
    """Actions to run before sending this segment's trajectory."""

    post_actions: list[Callable[[], bool]] = field(default_factory=list)
    """Actions to run after this segment's trajectory completes."""


def execute_segments(
    segments: List[Segment],
    configs: List[np.ndarray],
    times: List[float],
    joint_names: List[str],
    joint_indices: Optional[List[int]] = None,
    max_velocity: Optional[float] = None,
    max_acceleration: Optional[float] = None,
    controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
) -> bool:
    """Execute trajectory segments with pre/post action hooks.

    For each segment:
        1. Run all pre_actions (stop on first failure)
        2. Send the arm trajectory for this segment
        3. Run all post_actions (stop on first failure)

    Args:
        segments: Ordered list of Segment objects defining trajectory slices
            and their associated actions.
        configs: Full HPP configuration vectors.
        times: Timestamps for each config.
        joint_names: ROS2 joint names for the arm.
        joint_indices: Indices of arm DOFs in the HPP config vector.
            Default: 0..len(joint_names).
        max_velocity: Rescale times so no joint exceeds this velocity (rad/s).
        max_acceleration: Max joint acceleration for rescaling (rad/s^2).
        controller_topic: FollowJointTrajectory action topic.

    Returns:
        True if all segments and actions succeeded.
    """
    for i, segment in enumerate(segments):
        # 1. Pre-actions
        for action in segment.pre_actions:
            action_name = getattr(action, "__name__", None) or getattr(
                action, "__qualname__", repr(action)
            )
            logger.info("Segment %d: running pre-action '%s'", i, action_name)
            if not action():
                logger.error("Segment %d: pre-action '%s' failed", i, action_name)
                return False

        # 2. Send arm trajectory
        seg_configs = configs[segment.start_index : segment.end_index]
        seg_times = times[segment.start_index : segment.end_index]

        if len(seg_configs) >= 2:
            # Normalize times to start from 0
            t0 = seg_times[0]
            seg_times = [t - t0 for t in seg_times]

            logger.info(
                "Segment %d: sending %d configs (%.2fs)",
                i,
                len(seg_configs),
                seg_times[-1],
            )

            success = send_trajectory(
                seg_configs,
                seg_times,
                joint_names,
                controller_topic=controller_topic,
                max_velocity=max_velocity,
                max_acceleration=max_acceleration,
                joint_indices=joint_indices,
            )

            if not success:
                logger.error("Segment %d: arm trajectory failed", i)
                return False
        else:
            logger.info("Segment %d: single point, skipping trajectory", i)

        # 3. Post-actions
        for action in segment.post_actions:
            action_name = getattr(action, "__name__", None) or getattr(
                action, "__qualname__", repr(action)
            )
            logger.info("Segment %d: running post-action '%s'", i, action_name)
            if not action():
                logger.error("Segment %d: post-action '%s' failed", i, action_name)
                return False

    logger.info("All %d segments completed successfully", len(segments))
    return True
