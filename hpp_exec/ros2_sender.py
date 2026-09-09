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

    # Or attach actions by HPP graph transition name when executing:
    execute_segments(
        segments,
        configs,
        times,
        joint_names=[...],
        pre_actions_by_transition={"grasp transition": [gripper.close]},
        post_actions_by_transition={"release transition": [gripper.open]},
    )
"""

import logging
import threading
from itertools import count
from typing import Callable, List, Optional

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.type_support import check_for_type_support

from hpp_exec.actions import BackgroundAction
from hpp_exec.segments import Segment
from hpp_exec.trajectory_utils import configs_to_joint_trajectory

logger = logging.getLogger(__name__)
_NODE_IDS = count()
_RCLPY_INIT_LOCK = threading.Lock()
_TYPE_SUPPORT_LOCK = threading.Lock()
Action = Callable[[], bool]


def _ensure_rclpy_initialized() -> None:
    if rclpy.ok():
        return

    with _RCLPY_INIT_LOCK:
        if not rclpy.ok():
            rclpy.init()


def _ensure_trajectory_type_support() -> None:
    with _TYPE_SUPPORT_LOCK:
        check_for_type_support(FollowJointTrajectory)


def _action_name(action) -> str:
    action_owner = getattr(action, "__self__", None)
    if isinstance(action_owner, BackgroundAction):
        method_name = getattr(action, "__name__", None) or repr(action)
        return f"{action_owner.name}.{method_name}"

    return (
        getattr(action, "__qualname__", None)
        or getattr(action, "__name__", None)
        or repr(action)
    )


def _cancel_goal(executor, goal_handle, result_future):
    cancel_future = goal_handle.cancel_goal_async()
    executor.spin_until_future_complete(cancel_future, timeout_sec=5.0)
    response = cancel_future.result()
    if response is None or response.return_code != 0:
        return False
    executor.spin_until_future_complete(result_future, timeout_sec=5.0)
    result = result_future.result()
    return result is not None and (
        result.status == GoalStatus.STATUS_CANCELED
        or (
            result.status == GoalStatus.STATUS_SUCCEEDED
            and result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        )
    )


class _TrajectorySenderNode(Node):
    """Internal node for sending trajectories."""

    def __init__(
        self,
        controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
    ):
        super().__init__(f"hpp_trajectory_sender_{next(_NODE_IDS)}")
        _ensure_trajectory_type_support()
        self.client = ActionClient(self, FollowJointTrajectory, controller_topic)
        self._result = None

    def send_and_wait(
        self, trajectory, timeout_sec: float = 60.0, *, wait_for_completion=None
    ) -> bool:
        """Send trajectory and wait for execution to complete."""
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        future = goal_handle = result_future = None
        completed = cancel_requested = False
        try:
            if not self.client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("Trajectory controller not available")
                return False

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory

            self.get_logger().debug(
                f"Sending trajectory: {len(trajectory.points)} points"
            )

            future = self.client.send_goal_async(goal)
            executor.spin_until_future_complete(future, timeout_sec=10.0)

            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error("Trajectory goal rejected")
                return False

            result_future = goal_handle.get_result_async()
            if wait_for_completion is None:
                executor.spin_until_future_complete(
                    result_future, timeout_sec=timeout_sec
                )
            else:
                # The caller spins this node while observing controller feedback.
                executor.remove_node(self)
                try:
                    wait_for_completion(self, result_future)
                finally:
                    executor.add_node(self)
                if not result_future.done():
                    cancel_requested = True
                    completed = _cancel_goal(executor, goal_handle, result_future)
                    if not completed:
                        self.get_logger().error("Trajectory closure not confirmed")
                    return completed

            result = result_future.result()
            if result is None:
                self.get_logger().error("Trajectory execution timed out")
                return False

            if result.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error(
                    f"Trajectory execution failed with status {result.status}"
                )
                return False

            if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().error(
                    f"Trajectory execution failed with error code "
                    f"{result.result.error_code}: {result.result.error_string}"
                )
                return False

            completed = True
            self.get_logger().debug("Trajectory execution complete")
            return True
        finally:
            try:
                if not completed and not cancel_requested and future is not None:
                    if goal_handle is None:
                        executor.spin_until_future_complete(future, timeout_sec=5.0)
                        goal_handle = future.result()
                    if goal_handle is not None and goal_handle.accepted:
                        if result_future is None:
                            result_future = goal_handle.get_result_async()
                        if not _cancel_goal(executor, goal_handle, result_future):
                            self.get_logger().error(
                                "Trajectory cancellation not confirmed"
                            )
                    elif goal_handle is None:
                        self.get_logger().error("Trajectory acceptance unknown")
            finally:
                executor.remove_node(self)
                executor.shutdown()


def send_trajectory(
    configs: List[np.ndarray],
    times: List[float],
    joint_names: List[str],
    controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
    joint_indices: Optional[List[int]] = None,
    *,
    positions_only: bool = False,
    wait_for_completion=None,
) -> bool:
    """
    Send a trajectory to ros2_control.

    Args:
        configs: List of configuration vectors (numpy arrays).
        times: List of timestamps in seconds.
        joint_names: ROS2 joint names in order.
        controller_topic: FollowJointTrajectory action topic.
        joint_indices: Indices to extract from each config (default: 0..len(joint_names)).
        positions_only: Leave velocities empty to use the controller's speed setting.
        wait_for_completion: Optional blocking callable(node, result_future).
            It spins node, checks feedback and raises on failure. It must return
            only after confirming that the robot reached its target and stopped.
            A still-pending ROS goal is then canceled and its closure checked.
            The callable owns its waiting deadline; the default wait is 60 seconds.

    Returns:
        True if trajectory executed successfully.

    Example:
        # From your HPP script:
        path = planner.solve()
        timed_path = time_optimizer.optimize(path)
        configs = [
            np.array(timed_path(t)[0])
            for t in np.linspace(0, timed_path.length(), 100)
        ]
        times = list(np.linspace(0, timed_path.length(), 100))

        send_trajectory(
            configs, times,
            joint_names=["shoulder_pan", "shoulder_lift", "elbow", ...],
        )
    """
    # Convert to ROS2 message
    trajectory = configs_to_joint_trajectory(
        configs,
        times,
        joint_names,
        joint_indices=joint_indices,
    )

    if positions_only:
        for point in trajectory.points:
            point.velocities = []

    _ensure_rclpy_initialized()

    node = _TrajectorySenderNode(controller_topic)
    try:
        return node.send_and_wait(trajectory, wait_for_completion=wait_for_completion)
    finally:
        node.client.destroy()
        node.destroy_node()


def send_trajectory_async(
    configs: List[np.ndarray],
    times: List[float],
    joint_names: List[str],
    controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
    joint_indices: Optional[List[int]] = None,
):
    """
    Send trajectory without waiting for completion.

    Returns the goal handle for later status checking.
    Caller is responsible for ROS2 lifecycle (rclpy.init/shutdown).

    Args:
        times: List of timestamps in seconds.
    """
    trajectory = configs_to_joint_trajectory(
        configs,
        times,
        joint_names,
        joint_indices=joint_indices,
    )

    _ensure_rclpy_initialized()

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


def execute_segments(
    segments: List[Segment],
    configs: List[np.ndarray],
    times: List[float],
    joint_names: List[str],
    joint_indices: Optional[List[int]] = None,
    controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
    *,
    pre_actions_by_transition: dict[str, list[Action]] | None = None,
    post_actions_by_transition: dict[str, list[Action]] | None = None,
    positions_only: bool = False,
    wait_for_completion=None,
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
        times: Timestamps in seconds for each config.
        joint_names: ROS2 joint names for the arm.
        joint_indices: Indices of arm DOFs in the HPP config vector.
            Default: 0..len(joint_names).
        controller_topic: FollowJointTrajectory action topic.
        pre_actions_by_transition: Optional mapping from HPP graph transition
            names to ordered lists of actions to run before matching segments.
        post_actions_by_transition: Optional mapping from HPP graph transition
            names to ordered lists of actions to run after matching segments.
        positions_only: Forward positions without velocities to the controller.
        wait_for_completion: Optional callable(node, result_future, segment_configs).
            It follows the send_trajectory completion contract for each segment.
            segment_configs contains the same configuration vectors as configs;
            joint_indices applies only to the sent trajectory.

    Returns:
        True if all segments and actions succeeded.
    """
    if pre_actions_by_transition is None:
        pre_actions_by_transition = {}
    if post_actions_by_transition is None:
        post_actions_by_transition = {}

    for i, segment in enumerate(segments):
        # 1. Pre-actions
        pre_actions = segment.pre_actions + pre_actions_by_transition.get(
            segment.transition_name, []
        )
        for action in pre_actions:
            action_name = _action_name(action)
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
                joint_indices=joint_indices,
                positions_only=positions_only,
                wait_for_completion=(
                    None
                    if wait_for_completion is None
                    else lambda node, result, points=seg_configs: wait_for_completion(
                        node, result, points
                    )
                ),
            )

            if not success:
                logger.error("Segment %d: arm trajectory failed", i)
                return False
        else:
            logger.info("Segment %d: single point, skipping trajectory", i)

        # 3. Post-actions
        post_actions = segment.post_actions + post_actions_by_transition.get(
            segment.transition_name, []
        )
        for action in post_actions:
            action_name = _action_name(action)
            logger.info("Segment %d: running post-action '%s'", i, action_name)
            if not action():
                logger.error("Segment %d: post-action '%s' failed", i, action_name)
                return False

    logger.info("All %d segments completed successfully", len(segments))
    return True
