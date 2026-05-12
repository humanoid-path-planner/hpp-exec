"""
Example gripper controller implementations.

These are robot-specific; adapt or replace them for your setup.
Any class with open() -> bool and close() -> bool works with
segments_from_graph() and execute_segments().
"""

import logging
from typing import List, Optional

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

logger = logging.getLogger(__name__)


class GripperCommandController:
    """Gripper controller using control_msgs/action/GripperCommand.

    Works with standard parallel-jaw grippers controlled via the
    GripperCommand action interface.

    Args:
        topic: Action server topic (e.g. "/gripper_controller/gripper_command").
        open_position: Gripper position when open (meters or radians).
        close_position: Gripper position when closed.
        max_effort: Maximum gripper effort (0 = no limit).
    """

    def __init__(
        self,
        topic: str = "/gripper_controller/gripper_command",
        open_position: float = 0.04,
        close_position: float = 0.0,
        max_effort: float = 0.0,
    ):
        self.topic = topic
        self.open_position = open_position
        self.close_position = close_position
        self.max_effort = max_effort
        self._node: Optional[Node] = None
        self._client: Optional[ActionClient] = None

    def _ensure_client(self):
        if self._node is None:
            if not rclpy.ok():
                rclpy.init()
            self._node = Node("hpp_gripper_command_controller")
            self._client = ActionClient(self._node, GripperCommand, self.topic)

    def _send_command(self, position: float) -> bool:
        self._ensure_client()

        if not self._client.wait_for_server(timeout_sec=10.0):
            logger.error("Gripper action server not available: %s", self.topic)
            return False

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = self.max_effort

        logger.info("Sending gripper command: position=%.4f", position)
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            logger.error("Gripper command rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=30.0)
        logger.info("Gripper command completed")
        return True

    def open(self) -> bool:
        return self._send_command(self.open_position)

    def close(self) -> bool:
        return self._send_command(self.close_position)

    def destroy(self):
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
            self._client = None


class JointTrajectoryGripperController:
    """Gripper controller using FollowJointTrajectory action.

    For grippers controlled as joint trajectory controllers (e.g. via
    ros2_control JointTrajectoryController).

    Args:
        topic: Action server topic (e.g. "/gripper_controller/follow_joint_trajectory").
        joint_names: Gripper joint names.
        open_positions: Joint positions when open.
        close_positions: Joint positions when closed.
        duration: Time to complete the gripper motion (seconds).
    """

    def __init__(
        self,
        topic: str = "/gripper_controller/follow_joint_trajectory",
        joint_names: Optional[List[str]] = None,
        open_positions: Optional[List[float]] = None,
        close_positions: Optional[List[float]] = None,
        duration: float = 1.0,
    ):
        self.topic = topic
        self.joint_names = joint_names or ["gripper_finger_joint"]
        self.open_positions = open_positions or [0.04]
        self.close_positions = close_positions or [0.0]
        self.duration = duration
        self._node: Optional[Node] = None
        self._client: Optional[ActionClient] = None

    def _ensure_client(self):
        if self._node is None:
            if not rclpy.ok():
                rclpy.init()
            self._node = Node("hpp_gripper_trajectory_controller")
            self._client = ActionClient(self._node, FollowJointTrajectory, self.topic)

    def _send_positions(self, positions: List[float]) -> bool:
        self._ensure_client()

        if not self._client.wait_for_server(timeout_sec=10.0):
            logger.error("Gripper trajectory server not available: %s", self.topic)
            return False

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.time_from_start = Duration(
            sec=int(self.duration),
            nanosec=int((self.duration - int(self.duration)) * 1e9),
        )
        trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        logger.info("Sending gripper trajectory: %s -> %s", self.joint_names, positions)
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            logger.error("Gripper trajectory rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=30.0)
        logger.info("Gripper trajectory completed")
        return True

    def open(self) -> bool:
        return self._send_positions(self.open_positions)

    def close(self) -> bool:
        return self._send_positions(self.close_positions)

    def destroy(self):
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
            self._client = None


class FrankaGripperController:
    """Franka-native gripper using franka_msgs Grasp/Move actions.

    Uses franka_msgs/action/Move for opening (position control)
    and franka_msgs/action/Grasp for closing (force-controlled grasp).

    Only works with real Franka hardware (not Gazebo).
    For Gazebo, use JointTrajectoryGripperController instead.

    Args:
        arm_id: Robot arm identifier (e.g. "fr3", "panda").
        open_width: Finger width when open (meters). Default 0.08 (fully open).
        grasp_width: Target finger width when grasping (meters).
        grasp_speed: Gripper closing speed (m/s).
        grasp_force: Grasping force (Newtons).
        grasp_epsilon_inner: Inner tolerance for grasp success (meters).
        grasp_epsilon_outer: Outer tolerance for grasp success (meters).
        move_speed: Gripper opening speed (m/s).
    """

    def __init__(
        self,
        arm_id: str = "fr3",
        open_width: float = 0.08,
        grasp_width: float = 0.02,
        grasp_speed: float = 0.05,
        grasp_force: float = 50.0,
        grasp_epsilon_inner: float = 0.01,
        grasp_epsilon_outer: float = 0.01,
        move_speed: float = 0.05,
    ):
        self.arm_id = arm_id
        self.open_width = open_width
        self.grasp_width = grasp_width
        self.grasp_speed = grasp_speed
        self.grasp_force = grasp_force
        self.grasp_epsilon_inner = grasp_epsilon_inner
        self.grasp_epsilon_outer = grasp_epsilon_outer
        self.move_speed = move_speed
        self._node: Optional[Node] = None
        self._grasp_client: Optional[ActionClient] = None
        self._move_client: Optional[ActionClient] = None

    def _ensure_clients(self):
        if self._node is not None:
            return

        from franka_msgs.action import Grasp, Move

        if not rclpy.ok():
            rclpy.init()
        self._node = Node("franka_gripper_controller")
        self._grasp_client = ActionClient(
            self._node, Grasp, f"/{self.arm_id}_gripper/grasp"
        )
        self._move_client = ActionClient(
            self._node, Move, f"/{self.arm_id}_gripper/move"
        )

    def open(self) -> bool:
        """Open gripper to open_width using Move action."""
        self._ensure_clients()

        from franka_msgs.action import Move

        if not self._move_client.wait_for_server(timeout_sec=10.0):
            logger.error("Move action server not available")
            return False

        goal = Move.Goal()
        goal.width = self.open_width
        goal.speed = self.move_speed

        logger.info(
            "Opening gripper: width=%.3fm, speed=%.3fm/s",
            self.open_width,
            self.move_speed,
        )

        future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            logger.error("Move goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=30.0)
        logger.info("Gripper opened")
        return True

    def close(self) -> bool:
        """Close gripper using Grasp action (force-controlled)."""
        self._ensure_clients()

        from franka_msgs.action import Grasp

        if not self._grasp_client.wait_for_server(timeout_sec=10.0):
            logger.error("Grasp action server not available")
            return False

        goal = Grasp.Goal()
        goal.width = self.grasp_width
        goal.speed = self.grasp_speed
        goal.force = self.grasp_force
        goal.epsilon.inner = self.grasp_epsilon_inner
        goal.epsilon.outer = self.grasp_epsilon_outer

        logger.info(
            "Grasping: width=%.3fm, force=%.1fN",
            self.grasp_width,
            self.grasp_force,
        )

        future = self._grasp_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            logger.error("Grasp goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=30.0)
        logger.info("Grasp completed")
        return True

    def destroy(self):
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
            self._grasp_client = None
            self._move_client = None
