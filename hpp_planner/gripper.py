"""
Gripper coordination for HPP manipulation trajectories.

Detects grasp/release events from HPP constraint graph state transitions
and coordinates arm trajectory execution with gripper open/close commands.

Example:
    from hpp_planner.gripper import (
        execute_manipulation,
        GripperCommandController,
    )

    gripper = GripperCommandController(
        topic="/gripper_controller/gripper_command",
        open_position=0.04,
        close_position=0.0,
    )

    execute_manipulation(
        waypoints, times,
        arm_joint_names=["joint1", "joint2", ...],
        arm_joint_indices=[0, 1, 2, 3, 4, 5],
        gripper_controller=gripper,
        graph=constraint_graph,
    )
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import List, Optional, Protocol

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from hpp_planner.trajectory_utils import waypoints_to_joint_trajectory, add_time_parameterization

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class GraspTransition:
    """A point in the trajectory where the gripper state changes."""

    waypoint_index: int
    """Index in the waypoints list where this transition occurs."""

    time: float
    """Time at this waypoint."""

    grasps_before: set[str]
    """Active grasp names before the transition."""

    grasps_after: set[str]
    """Active grasp names after the transition."""

    acquired: set[str] = field(default_factory=set)
    """New grasps (close gripper)."""

    released: set[str] = field(default_factory=set)
    """Lost grasps (open gripper)."""

    def __post_init__(self):
        self.acquired = self.grasps_after - self.grasps_before
        self.released = self.grasps_before - self.grasps_after


# ---------------------------------------------------------------------------
# Constraint graph state parsing
# ---------------------------------------------------------------------------

def _parse_grasps_from_state_name(state_name: str) -> set[str]:
    """Extract active grasp descriptions from an HPP constraint graph state name.

    HPP state names encode active grasps, e.g.:
        "free"                                    -> set()
        "r_gripper grasps box/handle"             -> {"r_gripper grasps box/handle"}
        "r_gripper grasps box/handle : l_gripper grasps cup/handle"
            -> {"r_gripper grasps box/handle", "l_gripper grasps cup/handle"}

    The separator between multiple grasps is " : " (space-colon-space).
    """
    if not state_name or state_name.strip().lower() == "free":
        return set()

    # HPP uses " : " to separate multiple active grasps in state names
    parts = [p.strip() for p in state_name.split(" : ")]
    return {p for p in parts if "grasps" in p.lower() or "grasp" in p.lower()}


def extract_grasp_transitions(
    waypoints: List[np.ndarray],
    times: List[float],
    graph,
) -> List[GraspTransition]:
    """Detect grasp state changes by querying the HPP constraint graph.

    For each waypoint, queries the constraint graph to determine which grasps
    are active (via graph.getStateFromConfiguration). When the set of active
    grasps changes between consecutive waypoints, a GraspTransition is recorded.

    Args:
        waypoints: HPP configuration vectors along the path.
        times: Corresponding timestamps.
        graph: HPP manipulation constraint graph (pyhpp.manipulation.Graph).
            Must have a getStateFromConfiguration(q) method that returns the
            constraint graph state name for a given configuration.

    Returns:
        Ordered list of GraspTransition at each state change.
    """
    if len(waypoints) < 2:
        return []

    transitions = []
    prev_state = graph.getStateFromConfiguration(waypoints[0])
    prev_grasps = _parse_grasps_from_state_name(prev_state)

    for i in range(1, len(waypoints)):
        state = graph.getStateFromConfiguration(waypoints[i])
        grasps = _parse_grasps_from_state_name(state)

        if grasps != prev_grasps:
            transitions.append(GraspTransition(
                waypoint_index=i,
                time=times[i],
                grasps_before=prev_grasps,
                grasps_after=grasps,
            ))
            prev_grasps = grasps

    return transitions


# ---------------------------------------------------------------------------
# Gripper controller protocol and implementations
# ---------------------------------------------------------------------------

class GripperController(Protocol):
    """Interface for sending gripper open/close commands."""

    def open(self) -> bool:
        """Open the gripper. Returns True on success."""
        ...

    def close(self) -> bool:
        """Close the gripper. Returns True on success."""
        ...


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

        # Wait for result
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


# ---------------------------------------------------------------------------
# Main execution function
# ---------------------------------------------------------------------------

def execute_manipulation(
    waypoints: List[np.ndarray],
    times: List[float],
    arm_joint_names: List[str],
    arm_joint_indices: List[int],
    gripper_controller: GripperController,
    graph,
    max_velocity: float = 1.0,
    max_acceleration: float = 0.5,
    arm_controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
) -> bool:
    """Execute an HPP manipulation trajectory with gripper coordination.

    Splits the trajectory at grasp/release transitions. Between segments,
    sends the appropriate gripper command (open or close), then sends the
    next arm trajectory segment.

    Args:
        waypoints: Full HPP configuration vectors (arm + gripper + objects).
        times: Timestamps for each waypoint.
        arm_joint_names: ROS2 joint names for the arm.
        arm_joint_indices: Indices of arm DOFs in the HPP configuration vector.
        gripper_controller: Controller for sending open/close commands.
        graph: HPP manipulation constraint graph (pyhpp.manipulation.Graph).
        max_velocity: Max joint velocity for time scaling (rad/s).
        max_acceleration: Max joint acceleration for time scaling (rad/s^2).
        arm_controller_topic: FollowJointTrajectory action topic for the arm.

    Returns:
        True if all segments and gripper commands succeeded.
    """
    from hpp_planner.ros2_sender import send_trajectory

    # Find all grasp transitions
    transitions = extract_grasp_transitions(waypoints, times, graph)

    if not transitions:
        # No gripper events — just send the whole trajectory
        logger.info("No grasp transitions found, sending full trajectory")
        return send_trajectory(
            waypoints, times, arm_joint_names,
            controller_topic=arm_controller_topic,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration,
            joint_indices=arm_joint_indices,
        )

    # Build segment boundaries: [0, t1, t2, ..., end]
    split_indices = [0] + [t.waypoint_index for t in transitions] + [len(waypoints)]

    logger.info(
        "Found %d grasp transition(s), splitting into %d segments",
        len(transitions), len(split_indices) - 1,
    )

    for seg_idx in range(len(split_indices) - 1):
        start = split_indices[seg_idx]
        end = split_indices[seg_idx + 1]

        # Send gripper command if a transition precedes this segment
        if seg_idx > 0:
            transition = transitions[seg_idx - 1]

            if transition.acquired:
                logger.info(
                    "Segment %d: closing gripper (acquired: %s)",
                    seg_idx, transition.acquired,
                )
                if not gripper_controller.close():
                    logger.error("Gripper close failed at segment %d", seg_idx)
                    return False

            if transition.released:
                logger.info(
                    "Segment %d: opening gripper (released: %s)",
                    seg_idx, transition.released,
                )
                if not gripper_controller.open():
                    logger.error("Gripper open failed at segment %d", seg_idx)
                    return False

        # Send arm trajectory segment
        seg_waypoints = waypoints[start:end]
        seg_times = times[start:end]

        if len(seg_waypoints) < 2:
            logger.info("Segment %d: single point, skipping", seg_idx)
            continue

        # Normalize times to start from 0 for this segment
        t0 = seg_times[0]
        seg_times = [t - t0 for t in seg_times]

        logger.info(
            "Segment %d: sending %d waypoints (%.2fs)",
            seg_idx, len(seg_waypoints), seg_times[-1],
        )

        success = send_trajectory(
            seg_waypoints, seg_times, arm_joint_names,
            controller_topic=arm_controller_topic,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration,
            joint_indices=arm_joint_indices,
        )

        if not success:
            logger.error("Arm trajectory failed at segment %d", seg_idx)
            return False

    logger.info("Manipulation trajectory completed successfully")
    return True
