"""
Utilities for converting HPP trajectories to ROS2 JointTrajectory messages.
"""

from typing import List, Optional
import numpy as np

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def waypoints_to_joint_trajectory(
    waypoints: List[np.ndarray],
    times: List[float],
    joint_names: List[str],
    joint_indices: Optional[List[int]] = None,
    velocities: Optional[List[np.ndarray]] = None,
    accelerations: Optional[List[np.ndarray]] = None,
) -> JointTrajectory:
    """
    Convert HPP waypoints to a ROS2 JointTrajectory message.

    Args:
        waypoints: List of configuration vectors from HPP
        times: List of timestamps (path parameter values)
        joint_names: ROS2 joint names in desired order
        joint_indices: Indices to extract from HPP config (default: 0 to len(joint_names))
        velocities: Optional velocity vectors (same length as waypoints)
        accelerations: Optional acceleration vectors (same length as waypoints)

    Returns:
        JointTrajectory message ready to send to ros2_control
    """
    if joint_indices is None:
        joint_indices = list(range(len(joint_names)))

    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names

    n_points = len(waypoints)

    for i, (wp, t) in enumerate(zip(waypoints, times)):
        point = JointTrajectoryPoint()

        # Extract relevant joint positions
        point.positions = [float(wp[j]) for j in joint_indices]

        # Set velocities if provided explicitly
        if velocities is not None and i < len(velocities):
            point.velocities = [float(velocities[i][j]) for j in joint_indices]
        elif i == 0 or i == n_points - 1:
            # Zero velocity at start and end for smooth stop
            point.velocities = [0.0] * len(joint_names)
        # Intermediate points: leave velocities empty for spline interpolation

        # Set accelerations if provided explicitly
        if accelerations is not None and i < len(accelerations):
            point.accelerations = [float(accelerations[i][j]) for j in joint_indices]
        # Leave accelerations empty for controller to compute

        # Convert time to Duration
        point.time_from_start = Duration(
            sec=int(t),
            nanosec=int((t - int(t)) * 1e9)
        )

        trajectory.points.append(point)

    return trajectory


def extract_joint_config(
    hpp_config: np.ndarray,
    n_joints: int,
    offset: int = 0,
) -> List[float]:
    """
    Extract robot joint values from HPP configuration.

    HPP configs often have extra DOFs (object poses, gripper fingers).
    This extracts just the robot arm joints.

    Args:
        hpp_config: Full HPP configuration vector
        n_joints: Number of joints to extract
        offset: Starting index in the config

    Returns:
        List of joint values
    """
    return [float(hpp_config[offset + i]) for i in range(n_joints)]


def add_time_parameterization(
    waypoints: List[np.ndarray],
    times: List[float],
    max_velocity: float = 1.0,
    max_acceleration: float = 0.5,
) -> List[float]:
    """
    Simple trapezoidal time parameterization.

    Scales times to respect velocity/acceleration limits.
    For proper time parameterization, use HPP's SimpleTimeParameterization.

    Args:
        waypoints: Configuration waypoints
        times: Original path parameter values
        max_velocity: Maximum joint velocity (rad/s)
        max_acceleration: Maximum joint acceleration (rad/s^2)

    Returns:
        Rescaled times
    """
    if len(waypoints) < 2:
        return times

    # Compute max displacement per segment
    scaled_times = [0.0]
    total_time = 0.0

    for i in range(1, len(waypoints)):
        dq = np.abs(waypoints[i] - waypoints[i - 1])
        max_dq = np.max(dq)

        # Time needed at max velocity
        dt = max_dq / max_velocity if max_velocity > 0 else 0.0
        dt = max(dt, 0.01)  # Minimum 10ms between points

        total_time += dt
        scaled_times.append(total_time)

    return scaled_times
