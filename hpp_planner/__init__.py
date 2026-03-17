"""
hpp_planner - ROS2 execution utilities for HPP trajectories.

Send HPP-generated trajectories to ros2_control.

Example:
    from hpp_planner import send_trajectory

    # Your HPP script generates waypoints...
    waypoints = [...]
    times = [...]

    # Execute on robot
    send_trajectory(
        waypoints, times,
        joint_names=["joint1", "joint2", ...],
        max_velocity=1.0,
    )
"""

__version__ = "0.1.0"

from hpp_planner.trajectory_utils import (
    waypoints_to_joint_trajectory,
    add_time_parameterization,
    extract_joint_config,
)
from hpp_planner.ros2_sender import send_trajectory, send_trajectory_async

__all__ = [
    "send_trajectory",
    "send_trajectory_async",
    "waypoints_to_joint_trajectory",
    "add_time_parameterization",
    "extract_joint_config",
]
