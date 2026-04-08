"""
hpp_exec - ROS2 execution utilities for HPP trajectories.

Send HPP-generated trajectories to ros2_control.

Example:
    from hpp_exec import send_trajectory

    # Your HPP script generates configs...
    configs = [...]
    times = [...]

    # Execute on robot (times are path parameters, need rescaling)
    send_trajectory(
        configs, times,
        joint_names=["joint1", "joint2", ...],
        time_parameterization="trapezoidal",
        max_velocity=1.0,
    )
"""

__version__ = "0.1.0"

from hpp_exec.gripper import (
    GraspTransition,
    extract_grasp_transitions,
    segments_from_graph,
)
from hpp_exec.ros2_sender import (
    Segment,
    execute_segments,
    send_trajectory,
    send_trajectory_async,
)
from hpp_exec.trajectory_utils import (
    add_time_parameterization,
    configs_to_joint_trajectory,
    extract_joint_config,
)

__all__ = [
    "send_trajectory",
    "send_trajectory_async",
    "execute_segments",
    "Segment",
    "configs_to_joint_trajectory",
    "add_time_parameterization",
    "extract_joint_config",
    "segments_from_graph",
    "extract_grasp_transitions",
    "GraspTransition",
]
