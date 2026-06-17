"""Read ROS 2 joint states as ordered configuration vectors."""

import time
from typing import Sequence

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def joint_state_to_config(
    message: JointState,
    joint_names: Sequence[str],
    *,
    strip_prefix: bool = False,
) -> np.ndarray:
    """Return joint positions in the requested order."""
    positions = {
        name.split("::")[-1] if strip_prefix else name: value
        for name, value in zip(message.name, message.position)
    }
    return np.array([positions[joint] for joint in joint_names])


def read_current_configuration(
    node: Node,
    joint_names: Sequence[str],
    topic: str = "/joint_states",
    timeout_sec: float = 10.0,
    *,
    strip_prefix: bool = False,
    require_single_publisher: bool = False,
) -> np.ndarray | None:
    """Wait for one JointState message and return joint positions in order."""
    if require_single_publisher:
        publishers = node.count_publishers(topic)
        if publishers > 1:
            raise RuntimeError(f"{publishers} publishers on {topic}")

    messages = []
    subscription = node.create_subscription(JointState, topic, messages.append, 10)
    deadline = time.monotonic() + timeout_sec
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if messages:
                return joint_state_to_config(
                    messages[-1], joint_names, strip_prefix=strip_prefix
                )
    finally:
        node.destroy_subscription(subscription)
    return None
