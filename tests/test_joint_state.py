"""Tests for joint state helpers."""

import numpy as np
import pytest


def test_joint_state_to_config_orders_and_strips_gazebo_prefix():
    pytest.importorskip("sensor_msgs")

    from sensor_msgs.msg import JointState

    from hpp_exec.joint_state import joint_state_to_config

    message = JointState()
    message.name = ["robot::joint_2", "robot::joint_1"]
    message.position = [2.0, 1.0]

    np.testing.assert_allclose(
        joint_state_to_config(
            message,
            ["joint_1", "joint_2"],
            strip_prefix=True,
        ),
        [1.0, 2.0],
    )
