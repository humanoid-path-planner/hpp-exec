#!/usr/bin/env python3
"""
Test gripper coordination with FR3 in Gazebo.

Simulates a pick-and-place manipulation trajectory:
  1. Move arm to pre-grasp pose (gripper open)     ~3s
  2. STOP - Close gripper                           ~1s
  3. Move arm to place pose (carrying object)       ~4s
  4. STOP - Open gripper                            ~1s
  5. Retreat to home                                ~3s

Uses a MockConstraintGraph to simulate HPP path/graph transition queries,
so this test works without HPP installed.

Prerequisites:
    ./scripts/launch_gazebo_gripper.sh   (in terminal 1)

Usage (in terminal 2):
    PYTHONPATH=$HOME/devel/hpp-exec:$PYTHONPATH python3 ~/devel/hpp-exec/scripts/test_gripper_gazebo.py
"""

# Import from examples/ if needed.
import os
import sys

import numpy as np

from hpp_exec import execute_segments
from hpp_exec.gripper import extract_path_grasp_transitions, segments_from_graph

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "examples"))
from gripper_controllers import JointTrajectoryGripperController

# ---------------------------------------------------------------------------
# Mock constraint graph (no HPP needed)
# ---------------------------------------------------------------------------


class MockTransition:
    def __init__(self, name: str, state_from: str, state_to: str):
        self._name = name
        self.state_from = state_from
        self.state_to = state_to

    def name(self):
        return self._name


class MockSubPath:
    def __init__(self, length: float):
        self._length = length

    def length(self):
        return self._length


class MockPathVector:
    def __init__(self, keyframes):
        self.keyframes = keyframes
        self._subpaths = [
            MockSubPath(keyframes[i + 1][0] - keyframes[i][0])
            for i in range(len(keyframes) - 1)
        ]

    def length(self):
        return self.keyframes[-1][0]

    def numberPaths(self):
        return len(self._subpaths)

    def pathAtRank(self, rank):
        return self._subpaths[rank]

    def __call__(self, param):
        for (start, q0), (end, q1) in zip(self.keyframes, self.keyframes[1:]):
            if start <= param <= end:
                ratio = (param - start) / (end - start)
                arm = q0 + ratio * (q1 - q0)
                return np.concatenate([arm, np.zeros(9)])
        return np.concatenate([self.keyframes[-1][1], np.zeros(9)])


class MockConstraintGraph:
    """Simulates HPP constraint graph for a pick-and-place scenario.

    State transitions:
        s in [0, 3]:   "free"                         (approach)
        s in [3, 7]:   "gripper grasps object/handle"  (carrying)
        s in [7, 10]:  "free"                          (released)
    """

    grasped_state = "gripper grasps object/handle"

    def __init__(self):
        self._intervals = [
            (0.0, 3.0, MockTransition("free loop | approach", "free", "free")),
            (
                3.0,
                7.0,
                MockTransition(
                    "gripper > object/handle | f_01",
                    "free",
                    self.grasped_state,
                ),
            ),
            (
                7.0,
                10.0,
                MockTransition(
                    "gripper < object/handle | 01_f",
                    self.grasped_state,
                    "free",
                ),
            ),
        ]

    def transitionAtParam(self, path, param):
        del path
        for start, end, transition in self._intervals:
            if start <= param <= end:
                return transition
        raise ValueError(f"No transition at path parameter {param}")

    def getNodesConnectedByTransition(self, transition):
        return transition.state_from, transition.state_to


# ---------------------------------------------------------------------------
# Generate a pick-and-place trajectory for FR3
# ---------------------------------------------------------------------------


def generate_fr3_path():
    """Generate a clear pick-and-place path for FR3 (7 DOF arm)."""
    home = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    pregrasp = np.array([0.4, -0.3, 0.3, -1.5, 0.2, 1.2, 0.5])
    place = np.array([-0.4, -0.3, -0.3, -1.5, -0.2, 1.2, -0.5])
    retreat = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

    return MockPathVector(
        [
            (0.0, home),
            (3.0, pregrasp),
            (7.0, place),
            (10.0, retreat),
        ]
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    path = generate_fr3_path()
    graph = MockConstraintGraph()

    # Preview transitions
    transitions = extract_path_grasp_transitions(path, graph)
    print(f"\nFound {len(transitions)} grasp transition(s):")
    for t in transitions:
        action = "CLOSE" if t.acquired else "OPEN"
        print(f"  s={t.time:.2f}: {action} ({t.transition_name})")

    # Gripper controller (matches controllers_gazebo.yaml)
    gripper = JointTrajectoryGripperController(
        topic="/gripper_controller/follow_joint_trajectory",
        joint_names=["fr3_finger_joint1"],
        open_positions=[0.04],
        close_positions=[0.0],
        duration=1.0,
    )

    FR3_ARM_JOINTS = [
        "fr3_joint1",
        "fr3_joint2",
        "fr3_joint3",
        "fr3_joint4",
        "fr3_joint5",
        "fr3_joint6",
        "fr3_joint7",
    ]

    # Build segments from mock path/graph
    configs, times, segments = segments_from_graph(
        path,
        graph,
        on_grasp=gripper.close,
        on_release=gripper.open,
        n_per_unit=20,
    )

    print(f"\nExecuting {len(segments)} segments in Gazebo...")
    print("  Phase 1: Move to pre-grasp pose")
    print("  Phase 2: CLOSE gripper, then move to place pose")
    print("  Phase 3: OPEN gripper, then retreat to home")
    print()

    success = execute_segments(
        segments,
        configs,
        times,
        joint_names=FR3_ARM_JOINTS,
        joint_indices=list(range(7)),
        time_parameterization="trapezoidal",
        max_velocity=0.3,
    )

    print(f"\nResult: {'SUCCESS' if success else 'FAILED'}")
    gripper.destroy()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
