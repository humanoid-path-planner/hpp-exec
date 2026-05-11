#!/usr/bin/env python3
"""Check that path transition names disambiguate g1 from g1+g2.

This is a pure Python smoke test: no ROS, HPP server, or Gazebo is needed.
It exercises the failure mode where state names alone are not enough to know
which grasp changed.
"""

from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from hpp_exec.gripper import extract_path_grasp_transitions  # noqa: E402


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
    def __init__(self, subpath_lengths: list[float]):
        self._subpaths = [MockSubPath(length) for length in subpath_lengths]
        self._length = sum(subpath_lengths)

    def length(self):
        return self._length

    def numberPaths(self):
        return len(self._subpaths)

    def pathAtRank(self, rank):
        return self._subpaths[rank]

    def __call__(self, param):
        return np.array([float(param)])


class MockConstraintGraph:
    def __init__(self):
        self._intervals = [
            (0.0, 10.0, MockTransition("left/gripper > box1/handle | f_1", "free", "g1")),
            (
                10.0,
                20.0,
                MockTransition("right/gripper > box2/handle | 1_12", "g1", "g1+g2"),
            ),
            (
                20.0,
                30.0,
                MockTransition("right/gripper < box2/handle | 12_1", "g1+g2", "g1"),
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


def main():
    transitions = extract_path_grasp_transitions(
        MockPathVector([10.0, 10.0, 10.0]),
        MockConstraintGraph(),
    )

    for transition in transitions:
        print(
            transition.time,
            transition.state_before,
            "->",
            transition.state_after,
            transition.transition_name,
            "acquired:",
            sorted(transition.acquired),
            "released:",
            sorted(transition.released),
        )

    assert [transition.transition_name for transition in transitions] == [
        "left/gripper > box1/handle | f_1",
        "right/gripper > box2/handle | 1_12",
        "right/gripper < box2/handle | 12_1",
    ]
    assert transitions[1].acquired == {"right/gripper grasps box2/handle"}
    assert transitions[1].released == set()
    assert transitions[2].acquired == set()
    assert transitions[2].released == {"right/gripper grasps box2/handle"}

    print("OK: g1 and g1+g2 are disambiguated by graph transition names.")


if __name__ == "__main__":
    main()
