#!/usr/bin/env python3
"""Check that transition names disambiguate g1 from g1+g2.

This is a pure Python smoke test: no ROS, HPP server, or Gazebo is needed.
It exercises the failure mode where state names alone are not enough to know
which grasp changed.
"""

from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from hpp_exec.gripper import extract_grasp_transitions  # noqa: E402


class MockTransition:
    def __init__(self, name: str, state_from: str, state_to: str):
        self._name = name
        self.state_from = state_from
        self.state_to = state_to

    def name(self):
        return self._name


class MockConstraintGraph:
    def __init__(self, states_by_index, transition_names):
        self.states_by_index = sorted(states_by_index.items())
        self.transition_names = transition_names

    def getStateFromConfiguration(self, config):
        index = int(round(config[0]))
        state = "free"
        for threshold, name in self.states_by_index:
            if index >= threshold:
                state = name
            else:
                break
        return state

    def getTransitions(self):
        return [
            MockTransition(name, state_from, state_to)
            for (state_from, state_to), name in self.transition_names.items()
        ]

    def getNodesConnectedByTransition(self, transition):
        return transition.state_from, transition.state_to


def main():
    configs = [np.array([float(i)]) for i in range(40)]
    times = [float(i) for i in range(40)]

    graph = MockConstraintGraph(
        states_by_index={
            10: "g1",
            20: "g1+g2",
            30: "g1",
        },
        transition_names={
            ("free", "g1"): "left/gripper > box1/handle | f_1",
            ("g1", "g1+g2"): "right/gripper > box2/handle | 1_12",
            ("g1+g2", "g1"): "right/gripper < box2/handle | 12_1",
        },
    )

    transitions = extract_grasp_transitions(configs, times, graph)

    for transition in transitions:
        print(
            transition.config_index,
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
