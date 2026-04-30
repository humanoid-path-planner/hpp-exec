#!/usr/bin/env python3
"""
Unit tests for segment-based execution logic.

No ROS, no HPP, no Docker needed — runs on host with just numpy.

Usage:
    python3 -m pytest tests/test_segments.py -v
    # or simply:
    python3 tests/test_segments.py
"""

import os
import sys

import numpy as np

# Allow running from repo root without install
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


# ---------------------------------------------------------------------------
# Mock constraint graph (same pattern as test_gripper_gazebo.py)
# ---------------------------------------------------------------------------


class MockTransition:
    def __init__(self, name: str, state_from: str, state_to: str):
        self._name = name
        self.state_from = state_from
        self.state_to = state_to

    def name(self):
        return self._name


class MockConstraintGraph:
    """Simulates HPP constraint graph state queries."""

    def __init__(
        self,
        transitions: dict[int, str],
        transition_names: dict[tuple[str, str], str] | None = None,
    ):
        """
        Args:
            transitions: {config_index: state_name} mapping.
                Waypoints before the first key are "free".
            transition_names: optional {(state_from, state_to): transition_name}
                mapping, used to simulate pyhpp graph transition metadata.
        """
        self.transitions = sorted(transitions.items())
        self.transition_names = transition_names or {}

    def getStateFromConfiguration(self, config):
        # Use first element as config identifier
        idx = int(round(config[0]))
        state = "free"
        for threshold, name in self.transitions:
            if idx >= threshold:
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
        return (transition.state_from, transition.state_to)


# ---------------------------------------------------------------------------
# Tests for extract_grasp_transitions
# ---------------------------------------------------------------------------


def test_no_transitions():
    """All configs in same state → no transitions."""
    from hpp_exec.gripper import extract_grasp_transitions

    configs = [np.array([float(i)]) for i in range(10)]
    times = list(range(10))
    graph = MockConstraintGraph({})  # always "free"

    transitions = extract_grasp_transitions(configs, times, graph)
    assert transitions == []


def test_single_grasp():
    """free → grasped → should produce one transition."""
    from hpp_exec.gripper import extract_grasp_transitions

    configs = [np.array([float(i)]) for i in range(10)]
    times = [float(i) for i in range(10)]
    graph = MockConstraintGraph({5: "gripper grasps box/handle"})

    transitions = extract_grasp_transitions(configs, times, graph)
    assert len(transitions) == 1
    assert transitions[0].config_index == 5
    assert transitions[0].acquired == {"gripper grasps box/handle"}
    assert transitions[0].released == set()
    assert transitions[0].state_before == "free"
    assert transitions[0].state_after == "gripper grasps box/handle"


def test_grasp_and_release():
    """free → grasped → free → two transitions."""
    from hpp_exec.gripper import extract_grasp_transitions

    configs = [np.array([float(i)]) for i in range(30)]
    times = [float(i) for i in range(30)]
    graph = MockConstraintGraph(
        {
            10: "gripper grasps box/handle",
            20: "free",
        }
    )

    transitions = extract_grasp_transitions(configs, times, graph)
    assert len(transitions) == 2

    # First: grasp acquired
    assert transitions[0].config_index == 10
    assert transitions[0].acquired == {"gripper grasps box/handle"}
    assert transitions[0].released == set()

    # Second: grasp released
    assert transitions[1].config_index == 20
    assert transitions[1].acquired == set()
    assert transitions[1].released == {"gripper grasps box/handle"}


def test_transition_name_disambiguates_combined_state():
    """g1 -> g1+g2 should be identified by the transition that acquired g2."""
    from hpp_exec.gripper import extract_grasp_transitions

    configs = [np.array([float(i)]) for i in range(30)]
    times = [float(i) for i in range(30)]
    graph = MockConstraintGraph(
        {
            10: "g1",
            20: "g1+g2",
        },
        transition_names={
            ("free", "g1"): "left/gripper > box1/handle | f_01",
            ("g1", "g1+g2"): "right/gripper > box2/handle | 1_12",
        },
    )

    transitions = extract_grasp_transitions(configs, times, graph)
    assert len(transitions) == 2

    assert transitions[0].transition_name == "left/gripper > box1/handle | f_01"
    assert transitions[0].acquired == {"left/gripper grasps box1/handle"}
    assert transitions[0].released == set()

    assert transitions[1].state_before == "g1"
    assert transitions[1].state_after == "g1+g2"
    assert transitions[1].transition_name == "right/gripper > box2/handle | 1_12"
    assert transitions[1].acquired == {"right/gripper grasps box2/handle"}
    assert transitions[1].released == set()


def test_transition_name_disambiguates_release_from_combined_state():
    """g1+g2 -> g1 should be identified by the transition that released g2."""
    from hpp_exec.gripper import extract_grasp_transitions

    configs = [np.array([float(i)]) for i in range(30)]
    times = [float(i) for i in range(30)]
    graph = MockConstraintGraph(
        {
            10: "g1+g2",
            20: "g1",
        },
        transition_names={
            ("free", "g1+g2"): "left/gripper > box1/handle | f_01",
            ("g1+g2", "g1"): "right/gripper < box2/handle | 12_1",
        },
    )

    transitions = extract_grasp_transitions(configs, times, graph)
    assert len(transitions) == 2
    assert transitions[1].transition_name == "right/gripper < box2/handle | 12_1"
    assert transitions[1].acquired == set()
    assert transitions[1].released == {"right/gripper grasps box2/handle"}


def test_transition_name_parsing_helpers():
    """Transition names produce stable acquired/released grasp labels."""
    from hpp_exec.gripper import _parse_grasp_event_from_transition_name

    acquired, released = _parse_grasp_event_from_transition_name(
        "left/gripper > box1/handle | f_01"
    )
    assert acquired == {"left/gripper grasps box1/handle"}
    assert released == set()

    acquired, released = _parse_grasp_event_from_transition_name(
        "right/gripper < box2/handle | 12_1"
    )
    assert acquired == set()
    assert released == {"right/gripper grasps box2/handle"}

    acquired, released = _parse_grasp_event_from_transition_name("free loop | f")
    assert acquired == set()
    assert released == set()


def test_fallback_keeps_state_diff_when_transition_metadata_is_absent():
    """Old/mock graphs without transition metadata still work."""
    from hpp_exec.gripper import extract_grasp_transitions

    configs = [np.array([float(i)]) for i in range(12)]
    times = [float(i) for i in range(12)]
    graph = MockConstraintGraph({6: "gripper grasps box/handle"})

    transitions = extract_grasp_transitions(configs, times, graph)
    assert len(transitions) == 1
    assert transitions[0].transition_name is None
    assert transitions[0].acquired == {"gripper grasps box/handle"}
    assert transitions[0].released == set()


def test_empty_configs():
    from hpp_exec.gripper import extract_grasp_transitions

    assert extract_grasp_transitions([], [], MockConstraintGraph({})) == []


def test_single_config():
    from hpp_exec.gripper import extract_grasp_transitions

    assert (
        extract_grasp_transitions([np.array([0.0])], [0.0], MockConstraintGraph({}))
        == []
    )


# ---------------------------------------------------------------------------
# Tests for segments_from_graph
# ---------------------------------------------------------------------------


def test_segments_no_transitions():
    """No grasp changes → single segment covering all configs."""
    from hpp_exec.gripper import segments_from_graph

    configs = [np.array([float(i)]) for i in range(10)]
    times = [float(i) for i in range(10)]
    graph = MockConstraintGraph({})

    segments = segments_from_graph(
        configs,
        times,
        graph,
        on_grasp=lambda: True,
        on_release=lambda: True,
    )
    assert len(segments) == 1
    assert segments[0].start_index == 0
    assert segments[0].end_index == 10
    assert segments[0].pre_actions == []


def test_segments_pick_and_place():
    """Grasp at 10, release at 20 → 3 segments with correct actions."""
    from hpp_exec.gripper import segments_from_graph

    configs = [np.array([float(i)]) for i in range(30)]
    times = [float(i) for i in range(30)]
    graph = MockConstraintGraph(
        {
            10: "gripper grasps box/handle",
            20: "free",
        }
    )

    close_called = []
    open_called = []

    def mock_close():
        close_called.append(True)
        return True

    def mock_open():
        open_called.append(True)
        return True

    segments = segments_from_graph(
        configs,
        times,
        graph,
        on_grasp=mock_close,
        on_release=mock_open,
    )

    assert len(segments) == 3

    # Segment 0: approach (no pre-actions)
    assert segments[0].start_index == 0
    assert segments[0].end_index == 10
    assert segments[0].pre_actions == []

    # Segment 1: carry (close gripper before)
    assert segments[1].start_index == 10
    assert segments[1].end_index == 20
    assert len(segments[1].pre_actions) == 1

    # Segment 2: retreat (open gripper before)
    assert segments[2].start_index == 20
    assert segments[2].end_index == 30
    assert len(segments[2].pre_actions) == 1

    # Execute pre-actions to verify correct callbacks
    segments[1].pre_actions[0]()
    assert len(close_called) == 1
    assert len(open_called) == 0

    segments[2].pre_actions[0]()
    assert len(close_called) == 1
    assert len(open_called) == 1


def test_segments_accept_transition_aware_action():
    """on_grasp/on_release may inspect the unique graph transition."""
    from hpp_exec.gripper import segments_from_graph

    configs = [np.array([float(i)]) for i in range(20)]
    times = [float(i) for i in range(20)]
    graph = MockConstraintGraph(
        {10: "g1"},
        transition_names={
            ("free", "g1"): "left/gripper > box1/handle | f_01",
        },
    )
    seen = []

    def on_grasp(transition):
        seen.append(transition.transition_name)
        return True

    segments = segments_from_graph(
        configs,
        times,
        graph,
        on_grasp=on_grasp,
        on_release=lambda: True,
    )

    segments[1].pre_actions[0]()
    assert seen == ["left/gripper > box1/handle | f_01"]


def test_segments_accept_action_mapping_by_transition_name():
    """Transition-name mappings let different grasps run different actions."""
    from hpp_exec.gripper import segments_from_graph

    configs = [np.array([float(i)]) for i in range(30)]
    times = [float(i) for i in range(30)]
    graph = MockConstraintGraph(
        {
            10: "g1",
            20: "g1+g2",
        },
        transition_names={
            ("free", "g1"): "left/gripper > box1/handle | f_01",
            ("g1", "g1+g2"): "right/gripper > box2/handle | 1_12",
        },
    )
    calls = []

    segments = segments_from_graph(
        configs,
        times,
        graph,
        on_grasp={
            "left/gripper > box1/handle | f_01": (
                lambda: calls.append("left") or True
            ),
            "right/gripper > box2/handle | 1_12": (
                lambda: calls.append("right") or True
            ),
        },
        on_release=lambda: True,
    )

    segments[1].pre_actions[0]()
    segments[2].pre_actions[0]()
    assert calls == ["left", "right"]


def test_segments_accept_action_mapping_by_grasp_label():
    """Action maps can be keyed by parsed grasp label, not only edge name."""
    from hpp_exec.gripper import segments_from_graph

    configs = [np.array([float(i)]) for i in range(20)]
    times = [float(i) for i in range(20)]
    graph = MockConstraintGraph(
        {10: "g1"},
        transition_names={
            ("free", "g1"): "left/gripper > box1/handle | f_01",
        },
    )
    calls = []

    segments = segments_from_graph(
        configs,
        times,
        graph,
        on_grasp={
            "left/gripper grasps box1/handle": lambda: calls.append("left") or True,
        },
        on_release=lambda: True,
    )

    segments[1].pre_actions[0]()
    assert calls == ["left"]


def test_segments_accept_action_mapping_by_state_pair():
    """State-pair keys cover custom transition names with state-name fallback."""
    from hpp_exec.gripper import segments_from_graph

    configs = [np.array([float(i)]) for i in range(20)]
    times = [float(i) for i in range(20)]
    graph = MockConstraintGraph(
        {10: "gripper grasps box/handle"},
        transition_names={
            ("free", "gripper grasps box/handle"): "custom transition",
        },
    )
    calls = []

    segments = segments_from_graph(
        configs,
        times,
        graph,
        on_grasp={
            "free -> gripper grasps box/handle": (
                lambda: calls.append("state-pair") or True
            ),
        },
        on_release=lambda: True,
    )

    segments[1].pre_actions[0]()
    assert calls == ["state-pair"]


def test_segments_action_mapping_missing_key_raises():
    """A mapping should fail loudly when no key matches the graph transition."""
    from hpp_exec.gripper import segments_from_graph

    configs = [np.array([float(i)]) for i in range(20)]
    times = [float(i) for i in range(20)]
    graph = MockConstraintGraph(
        {10: "g1"},
        transition_names={
            ("free", "g1"): "left/gripper > box1/handle | f_01",
        },
    )

    try:
        segments_from_graph(
            configs,
            times,
            graph,
            on_grasp={"right/gripper > box2/handle | f_01": lambda: True},
            on_release=lambda: True,
        )
    except KeyError as exc:
        assert "No action found" in str(exc)
    else:
        raise AssertionError("segments_from_graph should fail for an unknown action key")


# ---------------------------------------------------------------------------
# Tests for Segment dataclass
# ---------------------------------------------------------------------------


def test_segment_defaults():
    from hpp_exec.ros2_sender import Segment

    seg = Segment(0, 100)
    assert seg.start_index == 0
    assert seg.end_index == 100
    assert seg.pre_actions == []
    assert seg.post_actions == []


def test_segment_with_actions():
    from hpp_exec.ros2_sender import Segment

    actions_run = []
    seg = Segment(
        10,
        20,
        pre_actions=[lambda: actions_run.append("pre") or True],
        post_actions=[lambda: actions_run.append("post") or True],
    )

    seg.pre_actions[0]()
    seg.post_actions[0]()
    assert actions_run == ["pre", "post"]


# ---------------------------------------------------------------------------
# Tests for state name parsing
# ---------------------------------------------------------------------------


def test_parse_free():
    from hpp_exec.gripper import _parse_grasps_from_state_name

    assert _parse_grasps_from_state_name("free") == set()
    assert _parse_grasps_from_state_name("Free") == set()
    assert _parse_grasps_from_state_name("") == set()


def test_parse_single_grasp():
    from hpp_exec.gripper import _parse_grasps_from_state_name

    result = _parse_grasps_from_state_name("r_gripper grasps box/handle")
    assert result == {"r_gripper grasps box/handle"}


def test_parse_multi_grasp():
    from hpp_exec.gripper import _parse_grasps_from_state_name

    result = _parse_grasps_from_state_name(
        "r_gripper grasps box/handle : l_gripper grasps cup/handle"
    )
    assert result == {"r_gripper grasps box/handle", "l_gripper grasps cup/handle"}


# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    test_funcs = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    passed = 0
    failed = 0
    for func in test_funcs:
        try:
            func()
            print(f"  PASS  {func.__name__}")
            passed += 1
        except Exception as e:
            print(f"  FAIL  {func.__name__}: {e}")
            failed += 1

    print(f"\n{passed} passed, {failed} failed")
    sys.exit(1 if failed else 0)
