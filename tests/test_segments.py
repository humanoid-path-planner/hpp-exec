"""
Unit tests for segment-based execution logic.
"""

import numpy as np


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
    """Small stand-in for an HPP PathVector made of fixed-length subpaths."""

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


class PathAwareGraph:
    """Mock graph returning the transition that owns a path parameter window."""

    def __init__(self, intervals):
        self.queried_params = []
        self._intervals = intervals

    def transitionAtParam(self, path, param):
        del path
        self.queried_params.append(param)
        for start, end, transition in self._intervals:
            if start <= param <= end:
                return transition
        raise ValueError(f"No transition at path parameter {param}")

    def getNodesConnectedByTransition(self, transition):
        return (transition.state_from, transition.state_to)


def windowed_pick_place_graph():
    grasped_state = "gripper grasps box/handle"
    return PathAwareGraph(
        [
            (0.0, 4.8, MockTransition("free loop | approach", "free", "free")),
            (
                4.8,
                5.2,
                MockTransition(
                    "gripper > box/handle | f_01",
                    "free",
                    grasped_state,
                ),
            ),
            (
                5.2,
                10.0,
                MockTransition(
                    "gripper < box/handle | 01_f",
                    grasped_state,
                    "free",
                ),
            ),
        ]
    )


def two_grasp_graph():
    return PathAwareGraph(
        [
            (0.0, 10.0, MockTransition("free loop | f", "free", "free")),
            (
                10.0,
                20.0,
                MockTransition("left/gripper > box1/handle | f_1", "free", "g1"),
            ),
            (
                20.0,
                30.0,
                MockTransition("right/gripper > box2/handle | 1_12", "g1", "g1+g2"),
            ),
            (
                30.0,
                40.0,
                MockTransition("right/gripper < box2/handle | 12_1", "g1+g2", "g1"),
            ),
        ]
    )


# ---------------------------------------------------------------------------
# Tests for graph-derived transition splitting
# ---------------------------------------------------------------------------


def test_segments_no_grasp_transitions():
    """A free-loop path stays one segment after adjacent intervals are merged."""
    from hpp_exec.gripper import segments_from_graph

    path = MockPathVector([3.0, 7.0])
    graph = PathAwareGraph(
        [
            (0.0, 3.0, MockTransition("free loop | f", "free", "free")),
            (3.0, 10.0, MockTransition("free loop | f", "free", "free")),
        ]
    )

    configs, times, segments = segments_from_graph(
        path,
        graph,
        on_grasp=lambda: True,
        on_release=lambda: True,
        sample_params=[0.0, 10.0],
    )

    assert graph.queried_params == [1.5, 6.5]
    assert times == [0.0, 10.0]
    assert np.allclose([config[0] for config in configs], times)
    assert len(segments) == 1
    assert segments[0].start_index == 0
    assert segments[0].end_index == 2
    assert segments[0].pre_actions == []


def test_segments_from_graph_inserts_transition_boundaries():
    """Continuous path intervals are sampled at their exact graph boundaries."""
    from hpp_exec.gripper import segments_from_graph

    path = MockPathVector([4.8, 0.4, 4.8])
    graph = windowed_pick_place_graph()
    seen = []

    def on_grasp(transition):
        seen.append(("grasp", transition.transition_name, transition.time))
        return True

    def on_release(transition):
        seen.append(("release", transition.transition_name, transition.time))
        return True

    configs, times, segments = segments_from_graph(
        path,
        graph,
        on_grasp=on_grasp,
        on_release=on_release,
        sample_params=[0.0, 10.0],
    )

    assert graph.queried_params == [2.4, 5.0, 7.6]
    assert times == [0.0, 4.8, 5.2, 10.0]
    assert np.allclose([config[0] for config in configs], times)

    assert len(segments) == 3
    assert [(seg.start_index, seg.end_index) for seg in segments] == [
        (0, 2),
        (1, 3),
        (2, 4),
    ]
    assert [len(seg.pre_actions) for seg in segments] == [0, 1, 1]

    for segment in segments:
        for action in segment.pre_actions:
            action()

    assert seen == [
        ("grasp", "gripper > box/handle | f_01", 4.8),
        ("release", "gripper < box/handle | 01_f", 5.2),
    ]


def test_graph_factory_transient_states_do_not_trigger_actions():
    """HPP factory pregrasp/preplace states use >/< but are not gripper events."""
    from hpp_exec.gripper import segments_from_graph

    path = MockPathVector([1.0, 1.0, 1.0])
    graph = PathAwareGraph(
        [
            (
                0.0,
                1.0,
                MockTransition(
                    "gripper > box/handle | f_01",
                    "free",
                    "gripper > box/handle | f_pregrasp",
                ),
            ),
            (
                1.0,
                2.0,
                MockTransition(
                    "gripper > box/handle | f_12",
                    "gripper > box/handle | f_pregrasp",
                    "gripper > box/handle | f_preplace",
                ),
            ),
            (
                2.0,
                3.0,
                MockTransition(
                    "gripper > box/handle | f_23",
                    "gripper > box/handle | f_preplace",
                    "gripper grasps box/handle",
                ),
            ),
        ]
    )

    _, _, segments = segments_from_graph(
        path,
        graph,
        on_grasp=lambda: True,
        on_release=lambda: True,
        sample_params=[0.0, 3.0],
    )

    assert [(segment.start_index, segment.end_index) for segment in segments] == [
        (0, 3),
        (2, 4),
    ]
    assert [len(segment.pre_actions) for segment in segments] == [0, 1]


def test_extract_path_grasp_transitions_reports_path_events():
    from hpp_exec.gripper import extract_path_grasp_transitions

    transitions = extract_path_grasp_transitions(
        MockPathVector([4.8, 0.4, 4.8]),
        windowed_pick_place_graph(),
    )

    assert [transition.config_index for transition in transitions] == [-1, -1]
    assert [transition.time for transition in transitions] == [4.8, 5.2]
    assert [transition.transition_name for transition in transitions] == [
        "gripper > box/handle | f_01",
        "gripper < box/handle | 01_f",
    ]
    assert transitions[0].acquired == {"gripper grasps box/handle"}
    assert transitions[0].released == set()
    assert transitions[1].acquired == set()
    assert transitions[1].released == {"gripper grasps box/handle"}


def test_segments_accept_transition_aware_action():
    """on_grasp/on_release may inspect the graph transition."""
    from hpp_exec.gripper import segments_from_graph

    seen = []

    def on_grasp(transition):
        seen.append((transition.transition_name, transition.time))
        return True

    configs, times, segments = segments_from_graph(
        MockPathVector([10.0, 10.0]),
        PathAwareGraph(
            [
                (0.0, 10.0, MockTransition("free loop | f", "free", "free")),
                (
                    10.0,
                    20.0,
                    MockTransition("left/gripper > box1/handle | f_1", "free", "g1"),
                ),
            ]
        ),
        on_grasp=on_grasp,
        on_release=lambda: True,
        sample_params=[0.0, 20.0],
    )

    assert times == [0.0, 10.0, 20.0]
    assert np.allclose([config[0] for config in configs], times)
    segments[1].pre_actions[0]()
    assert seen == [("left/gripper > box1/handle | f_1", 10.0)]


def test_segments_accept_action_mapping_by_transition_name():
    """Transition-name mappings let different grasps run different actions."""
    from hpp_exec.gripper import segments_from_graph

    calls = []
    _, _, segments = segments_from_graph(
        MockPathVector([10.0, 10.0, 10.0]),
        PathAwareGraph(two_grasp_graph()._intervals[:3]),
        on_grasp={
            "left/gripper > box1/handle | f_1": lambda: calls.append("left") or True,
            "right/gripper > box2/handle | 1_12": (
                lambda: calls.append("right") or True
            ),
        },
        on_release=lambda: True,
        sample_params=[0.0, 30.0],
    )

    segments[1].pre_actions[0]()
    segments[2].pre_actions[0]()
    assert calls == ["left", "right"]


def test_segments_accept_action_mapping_by_grasp_label():
    """Action maps can be keyed by parsed grasp label, not only edge name."""
    from hpp_exec.gripper import segments_from_graph

    calls = []
    _, _, segments = segments_from_graph(
        MockPathVector([10.0, 10.0]),
        PathAwareGraph(two_grasp_graph()._intervals[:2]),
        on_grasp={
            "left/gripper grasps box1/handle": lambda: calls.append("left") or True,
        },
        on_release=lambda: True,
        sample_params=[0.0, 20.0],
    )

    segments[1].pre_actions[0]()
    assert calls == ["left"]


def test_segments_accept_action_mapping_by_state_pair():
    """State-pair keys cover custom transition names with state-name fallback."""
    from hpp_exec.gripper import segments_from_graph

    calls = []
    _, _, segments = segments_from_graph(
        MockPathVector([5.0, 5.0]),
        PathAwareGraph(
            [
                (0.0, 5.0, MockTransition("free loop | f", "free", "free")),
                (
                    5.0,
                    10.0,
                    MockTransition(
                        "custom transition",
                        "free",
                        "gripper grasps box/handle",
                    ),
                ),
            ]
        ),
        on_grasp={
            "free -> gripper grasps box/handle": (
                lambda: calls.append("state-pair") or True
            ),
        },
        on_release=lambda: True,
        sample_params=[0.0, 10.0],
    )

    segments[1].pre_actions[0]()
    assert calls == ["state-pair"]


def test_segments_action_mapping_missing_key_raises():
    """A mapping should fail loudly when no key matches the graph transition."""
    from hpp_exec.gripper import segments_from_graph

    try:
        segments_from_graph(
            MockPathVector([10.0, 10.0]),
            PathAwareGraph(two_grasp_graph()._intervals[:2]),
            on_grasp={"right/gripper > box2/handle | f_1": lambda: True},
            on_release=lambda: True,
            sample_params=[0.0, 20.0],
        )
    except KeyError as exc:
        assert "No action found" in str(exc)
    else:
        raise AssertionError(
            "segments_from_graph should fail for an unknown action key"
        )


# ---------------------------------------------------------------------------
# Tests for Segment dataclass
# ---------------------------------------------------------------------------


def test_segment_defaults():
    from hpp_exec import Segment

    seg = Segment(0, 100)
    assert seg.start_index == 0
    assert seg.end_index == 100
    assert seg.pre_actions == []
    assert seg.post_actions == []


def test_segment_with_actions():
    from hpp_exec import Segment

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
# Tests for parsing helpers
# ---------------------------------------------------------------------------


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
