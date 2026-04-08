"""
Gripper coordination for HPP manipulation trajectories.

Detects grasp/release events from HPP constraint graph state transitions
and builds execution segments with gripper actions as pre/post hooks.

Example:
    from hpp_exec import execute_segments
    from hpp_exec.gripper import segments_from_graph

    # Define your own gripper (any object with open() -> bool, close() -> bool)
    gripper = MyGripperController(...)

    segments = segments_from_graph(
        configs, times, graph,
        on_grasp=gripper.close,
        on_release=gripper.open,
    )

    execute_segments(segments, configs, times, joint_names=[...])
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable, List

import numpy as np

if TYPE_CHECKING:
    from hpp_exec.ros2_sender import Segment

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------


@dataclass
class GraspTransition:
    """A point in the trajectory where the gripper state changes."""

    config_index: int
    """Index in the configs list where this transition occurs."""

    time: float
    """Time at this config."""

    grasps_before: set[str]
    """Active grasp names before the transition."""

    grasps_after: set[str]
    """Active grasp names after the transition."""

    acquired: set[str] = field(default_factory=set)
    """New grasps (close gripper)."""

    released: set[str] = field(default_factory=set)
    """Lost grasps (open gripper)."""

    def __post_init__(self):
        self.acquired = self.grasps_after - self.grasps_before
        self.released = self.grasps_before - self.grasps_after


# ---------------------------------------------------------------------------
# Constraint graph state parsing
# ---------------------------------------------------------------------------


def _parse_grasps_from_state_name(state_name: str) -> set[str]:
    """Extract active grasp descriptions from an HPP constraint graph state name.

    HPP state names encode active grasps, e.g.:
        "free"                                    -> set()
        "r_gripper grasps box/handle"             -> {"r_gripper grasps box/handle"}
        "r_gripper grasps box/handle : l_gripper grasps cup/handle"
            -> {"r_gripper grasps box/handle", "l_gripper grasps cup/handle"}

    The separator between multiple grasps is " : " (space-colon-space).
    """
    if not state_name or state_name.strip().lower() == "free":
        return set()

    # HPP uses " : " to separate multiple active grasps in state names
    parts = [p.strip() for p in state_name.split(" : ")]
    return {p for p in parts if "grasps" in p.lower() or "grasp" in p.lower()}


def extract_grasp_transitions(
    configs: List[np.ndarray],
    times: List[float],
    graph,
) -> List[GraspTransition]:
    """Detect grasp state changes by querying the HPP constraint graph.

    For each config, queries the constraint graph to determine which grasps
    are active (via graph.getStateFromConfiguration). When the set of active
    grasps changes between consecutive configs, a GraspTransition is recorded.

    Args:
        configs: HPP configuration vectors along the path.
        times: Corresponding timestamps.
        graph: HPP manipulation constraint graph (pyhpp.manipulation.Graph).
            Must have a getStateFromConfiguration(q) method that returns the
            constraint graph state name for a given configuration.

    Returns:
        Ordered list of GraspTransition at each state change.
    """
    if len(configs) < 2:
        return []

    transitions = []
    prev_state = graph.getStateFromConfiguration(configs[0])
    prev_grasps = _parse_grasps_from_state_name(prev_state)

    for i in range(1, len(configs)):
        state = graph.getStateFromConfiguration(configs[i])
        grasps = _parse_grasps_from_state_name(state)

        if grasps != prev_grasps:
            transitions.append(
                GraspTransition(
                    config_index=i,
                    time=times[i],
                    grasps_before=prev_grasps,
                    grasps_after=grasps,
                )
            )
            prev_grasps = grasps

    return transitions


# ---------------------------------------------------------------------------
# Segment builder from constraint graph
# ---------------------------------------------------------------------------


def segments_from_graph(
    configs: List[np.ndarray],
    times: List[float],
    graph,
    on_grasp: Callable[[], bool],
    on_release: Callable[[], bool],
) -> List["Segment"]:
    """Build execution segments from HPP constraint graph transitions.

    Detects grasp/release events along the path and creates Segment objects
    with on_grasp/on_release assigned as pre-actions on the appropriate segments.

    Args:
        configs: Full HPP configuration vectors along the path.
        times: Corresponding timestamps.
        graph: HPP manipulation constraint graph (pyhpp.manipulation.Graph).
        on_grasp: Action to run when a grasp is acquired (e.g. gripper.close).
        on_release: Action to run when a grasp is released (e.g. gripper.open).

    Returns:
        List of Segment objects ready for execute_segments().
    """
    from hpp_exec.ros2_sender import Segment

    transitions = extract_grasp_transitions(configs, times, graph)

    if not transitions:
        return [Segment(0, len(configs))]

    # Build segment boundaries: [0, t1, t2, ..., end]
    split_indices = [0] + [t.config_index for t in transitions] + [len(configs)]

    segments = []
    for i in range(len(split_indices) - 1):
        start = split_indices[i]
        end = split_indices[i + 1]

        pre_actions = []
        if i > 0:
            transition = transitions[i - 1]
            if transition.acquired:
                pre_actions.append(on_grasp)
            if transition.released:
                pre_actions.append(on_release)

        segments.append(Segment(start, end, pre_actions=pre_actions))

    return segments
