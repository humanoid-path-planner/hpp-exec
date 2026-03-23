#!/usr/bin/env python3
"""
Test gripper coordination with FR3 in Gazebo.

Simulates a pick-and-place manipulation trajectory:
  1. Move arm to pre-grasp pose (gripper open)     ~3s
  2. STOP - Close gripper                           ~1s
  3. Move arm to place pose (carrying object)       ~4s
  4. STOP - Open gripper                            ~1s
  5. Retreat to home                                ~3s

Uses a MockConstraintGraph to simulate HPP constraint graph state queries,
so this test works without HPP installed.

Prerequisites:
    ./scripts/launch_gazebo_gripper.sh   (in terminal 1)

Usage (in terminal 2):
    PYTHONPATH=$HOME/devel/hpp-exec:$PYTHONPATH python3 ~/devel/hpp-exec/scripts/test_gripper_gazebo.py
"""

import sys
import numpy as np

from hpp_exec import execute_segments
from hpp_exec.gripper import segments_from_graph, extract_grasp_transitions

# Import from examples/ — add to path if needed
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "examples"))
from gripper_controllers import JointTrajectoryGripperController


# ---------------------------------------------------------------------------
# Mock constraint graph (no HPP needed)
# ---------------------------------------------------------------------------

class MockConstraintGraph:
    """Simulates HPP constraint graph for a pick-and-place scenario.

    State transitions:
        configs 0-19:   "free"                         (approach)
        configs 20-39:  "gripper grasps object/handle"  (carrying)
        configs 40-59:  "free"                          (released)
    """

    def __init__(self, grasp_at=20, release_at=40):
        self.grasp_at = grasp_at
        self.release_at = release_at
        self._call_count = 0

    def getStateFromConfiguration(self, config):
        idx = self._call_count
        self._call_count += 1
        if idx < self.grasp_at:
            return "free"
        elif idx < self.release_at:
            return "gripper grasps object/handle"
        else:
            return "free"

    def reset(self):
        self._call_count = 0


# ---------------------------------------------------------------------------
# Generate a pick-and-place trajectory for FR3
# ---------------------------------------------------------------------------

def generate_fr3_trajectory():
    """Generate a clear pick-and-place trajectory for FR3 (7 DOF arm)."""

    home =      np.array([0.0, -0.785, 0.0, -2.356, 0.0,  1.571, 0.785])
    pregrasp =  np.array([0.4, -0.3,   0.3, -1.5,   0.2,  1.2,   0.5])
    place =     np.array([-0.4, -0.3,  -0.3, -1.5,  -0.2,  1.2,  -0.5])
    retreat =   np.array([0.0, -0.785, 0.0, -2.356, 0.0,  1.571, 0.785])

    configs = []
    times = []
    n_per_phase = 20

    # Phase 1: home → pregrasp (configs 0-19, "free")
    for i in range(n_per_phase):
        t = i / (n_per_phase - 1)
        arm = home + t * (pregrasp - home)
        q = np.concatenate([arm, np.zeros(9)])
        configs.append(q)
        times.append(i * 0.15)

    # Phase 2: pregrasp → place (configs 20-39, "grasped")
    for i in range(n_per_phase):
        t = i / (n_per_phase - 1)
        arm = pregrasp + t * (place - pregrasp)
        q = np.concatenate([arm, np.zeros(9)])
        configs.append(q)
        times.append(3.0 + i * 0.2)

    # Phase 3: place → retreat (configs 40-59, "free")
    for i in range(n_per_phase):
        t = i / (n_per_phase - 1)
        arm = place + t * (retreat - place)
        q = np.concatenate([arm, np.zeros(9)])
        configs.append(q)
        times.append(7.0 + i * 0.15)

    return configs, times


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    configs, times = generate_fr3_trajectory()
    graph = MockConstraintGraph(grasp_at=20, release_at=40)

    # Preview transitions
    transitions = extract_grasp_transitions(configs, times, graph)
    print(f"\nFound {len(transitions)} grasp transition(s):")
    for t in transitions:
        action = "CLOSE" if t.acquired else "OPEN"
        print(f"  t={t.time:.2f}s (config {t.config_index}): {action}")

    graph.reset()

    # Gripper controller (matches controllers_gazebo.yaml)
    gripper = JointTrajectoryGripperController(
        topic="/gripper_controller/follow_joint_trajectory",
        joint_names=["fr3_finger_joint1"],
        open_positions=[0.04],
        close_positions=[0.0],
        duration=1.0,
    )

    FR3_ARM_JOINTS = [
        "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
        "fr3_joint5", "fr3_joint6", "fr3_joint7",
    ]

    # Build segments from mock graph
    segments = segments_from_graph(
        configs, times, graph,
        on_grasp=gripper.close,
        on_release=gripper.open,
    )

    graph.reset()

    print(f"\nExecuting {len(segments)} segments in Gazebo...")
    print("  Phase 1: Move to pre-grasp pose")
    print("  Phase 2: CLOSE gripper, then move to place pose")
    print("  Phase 3: OPEN gripper, then retreat to home")
    print()

    success = execute_segments(
        segments, configs, times,
        joint_names=FR3_ARM_JOINTS,
        joint_indices=list(range(7)),
        max_velocity=0.3,
    )

    print(f"\nResult: {'SUCCESS' if success else 'FAILED'}")
    gripper.destroy()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
