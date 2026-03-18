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
    PYTHONPATH=$HOME/devel/hpp-planning:$PYTHONPATH python3 ~/devel/hpp-planning/scripts/test_gripper_gazebo.py
"""

import sys
import numpy as np

from hpp_planner.gripper import (
    execute_manipulation,
    extract_grasp_transitions,
    JointTrajectoryGripperController,
)


# ---------------------------------------------------------------------------
# Mock constraint graph (no HPP needed)
# ---------------------------------------------------------------------------

class MockConstraintGraph:
    """Simulates HPP constraint graph for a pick-and-place scenario.

    State transitions:
        waypoints 0-19:   "free"                         (approach)
        waypoints 20-39:  "gripper grasps object/handle"  (carrying)
        waypoints 40-59:  "free"                          (released)
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
    """Generate a clear pick-and-place trajectory for FR3 (7 DOF arm).

    Uses fewer waypoints with longer time spacing so movements are
    clearly visible in Gazebo.
    """

    # FR3 key poses (7 arm joints)
    home =      np.array([0.0, -0.785, 0.0, -2.356, 0.0,  1.571, 0.785])
    pregrasp =  np.array([0.4, -0.3,   0.3, -1.5,   0.2,  1.2,   0.5])
    place =     np.array([-0.4, -0.3,  -0.3, -1.5,  -0.2,  1.2,  -0.5])
    retreat =   np.array([0.0, -0.785, 0.0, -2.356, 0.0,  1.571, 0.785])

    waypoints = []
    times = []
    n_per_phase = 20

    # Phase 1: home → pregrasp (waypoints 0-19, "free", gripper open)
    for i in range(n_per_phase):
        t = i / (n_per_phase - 1)
        arm = home + t * (pregrasp - home)
        q = np.concatenate([arm, np.zeros(9)])  # pad for HPP-style config
        waypoints.append(q)
        times.append(i * 0.15)  # ~3s total

    # Phase 2: pregrasp → place (waypoints 20-39, "grasped", gripper closed)
    for i in range(n_per_phase):
        t = i / (n_per_phase - 1)
        arm = pregrasp + t * (place - pregrasp)
        q = np.concatenate([arm, np.zeros(9)])
        waypoints.append(q)
        times.append(3.0 + i * 0.2)  # ~4s total

    # Phase 3: place → retreat (waypoints 40-59, "free", gripper open)
    for i in range(n_per_phase):
        t = i / (n_per_phase - 1)
        arm = place + t * (retreat - place)
        q = np.concatenate([arm, np.zeros(9)])
        waypoints.append(q)
        times.append(7.0 + i * 0.15)  # ~3s total

    return waypoints, times


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    waypoints, times = generate_fr3_trajectory()
    graph = MockConstraintGraph(grasp_at=20, release_at=40)

    # Preview transitions
    transitions = extract_grasp_transitions(waypoints, times, graph)
    print(f"\nFound {len(transitions)} grasp transition(s):")
    for t in transitions:
        action = "CLOSE" if t.acquired else "OPEN"
        print(f"  t={t.time:.2f}s (waypoint {t.waypoint_index}): {action}")

    graph.reset()

    # Use JointTrajectoryController for gripper (matches controllers_gripper.yaml)
    gripper = JointTrajectoryGripperController(
        topic="/gripper_controller/follow_joint_trajectory",
        joint_names=["fr3_finger_joint1"],
        open_positions=[0.04],
        close_positions=[0.0],
        duration=1.0,  # 1 second for gripper open/close
    )

    FR3_ARM_JOINTS = [
        "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
        "fr3_joint5", "fr3_joint6", "fr3_joint7",
    ]

    print("\nExecuting pick-and-place in Gazebo...")
    print("  Phase 1: Move to pre-grasp pose")
    print("  Phase 2: CLOSE gripper, then move to place pose")
    print("  Phase 3: OPEN gripper, then retreat to home")
    print()

    success = execute_manipulation(
        waypoints, times,
        arm_joint_names=FR3_ARM_JOINTS,
        arm_joint_indices=list(range(7)),
        gripper_controller=gripper,
        graph=graph,
        max_velocity=0.3,  # slow for visibility
    )

    print(f"\nResult: {'SUCCESS' if success else 'FAILED'}")
    gripper.destroy()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
