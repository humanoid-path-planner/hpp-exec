#!/usr/bin/env python3
"""
Example: HPP manipulation trajectory with gripper coordination.

Demonstrates how to use execute_manipulation() to run an HPP manipulation
path that involves picking up an object. The constraint graph determines
when the gripper should open/close.

Usage with mock controller:
    # Terminal 1: Start mock controller with gripper
    python3 examples/mock_controller.py --urdf robots/ur5/ur5.urdf --gripper

    # Terminal 2: Run this example
    python3 examples/manipulation_example.py

Usage with real HPP planning (inside Docker):
    Uncomment the HPP section below and adapt to your robot/problem.
"""

import numpy as np

from hpp_planner import execute_manipulation
from hpp_planner.gripper import (
    GripperCommandController,
    extract_grasp_transitions,
)


# ---------------------------------------------------------------------------
# Mock constraint graph for demonstration (no HPP needed)
# ---------------------------------------------------------------------------

class MockConstraintGraph:
    """Simulates an HPP constraint graph for testing.

    Returns state names based on waypoint index to simulate a pick scenario:
      waypoints 0-29:  "free"                       (approach, gripper open)
      waypoints 30-69: "r_gripper grasps box/handle" (grasped, gripper closed)
      waypoints 70-99: "free"                        (released, gripper open)
    """

    def __init__(self, grasp_index: int = 30, release_index: int = 70):
        self.grasp_index = grasp_index
        self.release_index = release_index
        self._call_count = 0

    def getStateFromConfiguration(self, config):
        idx = self._call_count
        self._call_count += 1

        if idx < self.grasp_index:
            return "free"
        elif idx < self.release_index:
            return "r_gripper grasps box/handle"
        else:
            return "free"


def generate_mock_waypoints(n_points: int = 100):
    """Generate a synthetic pick-and-place trajectory for a 6-DOF arm."""
    waypoints = []
    times = []

    for i in range(n_points):
        t = i / (n_points - 1)
        # Simple sinusoidal motion for arm joints
        q = np.array([
            0.5 * np.sin(2 * np.pi * t),       # shoulder_pan
            -0.5 + 0.3 * np.sin(np.pi * t),    # shoulder_lift
            0.5 * np.cos(np.pi * t),            # elbow
            0.2 * np.sin(2 * np.pi * t),        # wrist_1
            0.1 * np.cos(2 * np.pi * t),        # wrist_2
            0.3 * t,                             # wrist_3
            # Extra DOFs that would exist in a real HPP config:
            0.04 if i < 30 or i >= 70 else 0.0, # gripper finger (not used for control)
            0.5, 0.3, 0.7, 0.0, 0.0, 0.0, 1.0, # object SE3 pose (freeflyer)
        ])
        waypoints.append(q)
        times.append(t * 10.0)  # 10 second trajectory

    return waypoints, times


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    # -----------------------------------------------------------------------
    # Option A: Mock data for testing without HPP
    # -----------------------------------------------------------------------
    waypoints, times = generate_mock_waypoints(100)
    graph = MockConstraintGraph(grasp_index=30, release_index=70)

    # -----------------------------------------------------------------------
    # Option B: Real HPP planning (uncomment and adapt)
    # -----------------------------------------------------------------------
    # from pyhpp.manipulation import Device, Graph, Problem, ManipulationPlanner
    # from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory, Rule
    #
    # robot = Device("ur5")
    # # ... load URDF, setup problem, create constraint graph ...
    # # ... solve ...
    # path = problem.path(0)
    # n_points = 100
    # waypoints = [np.array(path(t * path.length() / n_points)[0])
    #              for t in range(n_points)]
    # times = [t * path.length() / n_points for t in range(n_points)]
    # graph = constraint_graph  # your pyhpp.manipulation.Graph

    # Preview grasp transitions before executing
    transitions = extract_grasp_transitions(waypoints, times, graph)
    print(f"\nFound {len(transitions)} grasp transition(s):")
    for t in transitions:
        action = "CLOSE" if t.acquired else "OPEN"
        print(f"  t={t.time:.2f}s (waypoint {t.waypoint_index}): {action}")
        if t.acquired:
            print(f"    Acquired: {t.acquired}")
        if t.released:
            print(f"    Released: {t.released}")

    # Reset mock graph call counter for actual execution
    if isinstance(graph, MockConstraintGraph):
        graph._call_count = 0

    # Setup gripper controller
    gripper = GripperCommandController(
        topic="/gripper_controller/gripper_command",
        open_position=0.04,
        close_position=0.0,
        max_effort=10.0,
    )

    # Execute with gripper coordination
    print("\nExecuting manipulation trajectory...")
    success = execute_manipulation(
        waypoints, times,
        arm_joint_names=[
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
        ],
        arm_joint_indices=[0, 1, 2, 3, 4, 5],  # first 6 DOFs are arm joints
        gripper_controller=gripper,
        graph=graph,
        max_velocity=0.5,
    )

    print(f"\nResult: {'SUCCESS' if success else 'FAILED'}")


if __name__ == "__main__":
    main()
