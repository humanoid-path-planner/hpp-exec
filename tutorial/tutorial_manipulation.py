#!/usr/bin/env python3
"""
FR3 Pick-and-Place with HPP Manipulation Planning
==================================================

Full pipeline: HPP plans a pick-and-place trajectory using the constraint
graph, then hpp_planner sends arm segments and gripper commands to Gazebo.

The FR3 picks up a cube from position A and places it at position B.

Prerequisites:
    # Build HPP (first time only)
    cd ~/devel/src && make all

    # Terminal 1: Launch Gazebo
    ./hpp-planning/scripts/launch_gazebo_gripper.sh

    # Terminal 2: Run this tutorial
    docker exec -it hpp-planning bash
    python3 ~/devel/hpp-planning/tutorial/tutorial_manipulation.py
"""

import os
import sys
import time
from math import pi

import numpy as np
from pinocchio import SE3, Quaternion

from pyhpp.manipulation import (
    Device,
    Graph,
    Problem,
    ProgressiveProjector,
    urdf,
    ManipulationPlanner,
)
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.core import Dichotomy, Straight

from pyhpp.constraints import (
    Transformation,
    ComparisonTypes,
    ComparisonType,
    Implicit,
    LockedJoint,
)

from hpp_planner import execute_manipulation
from hpp_planner.gripper import (
    JointTrajectoryGripperController,
    extract_grasp_transitions,
)


# ---------------------------------------------------------------------------
# File paths
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)

FR3_URDF = os.path.join(PROJECT_DIR, "robots", "fr3", "fr3.urdf")
FR3_SRDF = os.path.join(PROJECT_DIR, "robots", "fr3", "fr3_manipulation.srdf")
CUBE_URDF = os.path.join(PROJECT_DIR, "robots", "cube", "cube.urdf")
CUBE_SRDF = os.path.join(PROJECT_DIR, "robots", "cube", "cube.srdf")

FR3_ARM_JOINTS = [
    "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
    "fr3_joint5", "fr3_joint6", "fr3_joint7",
]


def setup_problem():
    """Set up the HPP manipulation planning problem."""

    print("Loading FR3 robot and cube...")
    robot = Device("fr3-cube")

    # Load FR3 arm (fixed base)
    urdf.loadModel(robot, 0, "fr3", "anchor", FR3_URDF, FR3_SRDF, SE3.Identity())

    # Load cube as a freeflyer object (can be moved in space)
    cube_pose = SE3(Quaternion(1, 0, 0, 0), np.array([0.5, 0.0, 0.02]))
    urdf.loadModel(robot, 0, "cube", "freeflyer", CUBE_URDF, CUBE_SRDF, cube_pose)

    # Set bounds for the cube freeflyer joint (x, y, z, qx, qy, qz, qw)
    robot.setJointBounds("cube/root_joint", [
        -1.0, 1.0,     # x
        -1.0, 1.0,     # y
        -0.01, 1.0,    # z (above ground)
        -1.001, 1.001,  # qx
        -1.001, 1.001,  # qy
        -1.001, 1.001,  # qz
        -1.001, 1.001,  # qw
    ])

    print(f"  Device: {robot.name()}, config size: {robot.configSize()}")
    print(f"  Grippers: {list(robot.grippers().keys())}")
    print(f"  Handles: {list(robot.handles().keys())}")

    # --- Problem and constraint graph ---
    problem = Problem(robot)
    graph = Graph("graph", robot, problem)
    graph.maxIterations(40)
    graph.errorThreshold(0.0001)

    # --- Placement constraints ---
    # The cube must stay on the ground (z = 0.02, half the cube height)
    constraints = {}

    cube_on_ground = SE3(Quaternion(1, 0, 0, 0), np.array([0, 0, 0.02]))
    joint_id = robot.model().getJointId("cube/root_joint")

    # Placement: constrain z, roll, pitch (cube stays flat on ground)
    pc = Transformation(
        "place_cube", robot, joint_id,
        SE3.Identity(), cube_on_ground,
        [False, False, True, True, True, False],  # z, rx, ry constrained
    )
    cts = ComparisonTypes()
    cts[:] = (
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
    )
    constraints["place_cube"] = Implicit(pc, cts, [True, True, True])

    # Placement complement: x, y, yaw (cube can slide on ground)
    pc_comp = Transformation(
        "place_cube/complement", robot, joint_id,
        SE3.Identity(), cube_on_ground,
        [True, True, False, False, False, True],  # x, y, rz
    )
    cts_comp = ComparisonTypes()
    cts_comp[:] = (
        ComparisonType.Equality,
        ComparisonType.Equality,
        ComparisonType.Equality,
    )
    constraints["place_cube/complement"] = Implicit(pc_comp, cts_comp, [True, True, True])

    # Hold: lock cube in place when not grasped
    cts_hold = ComparisonTypes()
    cts_hold[:] = (
        ComparisonType.Equality,
        ComparisonType.Equality,
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
        ComparisonType.Equality,
    )
    constraints["place_cube/hold"] = LockedJoint(
        robot, "cube/root_joint",
        np.array([0, 0, 0.02, 0, 0, 0, 1]),  # x, y, z, qx, qy, qz, qw
        cts_hold,
    )

    graph.registerConstraints(
        constraints["place_cube"],
        constraints["place_cube/complement"],
        constraints["place_cube/hold"],
    )

    # --- Build constraint graph using factory ---
    factory = ConstraintGraphFactory(graph, constraints)
    factory.setGrippers(["fr3/gripper"])
    factory.setObjects(["cube"], [["cube/handle"]], [[]])
    factory.generate()

    # Add placement complement to grasp/release transitions
    for transition_name in graph.getTransitionNames():
        if "fr3/gripper > cube/handle" in transition_name:
            e = graph.getTransition(transition_name)
            graph.addNumericalConstraintsToTransition(
                e, [constraints["place_cube/complement"]]
            )
        if "fr3/gripper < cube/handle" in transition_name:
            e = graph.getTransition(transition_name)
            graph.addNumericalConstraintsToTransition(
                e, [constraints["place_cube/complement"]]
            )

    # --- Steering and path validation ---
    problem.steeringMethod = Straight(problem)
    problem.pathValidation = Dichotomy(robot, 0.0)
    problem.pathProjector = ProgressiveProjector(
        problem.distance(), problem.steeringMethod, 0.01,
    )

    graph.initialize()
    problem.constraintGraph(graph)

    return robot, problem, graph


def plan_pick_and_place(problem, graph):
    """Plan a pick-and-place trajectory."""

    # FR3 config (7 DOF) + finger (1 DOF) + cube freeflyer (7 DOF) = 15 DOF
    # Note: fr3_finger_joint2 is a mimic joint, not in the config

    # Initial config: arm at ready pose, cube at position A
    q_init = np.array([
        # FR3 arm (7 joints)
        0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785,
        # FR3 finger
        0.035,
        # Cube pose (x, y, z, qx, qy, qz, qw)
        0.5, 0.0, 0.02, 0.0, 0.0, 0.0, 1.0,
    ])

    # Goal config: arm at ready pose, cube at position B
    q_goal = np.array([
        # FR3 arm (same ready pose)
        0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785,
        # FR3 finger
        0.035,
        # Cube at new position
        0.3, 0.3, 0.02, 0.0, 0.0, 0.0, 1.0,
    ])

    problem.initConfig(q_init)
    problem.resetGoalConfigs()
    problem.addGoalConfig(q_goal)

    planner = ManipulationPlanner(problem)
    planner.maxIterations(5000)

    print("\nPlanning pick-and-place trajectory...")
    start = time.time()
    path = planner.solve()

    if path is None:
        print("  Planning FAILED")
        return None, None

    elapsed = time.time() - start
    print(f"  Solved in {elapsed:.1f}s, path length: {path.length():.3f}")

    # Sample waypoints from the path
    n_samples = max(int(path.length() / 0.02), 50)
    waypoints = []
    times = []

    for i in range(n_samples + 1):
        t = (i / n_samples) * path.length()
        q, success = path(t)
        if success:
            waypoints.append(np.array(q))
            times.append(t)

    print(f"  Extracted {len(waypoints)} waypoints")

    return waypoints, times


def execute_on_gazebo(waypoints, times, graph):
    """Send the planned trajectory to Gazebo with gripper coordination."""

    # Preview grasp transitions
    transitions = extract_grasp_transitions(waypoints, times, graph)
    print(f"\nGrasp transitions found: {len(transitions)}")
    for t in transitions:
        action = "CLOSE" if t.acquired else "OPEN"
        print(f"  t={t.time:.2f}s (waypoint {t.waypoint_index}): {action}")

    # Setup gripper controller (FR3 finger via JointTrajectoryController)
    gripper = JointTrajectoryGripperController(
        topic="/gripper_controller/follow_joint_trajectory",
        joint_names=["fr3_finger_joint1"],
        open_positions=[0.04],
        close_positions=[0.0],
        duration=1.0,
    )

    # FR3 arm joint indices in the HPP config vector
    # Config layout: [arm(0-6), finger(7), cube_freeflyer(8-14)]
    arm_indices = list(range(7))

    print("\nExecuting on Gazebo...")
    success = execute_manipulation(
        waypoints, times,
        arm_joint_names=FR3_ARM_JOINTS,
        arm_joint_indices=arm_indices,
        gripper_controller=gripper,
        graph=graph,
        max_velocity=0.3,  # slow for visibility
    )

    gripper.destroy()
    return success


def main():
    print("=" * 60)
    print("FR3 Pick-and-Place: HPP Planning + Gazebo Execution")
    print("=" * 60)

    for f in [FR3_URDF, FR3_SRDF, CUBE_URDF, CUBE_SRDF]:
        if not os.path.exists(f):
            print(f"Missing file: {f}")
            return 1

    robot, problem, graph = setup_problem()
    waypoints, times = plan_pick_and_place(problem, graph)

    if waypoints is None:
        return 1

    success = execute_on_gazebo(waypoints, times, graph)

    print(f"\nResult: {'SUCCESS' if success else 'FAILED'}")
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
