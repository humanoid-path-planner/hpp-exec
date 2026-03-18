#!/usr/bin/env python3
"""
FR3 Pick-and-Place — HPP Manipulation Planning
===============================================

Sets up and solves an HPP manipulation problem: FR3 picks a cube
from position A and places it at position B.

Run standalone to plan and inspect the result:
    python3 ~/devel/hpp-exec/tutorial/tutorial_manipulation.py

Or import from another script to access the path, robot, and graph:
    from tutorial_manipulation import plan_pick_and_place
    robot, problem, cg, path = plan_pick_and_place()
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


def plan_pick_and_place():
    """Plan a pick-and-place trajectory.

    Returns (robot, problem, cg, path) where path is the solved HPP path,
    or None if planning failed.
    """

    print("Loading FR3 + cube...")
    robot = Device("fr3-cube")
    urdf.loadModel(robot, 0, "fr3", "anchor", FR3_URDF, FR3_SRDF, SE3.Identity())

    # Load cube at origin (like ur3-spheres)
    cube_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
    urdf.loadModel(robot, 0, "cube", "freeflyer", CUBE_URDF, CUBE_SRDF, cube_pose)
    robot.setJointBounds("cube/root_joint", [
        -1.0, 1.0, -1.0, 1.0, -0.1, 1.0,
        -1.0001, 1.0001, -1.0001, 1.0001, -1.0001, 1.0001, -1.0001, 1.0001,
    ])
    print(f"  Config size: {robot.configSize()}")

    # --- Constraint graph ---
    problem = Problem(robot)
    cg = Graph("graph", robot, problem)
    constraints = dict()

    # Handle mask (as ur3-spheres does)
    h = robot.handles()["cube/handle"]
    h.mask = [True, True, True, False, True, True]

    # Placement constraints: cube on table at z=0.02 (half of 4cm cube)
    Id = SE3.Identity()
    cubePlacement = SE3(Quaternion(1, 0, 0, 0), np.array([0, 0, 0.02]))
    cube_joint = robot.model().getJointId("cube/root_joint")

    pc = Transformation(
        "place_cube", robot, cube_joint, Id, cubePlacement,
        [False, False, True, True, True, False],
    )
    cts = ComparisonTypes()
    cts[:] = (
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
    )
    constraints["place_cube"] = Implicit(pc, cts, [True, True, True])

    # Complement: x, y, rotz free
    pc = Transformation(
        "place_cube/complement", robot, cube_joint, Id, cubePlacement,
        [True, True, False, False, False, True],
    )
    cts[:] = (
        ComparisonType.Equality,
        ComparisonType.Equality,
        ComparisonType.Equality,
    )
    constraints["place_cube/complement"] = Implicit(pc, cts, [True, True, True])

    # Hold: LockedJoint
    cts[:] = (
        ComparisonType.Equality,
        ComparisonType.Equality,
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
        ComparisonType.Equality,
    )
    ll = LockedJoint(
        robot, "cube/root_joint",
        np.array([0, 0, 0.02, 0, 0, 0, 1]),
        cts,
    )
    constraints["place_cube/hold"] = ll
    cg.registerConstraints(
        constraints["place_cube"],
        constraints["place_cube/complement"],
        constraints["place_cube/hold"],
    )

    # Pre-placement (above table at z=0.1)
    pc = Transformation(
        "preplace_cube", robot, cube_joint, Id,
        SE3(Quaternion(1, 0, 0, 0), np.array([0, 0, 0.1])),
        [False, False, True, True, True, False],
    )
    cts[:] = (
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
    )
    constraints["preplace_cube"] = Implicit(pc, cts, [True, True, True])

    # --- Factory ---
    cg.maxIterations(40)
    cg.errorThreshold(0.0001)
    factory = ConstraintGraphFactory(cg, constraints)
    factory.setGrippers(["fr3/gripper"])
    factory.setObjects(["cube"], [["cube/handle"]], [[]])
    factory.generate()

    # Complement on preplace↔grasps transitions (same as ur3-spheres)
    try:
        e = cg.getTransition("fr3/gripper > cube/handle | f_23")
        cg.addNumericalConstraintsToTransition(
            e, [constraints["place_cube/complement"]])
    except RuntimeError:
        pass
    try:
        e = cg.getTransition("fr3/gripper < cube/handle | 0-0_32")
        cg.addNumericalConstraintsToTransition(
            e, [constraints["place_cube/complement"]])
    except RuntimeError:
        pass

    problem.steeringMethod = Straight(problem)
    problem.pathValidation = Dichotomy(robot, 0)
    problem.pathProjector = ProgressiveProjector(
        problem.distance(), problem.steeringMethod, 0.01)

    cg.initialize()
    print(f"  States: {cg.getStateNames()}")
    print(f"  Transitions: {len(cg.getTransitionNames())}")

    # --- Configs ---
    # FR3 arm (7) + fingers (2) + cube freeflyer (7) = 16
    q_init = np.array([
        0.0, -pi/4, 0.0, -3*pi/4, 0.0, pi/2, pi/4,   # arm (ready)
        0.035, 0.035,                                    # fingers
        0.5, 0.0, 0.02, 0.0, 0.0, 0.0, 1.0,            # cube at A
    ])
    q_goal = np.array([
        0.0, -pi/4, 0.0, -3*pi/4, 0.0, pi/2, pi/4,   # arm (ready)
        0.035, 0.035,                                    # fingers
        0.3, 0.3, 0.02, 0.0, 0.0, 0.0, 1.0,            # cube at B
    ])

    problem.initConfig(q_init)
    problem.addGoalConfig(q_goal)
    problem.constraintGraph(cg)

    # --- Solve ---
    planner = ManipulationPlanner(problem)
    planner.maxIterations(5000)

    print("\nPlanning...")
    start = time.time()
    path = planner.solve()

    if path is None:
        print("  FAILED")
        return robot, problem, cg, None

    elapsed = time.time() - start
    print(f"  Solved in {elapsed:.1f}s, path length: {path.length():.3f}")
    return robot, problem, cg, path


def extract_waypoints(path, n_per_unit=50):
    """Sample waypoints from an HPP path.

    Returns (full_waypoints, arm_waypoints, times) where:
    - full_waypoints: list of full config vectors (16 DOF)
    - arm_waypoints: list of arm-only configs (7 DOF)
    - times: list of path parameter values
    """
    n_samples = max(int(path.length() * n_per_unit), 50)
    full_waypoints = []
    arm_waypoints = []
    times = []
    for i in range(n_samples + 1):
        t = (i / n_samples) * path.length()
        q, success = path(t)
        if success:
            full_waypoints.append(np.array(q))
            arm_waypoints.append(np.array(q[:7]))
            times.append(t)
    return full_waypoints, arm_waypoints, times


def main():
    for f in [FR3_URDF, FR3_SRDF, CUBE_URDF, CUBE_SRDF]:
        if not os.path.exists(f):
            print(f"Missing: {f}")
            return 1

    robot, problem, cg, path = plan_pick_and_place()

    if path is None:
        return 1

    full_wp, arm_wp, times = extract_waypoints(path)
    print(f"  {len(arm_wp)} waypoints extracted")
    print(f"  Start cube pos: {full_wp[0][9:12]}")
    print(f"  End cube pos:   {full_wp[-1][9:12]}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
