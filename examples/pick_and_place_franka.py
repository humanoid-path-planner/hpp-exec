#!/usr/bin/env python3
"""
FR3 Pick-and-Place -- Real Hardware Execution
==============================================

Plans an HPP manipulation problem (FR3 picks a cube from A and places
it at B) and executes it using the native Franka gripper actions
(Grasp/Move) instead of JointTrajectoryController.

The Franka gripper node exposes:
    /{arm_id}_gripper/move   -- Move fingers to a width (for opening)
    /{arm_id}_gripper/grasp  -- Close with force control (for grasping)

These are only available on real hardware (not Gazebo).
For Gazebo, use pick_and_place_gazebo.py instead.

Prerequisites:
    # Launch franka_ros2 with real robot
    ros2 launch franka_bringup franka.launch.py robot_ip:=<ROBOT_IP> arm_id:=fr3

    # Run this script
    python3 pick_and_place_franka.py
"""

import os
import time
from math import pi

import numpy as np
from gripper_controllers import FrankaGripperController
from pinocchio import SE3, Quaternion
from pyhpp.constraints import (
    ComparisonType,
    ComparisonTypes,
    Implicit,
    LockedJoint,
    Transformation,
)
from pyhpp.core import Dichotomy, Straight
from pyhpp.manipulation import (
    Device,
    Graph,
    ManipulationPlanner,
    Problem,
    ProgressiveProjector,
    urdf,
)
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory

from hpp_exec import execute_segments
from hpp_exec.gripper import extract_path_grasp_transitions, segments_from_graph

# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)

FR3_URDF = os.path.join(PROJECT_DIR, "robots", "fr3", "fr3.urdf")
FR3_SRDF = os.path.join(PROJECT_DIR, "robots", "fr3", "fr3_manipulation.srdf")
CUBE_URDF = os.path.join(PROJECT_DIR, "robots", "cube", "cube.urdf")
CUBE_SRDF = os.path.join(PROJECT_DIR, "robots", "cube", "cube.srdf")

FR3_ARM_JOINTS = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]


def plan_pick_and_place():
    """Plan a pick-and-place trajectory.

    Returns (robot, problem, cg, path) where path is the solved HPP path,
    or None if planning failed.
    """

    print("Loading FR3 + cube...")
    robot = Device("fr3-cube")
    urdf.loadModel(robot, 0, "fr3", "anchor", FR3_URDF, FR3_SRDF, SE3.Identity())

    cube_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
    urdf.loadModel(robot, 0, "cube", "freeflyer", CUBE_URDF, CUBE_SRDF, cube_pose)
    robot.setJointBounds(
        "cube/root_joint",
        [
            -1.0, 1.0,
            -1.0, 1.0,
            -0.1, 1.0,
            -1.0001, 1.0001,
            -1.0001, 1.0001,
            -1.0001, 1.0001,
            -1.0001, 1.0001,
        ],
    )
    print(f"  Config size: {robot.configSize()}")

    # --- Constraint graph ---
    problem = Problem(robot)
    cg = Graph("graph", robot, problem)
    constraints = dict()

    h = robot.handles()["cube/handle"]
    h.mask = [True, True, True, False, True, True]

    # Placement: cube on table at z=0.02 (half of 4cm cube)
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
        np.array([0, 0, 0.02, 0, 0, 0, 1]), cts,
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

    cg.maxIterations(40)
    cg.errorThreshold(0.0001)
    factory = ConstraintGraphFactory(cg, constraints)
    factory.setGrippers(["fr3/gripper"])
    factory.setObjects(["cube"], [["cube/handle"]], [[]])
    factory.generate()

    for transition_name in (
        "fr3/gripper > cube/handle | f_23",
        "fr3/gripper < cube/handle | 0-0_32",
    ):
        try:
            edge = cg.getTransition(transition_name)
        except RuntimeError:
            continue
        cg.addNumericalConstraintsToTransition(
            edge, [constraints["place_cube/complement"]]
        )

    problem.steeringMethod = Straight(problem)
    problem.pathValidation = Dichotomy(robot, 0)
    problem.pathProjector = ProgressiveProjector(
        problem.distance(), problem.steeringMethod, 0.01
    )

    cg.initialize()
    print(f"  States: {cg.getStateNames()}")
    print(f"  Transitions: {len(cg.getTransitionNames())}")

    # FR3 arm (7) + fingers (2) + cube freeflyer (7) = 16
    q_init = np.array([
        0.0, -pi / 4, 0.0, -3 * pi / 4, 0.0, pi / 2, pi / 4,
        0.035, 0.035,
        0.5, 0.0, 0.02, 0.0, 0.0, 0.0, 1.0,  # cube at A
    ])
    q_goal = np.array([
        0.0, -pi / 4, 0.0, -3 * pi / 4, 0.0, pi / 2, pi / 4,
        0.035, 0.035,
        0.3, 0.3, 0.02, 0.0, 0.0, 0.0, 1.0,  # cube at B
    ])

    problem.initConfig(q_init)
    problem.addGoalConfig(q_goal)
    problem.constraintGraph(cg)

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


def main():
    # --- Plan ---
    robot, problem, cg, path = plan_pick_and_place()
    if path is None:
        return 1

    # --- Franka gripper (real hardware) ---
    gripper = FrankaGripperController(
        arm_id="fr3",
        open_width=0.08,  # fully open
        grasp_width=0.02,  # cube is 4cm, leave margin
        grasp_force=50.0,  # Newtons
        grasp_speed=0.05,  # m/s
    )

    # --- Sample path and build segments from graph transitions ---
    full_configs, times, segments = segments_from_graph(
        path,
        cg,
        on_grasp=gripper.close,
        on_release=gripper.open,
    )

    # --- Log transitions ---
    transitions = extract_path_grasp_transitions(path, cg)
    print(f"\nPath grasp transitions: {len(transitions)}")
    for t in transitions:
        action = "GRASP" if t.acquired else "RELEASE"
        print(f"  s={t.time:.2f}: {action} ({t.transition_name})")

    # --- Execute ---
    print(f"\nExecuting {len(segments)} segments on real FR3...")
    success = execute_segments(
        segments,
        full_configs,
        times,
        joint_names=FR3_ARM_JOINTS,
        joint_indices=list(range(7)),
        time_parameterization="trapezoidal",
        max_velocity=0.3,
    )

    gripper.destroy()

    print(f"\nResult: {'SUCCESS' if success else 'FAILED'}")
    return 0 if success else 1


if __name__ == "__main__":
    main()
