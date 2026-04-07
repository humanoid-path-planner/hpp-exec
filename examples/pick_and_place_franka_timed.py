#!/usr/bin/env python3
"""
FR3 Pick-and-Place — Real Hardware with HPP Time Parameterization
=================================================================

Self-contained script: plans a pick-and-place with HPP manipulation,
applies SimpleTimeParameterization for dynamically-feasible timing,
then executes on a real FR3 with Franka gripper actions.

This means the path parameter IS real time (seconds), so we bypass
hpp-exec's simple time scaling entirely.

Prerequisites:
    # Launch franka_ros2 with real robot
    ros2 launch franka_bringup franka.launch.py robot_ip:=<ROBOT_IP> arm_id:=fr3

    # Run this script
    python3 pick_and_place_franka_timed.py
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
from pyhpp.core import Dichotomy, SimpleTimeParameterization, Straight
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
from hpp_exec.gripper import extract_grasp_transitions, segments_from_graph

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


def extract_configs_timed(path, dt=0.02):
    """Sample configs from a time-parameterized HPP path.

    Since path.length() is already in seconds, we sample at a fixed
    timestep (default 50 Hz to match typical ROS2 control rates).

    Returns:
        (full_configs, arm_configs, times) where times are real seconds.
    """
    duration = path.length()
    n_samples = max(int(duration / dt), 2)

    full_configs = []
    arm_configs = []
    times = []

    for i in range(n_samples + 1):
        t = min((i / n_samples) * duration, duration)
        q, success = path(t)
        if success:
            full_configs.append(np.array(q))
            arm_configs.append(np.array(q[:7]))
            times.append(t)

    return full_configs, arm_configs, times


def plan_pick_and_place():
    """Plan a pick-and-place trajectory.

    Returns (robot, problem, cg, path) where path is the solved HPP path,
    or None if planning failed.
    """
    for f in [FR3_URDF, FR3_SRDF, CUBE_URDF, CUBE_SRDF]:
        if not os.path.exists(f):
            print(f"Missing: {f}")
            return None, None, None, None

    print("Loading FR3 + cube...")
    robot = Device("fr3-cube")
    urdf.loadModel(robot, 0, "fr3", "anchor", FR3_URDF, FR3_SRDF, SE3.Identity())

    cube_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
    urdf.loadModel(robot, 0, "cube", "freeflyer", CUBE_URDF, CUBE_SRDF, cube_pose)
    robot.setJointBounds(
        "cube/root_joint",
        [
            -1.0,
            1.0,
            -1.0,
            1.0,
            -0.1,
            1.0,
            -1.0001,
            1.0001,
            -1.0001,
            1.0001,
            -1.0001,
            1.0001,
            -1.0001,
            1.0001,
        ],
    )
    print(f"  Config size: {robot.configSize()}")

    # --- Constraint graph ---
    problem = Problem(robot)
    cg = Graph("graph", robot, problem)
    constraints = dict()

    h = robot.handles()["cube/handle"]
    h.mask = [True, True, True, False, True, True]

    # Placement constraints: cube on table at z=0.02 (half of 4cm cube)
    Id = SE3.Identity()
    cubePlacement = SE3(Quaternion(1, 0, 0, 0), np.array([0, 0, 0.02]))
    cube_joint = robot.model().getJointId("cube/root_joint")

    pc = Transformation(
        "place_cube",
        robot,
        cube_joint,
        Id,
        cubePlacement,
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
        "place_cube/complement",
        robot,
        cube_joint,
        Id,
        cubePlacement,
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
        robot,
        "cube/root_joint",
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
        "preplace_cube",
        robot,
        cube_joint,
        Id,
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

    # Complement on preplace↔grasps transitions
    try:
        e = cg.getTransition("fr3/gripper > cube/handle | f_23")
        cg.addNumericalConstraintsToTransition(
            e, [constraints["place_cube/complement"]]
        )
    except RuntimeError:
        pass
    try:
        e = cg.getTransition("fr3/gripper < cube/handle | 0-0_32")
        cg.addNumericalConstraintsToTransition(
            e, [constraints["place_cube/complement"]]
        )
    except RuntimeError:
        pass

    problem.steeringMethod = Straight(problem)
    problem.pathValidation = Dichotomy(robot, 0)
    problem.pathProjector = ProgressiveProjector(
        problem.distance(), problem.steeringMethod, 0.01
    )

    cg.initialize()
    print(f"  States: {cg.getStateNames()}")
    print(f"  Transitions: {len(cg.getTransitionNames())}")

    # --- Configs ---
    # FR3 arm (7) + fingers (2) + cube freeflyer (7) = 16
    q_init = np.array(
        [
            0.0,
            -pi / 4,
            0.0,
            -3 * pi / 4,
            0.0,
            pi / 2,
            pi / 4,  # arm (ready)
            0.035,
            0.035,  # fingers
            0.5,
            0.0,
            0.02,
            0.0,
            0.0,
            0.0,
            1.0,  # cube at A
        ]
    )
    q_goal = np.array(
        [
            0.0,
            -pi / 4,
            0.0,
            -3 * pi / 4,
            0.0,
            pi / 2,
            pi / 4,  # arm (ready)
            0.035,
            0.035,  # fingers
            0.3,
            0.3,
            0.02,
            0.0,
            0.0,
            0.0,
            1.0,  # cube at B
        ]
    )

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


def main():
    # --- Plan ---
    robot, problem, cg, path = plan_pick_and_place()
    if path is None:
        return 1

    print(f"\nRaw path length (geometric): {path.length():.3f}")

    # --- Time parameterization via HPP ---
    # Order 2 = 5th-order polynomial: zero velocity AND acceleration
    # at segment boundaries → smoothest motion.
    problem.setParameter("SimpleTimeParameterization/order", 2)
    problem.setParameter("SimpleTimeParameterization/maxAcceleration", 1.0)
    # safety < 1.0 scales down velocities (0.5 = use 50% of joint limits)
    problem.setParameter("SimpleTimeParameterization/safety", 0.5)

    optimizer = SimpleTimeParameterization(problem)
    timed_path = optimizer.optimize(path)

    duration = timed_path.length()
    print(f"Time-parameterized duration: {duration:.2f}s")

    # --- Sample configs (times are now real seconds) ---
    full_configs, arm_configs, times = extract_configs_timed(timed_path, dt=0.02)
    print(f"Sampled {len(full_configs)} configs at 50 Hz")

    # --- Log transitions ---
    transitions = extract_grasp_transitions(full_configs, times, cg)
    print(f"\nGrasp transitions: {len(transitions)}")
    for t in transitions:
        action = "GRASP" if t.acquired else "RELEASE"
        print(f"  t={t.time:.2f}s (config {t.config_index}): {action}")

    # --- Franka gripper (real hardware) ---
    gripper = FrankaGripperController(
        arm_id="fr3",
        open_width=0.08,
        grasp_width=0.02,
        grasp_force=50.0,
        grasp_speed=0.05,
    )

    # --- Build segments from constraint graph ---
    segments = segments_from_graph(
        full_configs,
        times,
        cg,
        on_grasp=gripper.close,
        on_release=gripper.open,
    )

    # --- Execute ---
    print(f"\nExecuting {len(segments)} segments on real FR3...")
    success = execute_segments(
        segments,
        full_configs,
        times,
        joint_names=FR3_ARM_JOINTS,
        joint_indices=list(range(7)),
        max_velocity=None,  # times already in seconds from HPP
    )

    gripper.destroy()

    print(f"\nResult: {'SUCCESS' if success else 'FAILED'}")
    return 0 if success else 1


if __name__ == "__main__":
    main()
