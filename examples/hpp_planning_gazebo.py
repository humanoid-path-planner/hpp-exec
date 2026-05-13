#!/usr/bin/env python3
"""
FR3 HPP Planning + Gazebo Execution
====================================

Uses HPP to plan a collision-free trajectory for the FR3 arm,
then sends it to Gazebo via send_trajectory().

Prerequisites:
    # Build HPP (first time only)
    cd ~/devel/src && make all

    # Terminal 1: Launch Gazebo
    ./hpp-exec/scripts/launch_gazebo_gripper.sh

    # Terminal 2: Run
    docker exec -it hpp-exec bash
    python3 ~/devel/hpp-exec/examples/hpp_planning_gazebo.py
"""

import os
import sys
import time

import numpy as np
from pinocchio import SE3
from pyhpp.core import BiRRTPlanner, Problem, TrapezoidalTimeParameterization
from pyhpp.pinocchio import Device, urdf

from hpp_exec import send_trajectory

# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)

FR3_URDF = os.path.join(PROJECT_DIR, "robots", "fr3", "fr3.urdf")
FR3_SRDF = os.path.join(PROJECT_DIR, "robots", "fr3", "fr3.srdf")

FR3_ARM_JOINTS = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]


def main():
    plan_only = "--plan-only" in sys.argv

    print("=" * 50)
    print("FR3 HPP Planning + Gazebo")
    print("=" * 50)

    for f in [FR3_URDF, FR3_SRDF]:
        if not os.path.exists(f):
            print(f"Missing: {f}")
            return 1

    # --- Load robot ---
    robot = Device("fr3")
    urdf.loadModel(robot, 0, "fr3", "anchor", FR3_URDF, FR3_SRDF, SE3.Identity())
    print(f"  Config size: {robot.configSize()}")

    # --- Setup problem ---
    problem = Problem(robot)

    # FR3 config: 7 arm joints + 2 finger joints (including mimic)
    # Fingers stay open (0.035), we only vary the arm.
    q_init = np.array(
        [
            0.0,
            -0.785,
            0.0,
            -2.356,
            0.0,
            1.571,
            0.785,  # arm (ready pose)
            0.035,
            0.035,  # fingers
        ]
    )
    q_goal = np.array(
        [
            0.5,
            -0.3,
            0.3,
            -1.5,
            0.2,
            1.2,
            0.5,  # arm (different pose)
            0.035,
            0.035,  # fingers
        ]
    )

    problem.initConfig(q_init)
    problem.addGoalConfig(q_goal)

    # --- Plan ---
    planner = BiRRTPlanner(problem)
    planner.maxIterations(5000)

    print("\nPlanning...")
    start = time.time()
    path = planner.solve()

    if path is None:
        print("  Planning FAILED")
        return 1

    elapsed = time.time() - start
    print(f"  Solved in {elapsed:.1f}s, path length: {path.length():.3f}")

    # --- Time parameterization ---
    optimizer = TrapezoidalTimeParameterization(problem)
    optimizer.maxVelocity = 0.5
    optimizer.maxAcceleration = 0.5
    timed_path = optimizer.optimize(path)
    print(f"  Timed duration: {timed_path.length():.2f}s")

    # --- Extract configs ---
    n_samples = max(int(timed_path.length() / 0.02), 50)
    configs = []
    times = []
    for i in range(n_samples + 1):
        t = (i / n_samples) * timed_path.length()
        q, success = timed_path(t)
        if success:
            # Extract arm joints only (indices 0-6), skip fingers
            configs.append(np.array(q[:7]))
            times.append(t)

    print(f"  Extracted {len(configs)} configs")
    print(f"  Start: {configs[0]}")
    print(f"  End:   {configs[-1]}")

    if plan_only:
        print("\n--plan-only: skipping Gazebo execution")
        return 0

    # --- Send to Gazebo ---
    print("\nSending to Gazebo...")
    success = send_trajectory(
        configs,
        times,
        joint_names=FR3_ARM_JOINTS,
    )

    if not success:
        print("  FAILED - is Gazebo running?")
        print("  Launch with: ./hpp-exec/scripts/launch_gazebo_gripper.sh")
        return 1

    print("Done!")
    return 0


if __name__ == "__main__":
    main()
