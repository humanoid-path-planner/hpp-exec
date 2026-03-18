#!/usr/bin/env python3
"""
Example: Franka Research 3 (FR3) planning with HPP.

Usage (in Docker):
    ./run.sh
    cd ~/devel/src && make all  # First time only
    python3 ~/devel/hpp-exec/examples/fr3_example.py
"""

import os
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
URDF = os.path.join(PROJECT_DIR, "robots", "fr3", "fr3.urdf")
SRDF = os.path.join(PROJECT_DIR, "robots", "fr3", "fr3.srdf")

FR3_JOINTS = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]


def plan_with_hpp(q_init: np.ndarray, q_goal: np.ndarray):
    """Plan using pyhpp."""
    from pyhpp.pinocchio import Device, urdf
    from pyhpp.core import Problem, BiRRTPlanner, RandomShortcut
    from pinocchio import SE3

    robot = Device("fr3")
    urdf.loadModel(robot, 0, "fr3", "anchor", URDF, SRDF, SE3.Identity())
    print(f"Robot: {robot.name()}, {robot.configSize()} DOF")

    problem = Problem(robot)
    problem.initConfig(q_init)
    problem.resetGoalConfigs()
    problem.addGoalConfig(q_goal)

    planner = BiRRTPlanner(problem)
    planner.maxIterations(2000)

    print("Planning...")
    path = planner.solve()

    if path is None:
        return None, None

    print(f"Path length: {path.length():.3f}")

    optimizer = RandomShortcut(problem)
    path = optimizer.optimize(path)

    # Sample waypoints
    n_samples = max(int(path.length() / 0.01), 10)
    waypoints = []
    times = []

    for i in range(n_samples + 1):
        t = (i / n_samples) * path.length()
        q, success = path(t)
        if success:
            waypoints.append(np.array(q))
            times.append(t)

    return waypoints, times


def main():
    print("FR3 Example")
    print("=" * 40)

    if not os.path.exists(URDF):
        print(f"URDF not found: {URDF}")
        return

    # FR3 home and a goal configuration
    q_init = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    q_goal = np.array([1.0, -0.5, 0.5, -2.0, 0.3, 1.2, 1.0])

    waypoints, times = plan_with_hpp(q_init, q_goal)

    if waypoints is None:
        print("Planning failed!")
        return

    print(f"Got {len(waypoints)} waypoints")

    # Test trajectory conversion
    from hpp_exec import send_trajectory

    send_trajectory(
        waypoints, times,
        joint_names=FR3_JOINTS,
        max_velocity=0.5,
    )

    print("SUCCESS")


if __name__ == "__main__":
    main()
