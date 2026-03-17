#!/usr/bin/env python3
"""
Example: Franka Emika Research (FER) planning with HPP.

Usage (in Docker):
    ./run.sh
    cd ~/devel/src && make all  # First time only
    python3 ~/devel/hpp-planning/examples/fer_example.py
"""

import os
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
URDF = os.path.join(PROJECT_DIR, "robots", "fer", "fer.urdf")
SRDF = os.path.join(PROJECT_DIR, "robots", "fer", "fer.srdf")

FER_JOINTS = [
    "fer_joint1",
    "fer_joint2",
    "fer_joint3",
    "fer_joint4",
    "fer_joint5",
    "fer_joint6",
    "fer_joint7",
]


def plan_with_hpp(q_init: np.ndarray, q_goal: np.ndarray):
    """Plan using pyhpp."""
    from pyhpp.pinocchio import Device, urdf
    from pyhpp.core import Problem, BiRRTPlanner, RandomShortcut
    from pinocchio import SE3

    robot = Device("fer")
    urdf.loadModel(robot, 0, "fer", "anchor", URDF, SRDF, SE3.Identity())
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
    print("FER Example")
    print("=" * 40)

    if not os.path.exists(URDF):
        print(f"URDF not found: {URDF}")
        return

    # FER home and a goal configuration
    q_init = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    q_goal = np.array([1.0, -0.5, 0.5, -2.0, 0.3, 1.2, 1.0])

    waypoints, times = plan_with_hpp(q_init, q_goal)

    if waypoints is None:
        print("Planning failed!")
        return

    print(f"Got {len(waypoints)} waypoints")

    # Test trajectory conversion
    from hpp_planner import send_trajectory

    send_trajectory(
        waypoints, times,
        joint_names=FER_JOINTS,
        max_velocity=0.5,
    )

    print("SUCCESS")


if __name__ == "__main__":
    main()
