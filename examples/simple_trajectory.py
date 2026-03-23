#!/usr/bin/env python3
"""
FR3 Simple Trajectory
=====================

Send a simple trajectory to the FR3 robot in Gazebo simulation.
Demonstrates send_trajectory() with time parameterization.

Prerequisites:
    # Terminal 1: Launch Gazebo with FR3
    ./hpp-exec/scripts/launch_gazebo_gripper.sh

    # Terminal 2: Run
    docker exec -it hpp-exec bash
    python3 ~/devel/hpp-exec/examples/simple_trajectory.py
"""

import sys
import numpy as np

from hpp_exec import send_trajectory


FR3_JOINTS = [
    "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
    "fr3_joint5", "fr3_joint6", "fr3_joint7",
]

# FR3 key poses (7 DOF)
HOME = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
POSE_A = np.array([0.5, -0.3, 0.3, -1.5, 0.2, 1.2, 0.5])
POSE_B = np.array([-0.5, -0.3, -0.3, -1.5, -0.2, 1.2, -0.5])


def interpolate(start, end, n_points=30):
    """Linear interpolation between two configurations."""
    configs = []
    times = []
    for i in range(n_points):
        t = i / (n_points - 1)
        configs.append(start + t * (end - start))
        times.append(t)  # path parameter, not real time
    return configs, times


def main():
    print("\n" + "=" * 50)
    print("FR3 Gazebo Tutorial")
    print("=" * 50)

    # Build a multi-segment trajectory: home → A → B → home
    segments = [
        ("Home -> Pose A", HOME, POSE_A),
        ("Pose A -> Pose B", POSE_A, POSE_B),
        ("Pose B -> Home", POSE_B, HOME),
    ]

    for name, start, end in segments:
        configs, times = interpolate(start, end, n_points=30)

        print(f"\n  {name} ({len(configs)} configs)...")

        # times are path parameters (0 to 1), NOT real seconds.
        # max_velocity rescales them to respect joint velocity limits.
        success = send_trajectory(
            configs, times,
            joint_names=FR3_JOINTS,
            max_velocity=0.5,
        )

        if not success:
            print("  FAILED - is Gazebo running?")
            print("  Launch with: ./hpp-exec/scripts/launch_gazebo_gripper.sh")
            return 1

        print(f"  Done.")

    print("\nAll segments complete!")
    return 0


if __name__ == "__main__":
    main()
