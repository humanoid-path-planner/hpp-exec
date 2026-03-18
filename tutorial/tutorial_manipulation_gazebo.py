#!/usr/bin/env python3
"""
FR3 Pick-and-Place — Gazebo Execution
======================================

Imports the HPP manipulation plan from tutorial_manipulation.py
and sends it to Gazebo with gripper coordination.

Prerequisites:
    # Terminal 1: Launch Gazebo
    ./hpp-exec/scripts/launch_gazebo_gripper.sh

    # Terminal 2: Run
    docker exec -it hpp-exec bash
    python3 ~/devel/hpp-exec/tutorial/tutorial_manipulation_gazebo.py
"""

import sys

from tutorial_manipulation import (
    plan_pick_and_place,
    extract_waypoints,
    FR3_ARM_JOINTS,
)
from hpp_exec import execute_manipulation
from hpp_exec.gripper import (
    JointTrajectoryGripperController,
    extract_grasp_transitions,
)


def main():
    # --- Plan ---
    robot, problem, cg, path = plan_pick_and_place()
    if path is None:
        return 1

    full_wp, arm_wp, times = extract_waypoints(path)

    # --- Detect grasp transitions ---
    transitions = extract_grasp_transitions(full_wp, times, cg)
    print(f"\nGrasp transitions: {len(transitions)}")
    for t in transitions:
        action = "CLOSE" if t.acquired else "OPEN"
        print(f"  t={t.time:.2f}s (wp {t.waypoint_index}): {action}")

    # --- Gripper controller ---
    gripper = JointTrajectoryGripperController(
        topic="/gripper_controller/follow_joint_trajectory",
        joint_names=["fr3_finger_joint1"],
        open_positions=[0.04],
        close_positions=[0.0],
        duration=1.0,
    )

    # --- Execute ---
    print("\nExecuting on Gazebo...")
    arm_indices = list(range(7))
    success = execute_manipulation(
        full_wp, times,
        arm_joint_names=FR3_ARM_JOINTS,
        arm_joint_indices=arm_indices,
        gripper_controller=gripper,
        graph=cg,
        max_velocity=0.3,
    )

    gripper.destroy()

    print(f"\nResult: {'SUCCESS' if success else 'FAILED'}")
    return 0 if success else 1


if __name__ == "__main__":
    main()
