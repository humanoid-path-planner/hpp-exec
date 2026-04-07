#!/usr/bin/env python3
"""
FR3 Pick-and-Place — Gazebo Execution
======================================

Imports the HPP manipulation plan from pick_and_place_planning.py
and sends it to Gazebo with gripper coordination.

Prerequisites:
    # Terminal 1: Launch Gazebo
    ./hpp-exec/scripts/launch_gazebo_gripper.sh

    # Terminal 2: Run
    docker exec -it hpp-exec bash
    python3 ~/devel/hpp-exec/examples/pick_and_place_gazebo.py
"""

from gripper_controllers import JointTrajectoryGripperController
from pick_and_place_planning import (
    FR3_ARM_JOINTS,
    extract_configs,
    plan_pick_and_place,
)

from hpp_exec import execute_segments
from hpp_exec.gripper import extract_grasp_transitions, segments_from_graph


def main():
    # --- Plan ---
    robot, problem, cg, path = plan_pick_and_place()
    if path is None:
        return 1

    full_configs, arm_configs, times = extract_configs(path)

    # --- Detect grasp transitions (for logging) ---
    transitions = extract_grasp_transitions(full_configs, times, cg)
    print(f"\nGrasp transitions: {len(transitions)}")
    for t in transitions:
        action = "CLOSE" if t.acquired else "OPEN"
        print(f"  t={t.time:.2f}s (config {t.config_index}): {action}")

    # --- Gripper controller (FR3-specific) ---
    gripper = JointTrajectoryGripperController(
        topic="/gripper_controller/follow_joint_trajectory",
        joint_names=["fr3_finger_joint1"],
        open_positions=[0.04],
        close_positions=[0.0],
        duration=1.0,
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
    print(f"\nExecuting {len(segments)} segments on Gazebo...")
    success = execute_segments(
        segments,
        full_configs,
        times,
        joint_names=FR3_ARM_JOINTS,
        joint_indices=list(range(7)),
        max_velocity=0.3,
    )

    gripper.destroy()

    print(f"\nResult: {'SUCCESS' if success else 'FAILED'}")
    return 0 if success else 1


if __name__ == "__main__":
    main()
