#!/usr/bin/env python3
"""
FR3 Pick-and-Place -- Real Hardware Execution
==============================================

Same HPP plan as pick_and_place_gazebo.py, but uses the native
Franka gripper actions (Grasp/Move) instead of JointTrajectoryController.

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

from pick_and_place_planning import (
    plan_pick_and_place,
    extract_configs,
    FR3_ARM_JOINTS,
)
from hpp_exec import execute_segments
from hpp_exec.gripper import segments_from_graph, extract_grasp_transitions
from gripper_controllers import FrankaGripperController


def main():
    # --- Plan ---
    robot, problem, cg, path = plan_pick_and_place()
    if path is None:
        return 1

    full_configs, arm_configs, times = extract_configs(path)

    # --- Log transitions ---
    transitions = extract_grasp_transitions(full_configs, times, cg)
    print(f"\nGrasp transitions: {len(transitions)}")
    for t in transitions:
        action = "GRASP" if t.acquired else "RELEASE"
        print(f"  t={t.time:.2f}s (config {t.config_index}): {action}")

    # --- Franka gripper (real hardware) ---
    gripper = FrankaGripperController(
        arm_id="fr3",
        open_width=0.08,       # fully open
        grasp_width=0.02,      # cube is 4cm, leave margin
        grasp_force=50.0,      # Newtons
        grasp_speed=0.05,      # m/s
    )

    # --- Build segments from constraint graph ---
    segments = segments_from_graph(
        full_configs, times, cg,
        on_grasp=gripper.close,
        on_release=gripper.open,
    )

    # --- Execute ---
    print(f"\nExecuting {len(segments)} segments on real FR3...")
    success = execute_segments(
        segments, full_configs, times,
        joint_names=FR3_ARM_JOINTS,
        joint_indices=list(range(7)),
        max_velocity=0.3,
    )

    gripper.destroy()

    print(f"\nResult: {'SUCCESS' if success else 'FAILED'}")
    return 0 if success else 1


if __name__ == "__main__":
    main()
