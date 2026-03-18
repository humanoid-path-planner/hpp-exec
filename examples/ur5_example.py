#!/usr/bin/env python3
"""
Example: UR5 planning with HPP + trajectory execution via hpp_exec.

This script shows the intended workflow:
1. Use pyhpp directly for motion planning
2. Extract waypoints from the HPP path
3. Use hpp_exec utilities to send trajectory to ros2_control

Prerequisites:
    - pyhpp installed (hpp-python native bindings)
    - ROS2 Jazzy with control_msgs, trajectory_msgs
    - ROS_PACKAGE_PATH includes example-robot-data (for mesh resolution)

Usage (in Docker):
    ./run.sh
    cd ~/devel/src && make all  # First time only
    python3 ~/devel/hpp-exec/examples/ur5_example.py
"""

import sys
import os
import numpy as np

# Resolve package:// URIs for URDF mesh loading
if "ROS_PACKAGE_PATH" not in os.environ:
    for candidate in ["/opt/openrobots/share", os.path.expanduser("~/devel/install/share")]:
        if os.path.isdir(os.path.join(candidate, "example-robot-data")):
            os.environ["ROS_PACKAGE_PATH"] = candidate
            break

# Paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
URDF = os.path.join(PROJECT_DIR, "robots", "ur5", "ur5.urdf")
SRDF = os.path.join(PROJECT_DIR, "robots", "ur5", "ur5.srdf")

# UR5 joint names (must match ros2_control config)
UR5_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def plan_with_hpp(q_init: np.ndarray, q_goal: np.ndarray):
    """
    Plan a path using HPP directly (pyhpp native bindings).

    This is where users write their own HPP logic:
    - Constraint graphs for manipulation
    - Path optimizers
    - Custom planners
    """
    from pyhpp.pinocchio import Device, urdf
    from pyhpp.core import Problem, BiRRTPlanner, RandomShortcut
    from pinocchio import SE3

    # Load robot
    robot = Device("ur5")
    urdf.loadModel(robot, 0, "ur5", "anchor", URDF, SRDF, SE3.Identity())

    print(f"Robot: {robot.name()}, {robot.configSize()} DOF")

    # Create problem
    problem = Problem(robot)

    # Set initial and goal configs
    problem.initConfig(q_init)
    problem.resetGoalConfigs()
    problem.addGoalConfig(q_goal)

    # Solve with BiRRT
    planner = BiRRTPlanner(problem)
    planner.maxIterations(2000)

    print("Planning...")
    path = planner.solve()

    if path is None:
        print("Planning failed!")
        return None, None

    print(f"Path found, length: {path.length():.3f}")

    # Apply path optimizer
    optimizer = RandomShortcut(problem)
    path = optimizer.optimize(path)
    print(f"After optimization: {path.length():.3f}")

    # Extract waypoints (sample path at ~1cm resolution)
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


def test_trajectory_conversion(waypoints, times):
    """Test trajectory conversion without sending to controller."""
    from hpp_exec import waypoints_to_joint_trajectory, add_time_parameterization

    # Scale path parameter to real time
    scaled_times = add_time_parameterization(
        waypoints, times,
        max_velocity=1.0,
        max_acceleration=0.5,
    )
    print(f"Time parameterization: {scaled_times[-1]:.2f}s total")

    # Convert to ROS2 message
    trajectory = waypoints_to_joint_trajectory(waypoints, scaled_times, UR5_JOINTS)

    print(f"JointTrajectory: {len(trajectory.points)} points, {len(trajectory.joint_names)} joints")

    # Validate
    assert len(trajectory.joint_names) == 6
    assert len(trajectory.points) > 0
    assert len(trajectory.points[0].positions) == 6

    return trajectory


def main():
    print("=" * 60)
    print("hpp-exec Example: UR5")
    print("=" * 60)

    # Check URDF exists
    if not os.path.exists(URDF):
        print(f"ERROR: URDF not found at {URDF}")
        sys.exit(1)

    # Define configurations
    q_init = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
    q_goal = np.array([2.5, -0.5, -0.8, -1.5, 1.5, 1.0])

    print(f"\nq_init: {q_init}")
    print(f"q_goal: {q_goal}")

    # Step 1: Plan with HPP (user's own script)
    print("\n--- Step 1: Plan with pyhpp ---")
    waypoints, times = plan_with_hpp(q_init, q_goal)

    if waypoints is None:
        print("Planning failed!")
        sys.exit(1)

    print(f"Got {len(waypoints)} waypoints")

    # Step 2: Test trajectory conversion (without sending)
    print("\n--- Step 2: Convert to ROS2 trajectory ---")
    trajectory = test_trajectory_conversion(waypoints, times)

    # Step 3: (Optional) Send to ros2_control
    # Uncomment when running with a real or simulated robot:
    #
    # from hpp_exec import send_trajectory
    # success = send_trajectory(
    #     waypoints, times,
    #     joint_names=UR5_JOINTS,
    #     max_velocity=1.0,
    #     controller_topic="/joint_trajectory_controller/follow_joint_trajectory",
    # )
    # print(f"Trajectory sent: {success}")

    print("\n" + "=" * 60)
    print("SUCCESS - Example completed")
    print("=" * 60)


if __name__ == "__main__":
    main()
