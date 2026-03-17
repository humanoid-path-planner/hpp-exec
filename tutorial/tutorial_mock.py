#!/usr/bin/env python3
"""
HPP + Mock Controller Tutorial
==============================

Tests the full HPP planning + hpp_planner execution pipeline.

Usage:
    # Terminal 1: Start mock controller
    cd hpp-planning
    python3 examples/mock_controller.py --urdf robots/ur5/ur5.urdf

    # Terminal 2: Run this script
    cd hpp-planning
    python3 tutorial/tutorial_mock.py
"""

import os
import time
import numpy as np

print("\n" + "=" * 60)
print("HPP Planning + Mock Controller Test")
print("=" * 60)

from pinocchio import SE3
from pyhpp.pinocchio import Device, urdf
from pyhpp.core import Problem, BiRRTPlanner

# Robot file paths (UR5 - meshes available via example-robot-data)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
URDF_FILE = os.path.join(PROJECT_DIR, "robots", "ur5", "ur5.urdf")
SRDF_FILE = os.path.join(PROJECT_DIR, "robots", "ur5", "ur5.srdf")

print(f"\nLoading UR5 robot...")

robot = Device("ur5")
urdf.loadModel(robot, 0, "ur5", "anchor", URDF_FILE, SRDF_FILE, SE3.Identity())
print(f"  Loaded: {robot.configSize()} DOF")

problem = Problem(robot)

# UR5 start and goal configurations (6 DOF)
q_home = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
q_target = np.array([1.5, -0.5, -1.0, 0.5, 1.5, 1.0])

problem.resetGoalConfigs()
problem.initConfig(q_home)
problem.addGoalConfig(q_target)

planner = BiRRTPlanner(problem)
planner.maxIterations(2000)

print("\nPlanning...")
start = time.time()
path = planner.solve()

if path is None:
    print("Planning failed!")
    exit(1)

print(f"  Done in {time.time() - start:.2f}s, path length: {path.length():.3f}")

# Extract waypoints
waypoints = []
times = []
n_samples = max(int(path.length() / 0.01), 10)

for i in range(n_samples + 1):
    t = (i / n_samples) * path.length()
    q, success = path(t)
    if success:
        waypoints.append(np.array(q))
        times.append(t)

print(f"  {len(waypoints)} waypoints")

# Send to mock controller
print("\nSending to mock controller...")

from hpp_planner import send_trajectory

UR5_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

success = send_trajectory(
    waypoints,
    times,
    joint_names=UR5_JOINTS,
    max_velocity=0.5,
)

if success:
    print("\nSUCCESS!")
else:
    print("\nFailed - is mock_controller running?")
