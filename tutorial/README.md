# HPP-Planning Tutorials

All tutorials use the FR3 robot in Gazebo simulation.

## Setup

```bash
cd ~/devel/hpp-planning
./run.sh --rebuild   # first time only, to build the Docker image
./run.sh             # start the container

# Build HPP (first time only, needed for manipulation tutorial)
cd ~/devel/src && make all
```

## Simple Trajectory

Send a multi-pose trajectory to the FR3 arm in Gazebo. No HPP needed.

```bash
# Terminal 1: Launch Gazebo
./hpp-planning/scripts/launch_gazebo_gripper.sh

# Terminal 2: Run tutorial
docker exec -it hpp-planning bash
python3 ~/devel/hpp-planning/tutorial/tutorial_gazebo.py
```

## Pick-and-Place with HPP Manipulation Planning

Full pipeline: HPP plans a pick-and-place with constraint graph, then
hpp_planner sends arm trajectories and gripper commands to Gazebo.

Requires HPP to be built (`cd ~/devel/src && make all`).

```bash
# Terminal 1: Launch Gazebo
./hpp-planning/scripts/launch_gazebo_gripper.sh

# Terminal 2: Run manipulation tutorial
docker exec -it hpp-planning bash
python3 ~/devel/hpp-planning/tutorial/tutorial_manipulation.py
```

What it does:
1. Loads FR3 + cube into HPP manipulation planning
2. Creates constraint graph (free / grasped states)
3. Plans a pick-and-place trajectory with ManipulationPlanner
4. Detects grasp transitions from the constraint graph
5. Sends arm segments + gripper commands to Gazebo

## Gripper Coordination Test (No HPP)

Test gripper coordination with hardcoded trajectories (no HPP needed).

```bash
# Terminal 1: Launch Gazebo
./hpp-planning/scripts/launch_gazebo_gripper.sh

# Terminal 2: Run gripper test
docker exec -it hpp-planning bash
python3 ~/devel/hpp-planning/scripts/test_gripper_gazebo.py
```

## API Overview

```python
from hpp_planner import send_trajectory, execute_manipulation

# Simple trajectory (no gripper)
send_trajectory(waypoints, times, joint_names=JOINTS, max_velocity=1.0)

# Manipulation with gripper coordination
execute_manipulation(
    waypoints, times,
    arm_joint_names=JOINTS,
    arm_joint_indices=[0, 1, 2, 3, 4, 5, 6],
    gripper_controller=gripper,
    graph=constraint_graph,
)
```
