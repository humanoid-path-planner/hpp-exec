# HPP-Exec Tutorials

All tutorials use the FR3 robot in Gazebo simulation.

## Setup

```bash
cd ~/devel/hpp-exec
./run.sh --rebuild   # first time only, to build the Docker image
./run.sh             # start the container

# Build HPP (first time only)
cd ~/devel/src && make all
```

## 1. Simple Trajectory (no HPP)

Send a multi-pose trajectory to the FR3 arm in Gazebo.

```bash
# Terminal 1: Launch Gazebo
./hpp-exec/scripts/launch_gazebo_gripper.sh

# Terminal 2
docker exec -it hpp-exec bash
python3 ~/devel/hpp-exec/tutorial/tutorial_gazebo.py
```

## 2. HPP Planning + Gazebo

Uses HPP BiRRT to plan a collision-free path, then sends it to Gazebo.

```bash
# Terminal 1: Launch Gazebo
./hpp-exec/scripts/launch_gazebo_gripper.sh

# Terminal 2
docker exec -it hpp-exec bash
python3 ~/devel/hpp-exec/tutorial/tutorial_hpp_gazebo.py
```

## 3. HPP Manipulation (Pick-and-Place)

Two files, separating planning from execution:

- **`tutorial_manipulation.py`** — sets up the HPP manipulation problem
  (FR3 + cube, constraint graph, placement constraints) and solves it.
  Run standalone or use `python -i` to inspect the path in viser.

- **`tutorial_manipulation_gazebo.py`** — imports the plan from above,
  detects grasp transitions, and sends arm + gripper commands to Gazebo.

```bash
# Planning only (no Gazebo needed)
python3 -i ~/devel/hpp-exec/tutorial/tutorial_manipulation.py
# Then in the interactive session:
#   from pyhpp_viser import Viewer
#   v = Viewer(robot, problem); v.start(); v.loadPath(path)

# Full execution on Gazebo
# Terminal 1: Launch Gazebo
./hpp-exec/scripts/launch_gazebo_gripper.sh

# Terminal 2
docker exec -it hpp-exec bash
python3 ~/devel/hpp-exec/tutorial/tutorial_manipulation_gazebo.py
```

## API Overview

```python
from hpp_exec import send_trajectory, execute_manipulation

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
