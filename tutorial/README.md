# HPP-Planning Tutorials

All tutorials use the FR3 robot in Gazebo simulation.

## Setup

```bash
cd ~/devel/hpp-planning
./run.sh --rebuild   # first time only, to build the Docker image
./run.sh             # start the container
```

## Simple Trajectory

Send a multi-pose trajectory to the FR3 arm in Gazebo.

```bash
# Terminal 1: Launch Gazebo
./hpp-planning/scripts/launch_gazebo_gripper.sh

# Terminal 2: Run tutorial
docker exec -it hpp-planning bash
python3 ~/devel/hpp-planning/tutorial/tutorial_gazebo.py
```

## Pick-and-Place (Gripper Coordination)

Demonstrates gripper open/close interleaved with arm trajectories.

```bash
# Terminal 1: Launch Gazebo (same as above)
./hpp-planning/scripts/launch_gazebo_gripper.sh

# Terminal 2: Run gripper test
docker exec -it hpp-planning bash
python3 ~/devel/hpp-planning/scripts/test_gripper_gazebo.py
```

## API Overview

```python
from hpp_planner import send_trajectory

# times are HPP path parameters — max_velocity rescales to real time
send_trajectory(waypoints, times, joint_names=JOINTS, max_velocity=1.0)
```
