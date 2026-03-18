# hpp-exec

ROS2 execution utilities for HPP-generated trajectories.

## Overview

You write your HPP planning script using `pyhpp` directly, then use `hpp_exec` to send the trajectory to `ros2_control`.

```python
from pyhpp.pinocchio import Device, urdf
from pyhpp.core import Problem, BiRRTPlanner
from hpp_exec import send_trajectory

# Your HPP planning script...
robot = Device("ur5")
urdf.loadModel(robot, ...)
problem = Problem(robot)
# ... solve and get waypoints ...

# Execute on robot
send_trajectory(
    waypoints, times,
    joint_names=["shoulder_pan_joint", "shoulder_lift_joint", ...],
    max_velocity=1.0,
)
```

## Installation

### Docker (recommended)

```bash
cd hpp-exec
./run.sh

# First time inside container:
cd ~/devel/src && make all
```

### API

```python
from hpp_exec import send_trajectory, waypoints_to_joint_trajectory

# Main function - send trajectory to ros2_control
send_trajectory(
    waypoints,              # List[np.ndarray] from HPP
    times,                  # List[float] path parameter values
    joint_names,            # List[str] ROS2 joint names
    max_velocity=1.0,       # Rescale times to respect velocity limit
    max_acceleration=0.5,   # Rescale times to respect acceleration limit
    controller_topic="...", # FollowJointTrajectory action topic
)

# Lower-level conversion
trajectory = waypoints_to_joint_trajectory(waypoints, times, joint_names)
```

## Tutorial

Tutorials use the FR3 robot in Gazebo simulation.

```bash
# Terminal 1: Launch Gazebo with FR3
./run.sh
./hpp-exec/scripts/launch_gazebo_gripper.sh

# Terminal 2: Run tutorial
docker exec -it hpp-exec bash
python3 ~/devel/hpp-exec/tutorial/tutorial_gazebo.py
```

See [tutorial/README.md](tutorial/README.md) for all tutorials.

## Structure

```
hpp-exec/
├── hpp_exec/           # Python package
│   ├── __init__.py
│   ├── trajectory_utils.py # HPP config → ROS2 JointTrajectory conversion
│   ├── ros2_sender.py     # send_trajectory() via FollowJointTrajectory action
│   └── gripper.py         # Gripper coordination for manipulation trajectories
├── scripts/               # Gazebo gripper testing (FR3)
├── tutorial/              # HPP tutorials
├── examples/              # Usage examples + mock controller
├── robots/                # URDF/SRDF for examples
├── docker/
├── Dockerfile
└── run.sh
```

## License

BSD-2-Clause
