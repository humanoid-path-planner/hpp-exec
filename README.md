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
# ... solve and get configs ...

# Execute on robot
send_trajectory(
    configs, times,
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
from hpp_exec import send_trajectory, configs_to_joint_trajectory

# Main function - send trajectory to ros2_control
send_trajectory(
    configs,              # List[np.ndarray] from HPP
    times,                  # List[float] path parameter values
    joint_names,            # List[str] ROS2 joint names
    max_velocity=1.0,       # Rescale times to respect velocity limit
    max_acceleration=0.5,   # Rescale times to respect acceleration limit
    controller_topic="...", # FollowJointTrajectory action topic
)

# Lower-level conversion
trajectory = configs_to_joint_trajectory(configs, times, joint_names)
```

## Examples

Examples use the FR3 robot in Gazebo simulation or real hardware.

```bash
# Terminal 1: Launch Gazebo with FR3
./run.sh
./hpp-exec/scripts/launch_gazebo_gripper.sh

# Terminal 2: Run example
docker exec -it hpp-exec bash
python3 ~/devel/hpp-exec/examples/simple_trajectory.py
```

See [examples/README.md](examples/README.md) for all examples.

## Structure

```
hpp-exec/
├── hpp_exec/           # Python package
│   ├── __init__.py
│   ├── trajectory_utils.py # HPP config → ROS2 JointTrajectory conversion
│   ├── ros2_sender.py     # send_trajectory() via FollowJointTrajectory action
│   └── gripper.py         # Gripper coordination for manipulation trajectories
├── examples/              # Usage examples (Gazebo + real hardware)
├── scripts/               # Launch scripts for Gazebo
├── robots/                # URDF/SRDF for examples
├── docs/                  # Architecture documentation
├── docker/
├── Dockerfile
└── run.sh
```

## License

BSD-2-Clause
