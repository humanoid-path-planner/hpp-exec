# hpp-planning

ROS2 execution utilities for HPP-generated trajectories.

## Overview

You write your HPP planning script using `pyhpp` directly, then use `hpp_planner` to send the trajectory to `ros2_control`.

```python
from pyhpp.pinocchio import Device, urdf
from pyhpp.core import Problem, BiRRTPlanner
from hpp_planner import send_trajectory

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
cd hpp-planning
./run.sh

# First time inside container:
cd ~/devel/src && make all
```

### API

```python
from hpp_planner import send_trajectory, waypoints_to_joint_trajectory

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

## Testing

```bash
# Terminal 1: Start mock controller
./run.sh python3 examples/mock_controller.py --urdf robots/ur5/ur5.urdf

# Terminal 2: View with rqt
./run.sh rqt_graph

# Terminal 3: Send test trajectory
./run.sh python3 -c "
from hpp_planner import send_trajectory
import numpy as np
waypoints = [np.array([0,0,0,0,0,0]), np.array([1.5,-0.5,-1.0,0.5,1.0,-0.5])]
send_trajectory(waypoints, [0.0, 3.0],
    joint_names=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
                 'wrist_1_joint','wrist_2_joint','wrist_3_joint'],
    max_velocity=0.5)
"
```

## Tutorial

Complete HPP + Gazebo pipeline:

```bash
# Quick test with mock controller
./run.sh python3 examples/mock_controller.py --urdf robots/fr3/fr3.urdf \
    --joints fr3_joint1 fr3_joint2 fr3_joint3 fr3_joint4 fr3_joint5 fr3_joint6 fr3_joint7
./run.sh python3 tutorial/tutorial_mock.py

# Full test with Gazebo
./run.sh python3 tutorial/tutorial_hpp_gazebo.py
```

See [tutorial/README.md](tutorial/README.md) for details.

## Structure

```
hpp-planning/
├── hpp_planner/           # Python package
│   ├── __init__.py
│   ├── ros2_sender.py     # send_trajectory()
│   └── trajectory_utils.py
├── tutorial/              # HPP + Gazebo tutorials
│   ├── tutorial_mock.py
│   └── tutorial_hpp_gazebo.py
├── examples/
│   ├── mock_controller.py # Test receiver
│   ├── ur5_example.py
│   ├── fer_example.py
│   └── fr3_example.py
├── robots/                # URDF/SRDF for examples
├── docker/
├── Dockerfile
└── run.sh
```

## License

BSD-2-Clause
