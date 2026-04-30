# hpp-exec

ROS2 execution utilities for HPP-generated trajectories.

## Overview

You write your HPP planning script using `pyhpp` directly, then use `hpp_exec` to send the trajectory to `ros2_control`.

## Tutorials

The official tutorials for this package are in [hpp_tutorial](https://github.com/humanoid-path-planner/hpp_tutorial):

- **Tutorial 6**: Plan a simple arm motion and execute on Gazebo via `send_trajectory()`
- **Tutorial 7**: Pick-and-place with gripper actions using `segments_from_graph()` and `execute_segments()`

## Documentation

- [Generated Doxygen documentation](https://gepetto.github.io/doc/hpp-exec/doxygen-html/index.html):
  user reference for the API, execution model, and troubleshooting.

## Creating configs and times from HPP

After planning and time parameterization with HPP, you have a `Path` object that maps time (seconds) to robot configurations. Sample it at regular intervals to get discrete configs:

```python
import numpy as np

# After solving and time parameterization:
# p_timed = ps.getPath(pathId)  # or via path optimizer

n_samples = 50
configs = []
times = []

for i in range(n_samples + 1):
    t = (i / n_samples) * p_timed.length()
    q, success = p_timed(t)
    if success:
        configs.append(np.array(q))
        times.append(t)

# configs: List[np.ndarray] - configuration vectors at each sample
# times: List[float] - timestamps in seconds
```

The HPP configuration vector typically includes all robot DOFs. Use `joint_indices` to select which joints to send to ros2_control (e.g., arm joints only, excluding fingers).

## Sending to ros2_control

```python
from hpp_exec import send_trajectory

send_trajectory(
    configs, times,
    joint_names=["joint1", "joint2", ...],  # ROS2 joint names
    joint_indices=list(range(7)),            # Which HPP config indices to use
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
from hpp_exec import (
    Segment,
    execute_segments,
    send_trajectory,
    segments_from_graph,
)

# Main function - send trajectory to ros2_control
send_trajectory(
    configs,              # List[np.ndarray] from HPP
    times,                # List[float] timestamps or path parameter values
    joint_names,            # List[str] ROS2 joint names
    time_parameterization="none",  # "none" for seconds, "trapezoidal" to rescale
    controller_topic="...", # FollowJointTrajectory action topic
)

# Split a manipulation path into executable pieces.
segments = segments_from_graph(
    configs, times, graph,
    on_grasp=close_gripper,
    on_release=open_gripper,
)
execute_segments(segments, configs, times, joint_names)
```

See the generated Doxygen documentation for `send_trajectory_async()`,
`configs_to_joint_trajectory()`, `add_time_parameterization()`, and other
lower-level helpers.

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
├── docker/
├── Dockerfile
└── run.sh
```

## License

BSD-2-Clause
