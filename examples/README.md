# HPP-Exec Examples

All examples use the FR3 robot. Gazebo examples run in Docker; Franka examples require real hardware.

## Setup

```bash
cd ~/devel/hpp-exec
./run.sh             # start the container

# Build HPP (first time only)
cd ~/devel/src && make all
```

## Examples

| File | Description | Target |
|------|-------------|--------|
| `simple_trajectory.py` | Send hardcoded poses, no HPP planning | Gazebo |
| `hpp_planning_gazebo.py` | HPP BiRRT planning + execution | Gazebo |
| `pick_and_place_planning.py` | HPP manipulation problem (importable) | Planning only |
| `pick_and_place_gazebo.py` | Pick-and-place with gripper on Gazebo | Gazebo |
| `pick_and_place_franka.py` | Pick-and-place with Franka gripper | Real hardware |
| `pick_and_place_franka_timed.py` | Same + HPP `SimpleTimeParameterization` | Real hardware |
| `mock_controller.py` | Mock ROS2 controller for testing | Local |

### Running Gazebo examples

```bash
# Terminal 1: Launch Gazebo
./hpp-exec/scripts/launch_gazebo_gripper.sh

# Terminal 2
docker exec -it hpp-exec bash
python3 ~/devel/hpp-exec/examples/<example>.py
```

### Running Franka examples

```bash
# Launch franka_ros2
ros2 launch franka_bringup franka.launch.py robot_ip:=<ROBOT_IP> arm_id:=fr3

# Run
python3 examples/pick_and_place_franka.py
```

## API Documentation

See [docs/API.md](../docs/API.md) for the full API reference.
