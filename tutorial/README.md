# HPP-Planning Tutorials

## Files

- `example_api_usage.py` - Shows the before/after API comparison
- `tutorial_mock.py` - Complete working example with mock controller (UR5)

## Quick Test with Mock Controller

```bash
# Terminal 1: Start mock controller
cd ~/devel/hpp-planning
./run.sh
python3 examples/mock_controller.py --urdf robots/ur5/ur5.urdf

# Terminal 2: Run tutorial
docker exec -it hpp-planning bash
python3 tutorial/tutorial_mock.py
```

## API Comparison

**Before (40+ lines of ROS2 code):**
```python
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
# ... create node, action client, build message manually ...
```

**After (3 lines with hpp_planner):**
```python
from hpp_planner import send_trajectory
send_trajectory(waypoints, times, joint_names=JOINTS)
```
