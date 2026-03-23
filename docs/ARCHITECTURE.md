# HPP-Exec: Execution Pipeline Architecture

This document explains how hpp-exec bridges HPP planning output to ROS2 robot execution.

## Overview

```
HPP Planning              ->   hpp-exec                  ->   Real Robot / Gazebo
(you write this)               (this package)                 (ros2_control)

pyhpp scripts             ->   send_trajectory()         ->   /joint_trajectory_controller
constraint graph          ->   segments_from_graph()     ->   Action servers
configs + times           ->   execute_segments()        ->   Controllers
```

**Your HPP script produces:** configs (joint configurations) + path parameter values
**hpp-exec converts to:** ROS2 JointTrajectory messages + action calls
**ros2_control executes:** sends commands to Gazebo or real robot

---

## Layer 1: ROS2 Controllers (robot/Gazebo side)

These are **not** part of hpp-exec - they run in Gazebo or on the real robot.

### What is a controller?

A controller is a ROS2 node that:
1. **Subscribes** to command interfaces (receives trajectory goals)
2. **Publishes** to hardware interfaces (sends joint commands)
3. Runs in a control loop (typically 1kHz on real hardware, 100Hz in Gazebo)

### Controller Manager

The `controller_manager` node loads and manages controllers. In Gazebo, it's launched automatically by `ros2_control` when you spawn a robot.

```bash
# See active controllers
ros2 control list_controllers

# Output:
# joint_trajectory_controller   active   joint_trajectory_controller/JointTrajectoryController
# gripper_controller            active   joint_trajectory_controller/JointTrajectoryController
```

### JointTrajectoryController

**Type:** `joint_trajectory_controller/JointTrajectoryController`
**Config:** `scripts/controllers_gripper.yaml`

```yaml
joint_trajectory_controller:
  ros__parameters:
    joints:
      - fr3_joint1
      - fr3_joint2
      # ... 7 arm joints
    command_interfaces:
      - position    # sends position commands
    state_interfaces:
      - position    # reads current position
      - velocity    # reads current velocity
```

This controller:
1. **Receives** goals on `/joint_trajectory_controller/follow_joint_trajectory` (action)
2. **Interpolates** between configs (cubic spline by default)
3. **Sends** position commands to each joint at the control rate
4. **Reports** success/failure when trajectory completes

---

## Layer 2: ROS2 Actions (communication mechanism)

### What is an Action?

An action is a ROS2 communication pattern for long-running tasks:
- **Goal** -> sent by client, contains the request
- **Feedback** -> sent during execution (progress updates)
- **Result** -> sent when complete (success/failure)

Unlike services (request/response), actions are **cancellable** and **non-blocking**.

### FollowJointTrajectory Action

**Type:** `control_msgs/action/FollowJointTrajectory`

```
# Goal
trajectory_msgs/JointTrajectory trajectory
    JointTrajectoryPoint[] points
        float64[] positions      # joint positions at this config
        float64[] velocities     # (optional) joint velocities
        Duration time_from_start # when to reach this point

# Result
int32 error_code   # 0 = success

# Feedback
JointTrajectoryPoint actual    # current position
JointTrajectoryPoint desired   # target position
JointTrajectoryPoint error     # difference
```

### Action Server vs Action Client

```
+----------------------+         +----------------------+
|  hpp-exec            |         |  ros2_control        |
|  (ACTION CLIENT)     | ------> |  (ACTION SERVER)     |
|                      |         |                      |
|  send_trajectory()   |  Goal   |  JointTrajectory     |
|  waits for result    | <------ |  Controller executes |
|                      |  Result |  trajectory          |
+----------------------+         +----------------------+
```

---

## Layer 3: hpp-exec internals

### File: `hpp_exec/ros2_sender.py`

#### `send_trajectory()` - The core function

```python
def send_trajectory(
    configs: list[np.ndarray],        # HPP configurations
    times: list[float],                # path parameter values
    joint_names: list[str],            # ["fr3_joint1", ...]
    max_velocity: float = 1.0,         # rad/s limit
    joint_indices: list[int] = None,   # which DOFs to extract
    controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
) -> bool:
```

**Execution flow:**
```
1. Extract arm joints from HPP config
   configs[:, joint_indices] -> arm-only positions

2. Time parameterization (if max_velocity provided)
   path parameter -> real time (seconds)

3. Build JointTrajectory message
   for each config:
     point.positions = [j1, j2, j3, j4, j5, j6, j7]
     point.time_from_start = Duration(sec=t)

4. Create ROS2 node with ActionClient
   client = ActionClient(node, FollowJointTrajectory, topic)

5. Send goal and wait
   future = client.send_goal_async(goal)
   rclpy.spin_until_future_complete(node, future)

6. Return success/failure
```

#### `Segment` dataclass

```python
@dataclass
class Segment:
    start_index: int   # first config (inclusive)
    end_index: int     # last config (exclusive)
    pre_actions: list[Callable[[], bool]] = []   # run BEFORE trajectory
    post_actions: list[Callable[[], bool]] = []  # run AFTER trajectory
```

A segment defines a slice of the trajectory plus callbacks.

#### `execute_segments()`

```python
def execute_segments(segments, configs, times, joint_names, ...):
    for segment in segments:
        # 1. Pre-actions (e.g., close gripper)
        for action in segment.pre_actions:
            if not action():
                return False

        # 2. Send trajectory slice
        seg_configs = configs[segment.start_index:segment.end_index]
        seg_times = times[segment.start_index:segment.end_index]
        send_trajectory(seg_configs, seg_times, joint_names, ...)

        # 3. Post-actions (e.g., open gripper)
        for action in segment.post_actions:
            if not action():
                return False

    return True
```

---

## Layer 4: Gripper coordination

### File: `hpp_exec/gripper.py`

#### `extract_grasp_transitions()`

Queries the HPP constraint graph to detect when grasps change:

```python
def extract_grasp_transitions(configs, times, graph):
    transitions = []
    prev_grasps = set()

    for i, cfg in enumerate(configs):
        state_name = graph.getStateFromConfiguration(cfg)
        # e.g., "free" or "gripper grasps box/handle"

        current_grasps = parse_state_name(state_name)
        # e.g., {"gripper grasps box/handle"}

        if current_grasps != prev_grasps:
            transitions.append(GraspTransition(
                config_index=i,
                acquired=current_grasps - prev_grasps,  # new grasps
                released=prev_grasps - current_grasps,  # lost grasps
            ))
        prev_grasps = current_grasps

    return transitions
```

#### `segments_from_graph()`

Converts transitions into executable segments:

```python
def segments_from_graph(configs, times, graph, on_grasp, on_release):
    transitions = extract_grasp_transitions(configs, times, graph)

    # Example output with pick-and-place:
    # transitions[0]: config 150, acquired={"gripper grasps box/handle"}
    # transitions[1]: config 300, released={"gripper grasps box/handle"}

    # Result:
    # Segment 0: [0, 150)   - no actions
    # Segment 1: [150, 300) - pre_actions=[gripper.close]
    # Segment 2: [300, 462) - pre_actions=[gripper.open]

    return segments
```

#### Gripper controllers

**Option A: `JointTrajectoryGripperController`** (for Gazebo)
```python
gripper = JointTrajectoryGripperController(
    topic="/gripper_controller/follow_joint_trajectory",
    joint_names=["fr3_finger_joint1"],
    open_positions=[0.04],   # meters
    close_positions=[0.0],
    duration=1.0,            # seconds to move
)
```
Uses same `FollowJointTrajectory` action as arm. Treats gripper as another joint.

**Option B: `GripperCommandController`** (for dedicated gripper hardware)
```python
gripper = GripperCommandController(
    topic="/gripper_controller/gripper_command",
    open_position=0.04,
    close_position=0.0,
    max_effort=50.0,  # Newtons
)
```
Uses `GripperCommand` action. More semantic (position + force), but requires `gripper_action_controller/GripperActionController` on robot side.

---

## Full Execution Diagram

```
+-------------------------------------------------------------------------+
| YOUR HPP SCRIPT                                                         |
|                                                                         |
|  from pyhpp.manipulation import Robot, Problem, ConstraintGraph         |
|  robot, problem, cg, path = plan_pick_and_place()                       |
|  configs = [path(t) for t in np.linspace(0, path.length(), 500)]        |
|                                                                         |
+-------------------------------------------------------------------------+
                                    |
                                    v
+-------------------------------------------------------------------------+
| SEGMENT BUILDING (hpp_exec/gripper.py)                                  |
|                                                                         |
|  segments = segments_from_graph(                                        |
|      configs, times, cg,                                                |
|      on_grasp=gripper.close,    # <- your callable                      |
|      on_release=gripper.open,   # <- your callable                      |
|  )                                                                      |
|                                                                         |
|  Result:                                                                |
|    Segment(0, 150, pre_actions=[])                    # approach        |
|    Segment(150, 300, pre_actions=[gripper.close])     # carry           |
|    Segment(300, 462, pre_actions=[gripper.open])      # retreat         |
|                                                                         |
+-------------------------------------------------------------------------+
                                    |
                                    v
+-------------------------------------------------------------------------+
| EXECUTION LOOP (hpp_exec/ros2_sender.py)                                |
|                                                                         |
|  execute_segments(segments, configs, times, joint_names)                |
|                                                                         |
|  FOR segment IN segments:                                               |
|    1. RUN pre_actions:                                                  |
|       --> gripper.close() -> FollowJointTrajectory action               |
|                              to /gripper_controller                     |
|                                                                         |
|    2. SEND trajectory:                                                  |
|       --> send_trajectory() -> FollowJointTrajectory action             |
|                                to /joint_trajectory_controller          |
|                                                                         |
|    3. RUN post_actions (if any)                                         |
|                                                                         |
+-------------------------------------------------------------------------+
                                    |
                                    v
+-------------------------------------------------------------------------+
| ROS2_CONTROL (Gazebo / Real Robot)                                      |
|                                                                         |
|  joint_trajectory_controller:                                           |
|    - receives JointTrajectory                                           |
|    - interpolates at 100Hz (Gazebo) / 1kHz (real)                       |
|    - sends position commands to joints                                  |
|                                                                         |
|  gripper_controller:                                                    |
|    - receives JointTrajectory (single point)                            |
|    - moves gripper finger to target position                            |
|                                                                         |
+-------------------------------------------------------------------------+
                                    |
                                    v
+-------------------------------------------------------------------------+
| GAZEBO PHYSICS                                                          |
|                                                                         |
|  - Simulates joint dynamics                                             |
|  - Publishes /joint_states at 100Hz                                     |
|  - Robot moves in visualization                                         |
|                                                                         |
+-------------------------------------------------------------------------+
```

---

## About controllers_gripper.yaml

**This file was created by us** - it's not from any official Franka package.

The official `franka_gazebo_bringup` only spawns a `joint_trajectory_controller` for the arm. We added a second controller for the gripper because:
1. We want to control gripper separately from arm
2. We want to stop arm -> close gripper -> resume arm (sequential)

---

## Controller Config Explained

### What each field means

```yaml
joint_trajectory_controller:    # <- Controller NAME (arbitrary)
  ros__parameters:              # <- ROS2 parameter namespace
    joints:                     # <- Which joints this controller manages
      - fr3_joint1
      - fr3_joint2
      - fr3_joint3
      - fr3_joint4
      - fr3_joint5
      - fr3_joint6
      - fr3_joint7
    command_interfaces:         # <- How to send commands to hardware
      - position                # Options: position, velocity, effort
    state_interfaces:           # <- What to read from hardware
      - position
      - velocity
```

### How the config gets loaded

1. **URDF defines hardware interface**
   ```xml
   <ros2_control name="fr3_arm" type="system">
     <joint name="fr3_joint1">
       <command_interface name="position"/>
       <state_interface name="position"/>
       <state_interface name="velocity"/>
     </joint>
   </ros2_control>
   ```

2. **Controller manager spawns**
   ```bash
   ros2 run controller_manager spawner joint_trajectory_controller \
        --param-file controllers_gripper.yaml
   ```

3. **Controller is now active**
   ```bash
   $ ros2 action list
   /joint_trajectory_controller/follow_joint_trajectory
   ```

### Why we have TWO controllers

Without a separate gripper controller:
- Arm and gripper would be ONE trajectory
- Can't stop arm mid-trajectory to close gripper
- HPP plans arm+gripper together, but we execute separately

With separate controllers:
- Arm moves to pre-grasp -> **STOP**
- Gripper closes -> **STOP**
- Arm moves to place -> **STOP**
- Gripper opens -> **STOP**
- Arm retreats

---

## Segment Execution Details

### The `execute_segments()` loop

```python
for i, segment in enumerate(segments):
    # PHASE 1: PRE-ACTIONS
    for action in segment.pre_actions:
        if not action():
            return False  # Fail-fast

    # PHASE 2: ARM TRAJECTORY
    seg_configs = configs[segment.start_index:segment.end_index]
    seg_times = times[segment.start_index:segment.end_index]

    if len(seg_configs) >= 2:
        # Normalize times: segment starts at t=0
        t0 = seg_times[0]
        seg_times = [t - t0 for t in seg_times]

        send_trajectory(seg_configs, seg_times, joint_names, ...)

    # PHASE 3: POST-ACTIONS
    for action in segment.post_actions:
        if not action():
            return False

return True
```

### Timeline visualization

```
Time ---------------------------------------------------------------------->

Segment 0: [0, 150)
|
+- pre_actions: []                     (none)
+- trajectory: configs 0->149          ====================
+- post_actions: []                    (none)

Segment 1: [150, 300)
|
+- pre_actions: [gripper.close]        == (1s gripper motion)
+- trajectory: configs 150->299        ============================
+- post_actions: []                    (none)

Segment 2: [300, 462)
|
+- pre_actions: [gripper.open]         == (1s gripper motion)
+- trajectory: configs 300->461        ==================================
+- post_actions: []                    (none)

                                       +--------------------------------+
                                         Everything is SEQUENTIAL
                                         (no parallel execution)
```

### Error handling: fail-fast

```python
if not action():
    return False  # Immediately stop
```

Why fail-fast?
- If gripper fails to close -> don't continue (might drop object)
- If arm trajectory fails -> don't send gripper commands
- Partial execution is dangerous in manipulation

---

## Constraint Graph -> Segments

### HPP Constraint Graph states

In HPP manipulation, states encode what the robot is holding:

```
"free"                              -> gripper empty
"fr3_gripper grasps box/handle"     -> holding box
"fr3_gripper grasps box/handle : l_gripper grasps cup/handle"
                                    -> both arms holding objects
```

### Visual: transitions -> segments

```
Configs: 0 --------------------- 150 --------------------- 300 --------------------- 462
                                  |                         |
         state: "free"            | state: "grasps"         | state: "free"
                                  |                         |
Transitions:                 [acquired]                [released]

Segments:  +---- Segment 0 ------++---- Segment 1 ----------++---- Segment 2 ----------+
           pre: []                pre: [close]               pre: [open]
```

### Dual-arm case

For two robots with prefixed joint names:

```
State: "left_gripper grasps box/handle : right_gripper grasps cup/handle"
```

The current `segments_from_graph()` has ONE `on_grasp` callback - it doesn't distinguish which gripper. For dual-arm, you'd need to extend this to map gripper names to specific controllers.
