# HPP-Exec API Reference

## `hpp_exec.send_trajectory()`

Send a trajectory to a ros2_control `JointTrajectoryController`.

```python
from hpp_exec import send_trajectory

success = send_trajectory(
    configs,                # List[np.ndarray] — HPP configuration vectors
    times,                  # List[float] — timestamps (path params or seconds)
    joint_names,            # List[str] — ROS2 joint names
    max_velocity=1.0,       # float — rescale times to respect this limit (rad/s)
                            #   Required when times are HPP path parameters.
                            #   Pass None to skip rescaling (times already in seconds).
    max_acceleration=0.5,   # float — acceleration limit for rescaling (rad/s^2)
    joint_indices=None,     # List[int] — indices to extract from HPP config
                            #   Default: 0..len(joint_names)
    controller_topic="/joint_trajectory_controller/follow_joint_trajectory",
)
```

## `hpp_exec.execute_segments()`

Execute trajectory segments with pre/post action hooks. For each segment: run pre-actions, send arm trajectory, run post-actions. Fails fast on any error.

```python
from hpp_exec import execute_segments, Segment

segments = [
    Segment(0, 150),                                # approach
    Segment(150, 300, pre_actions=[gripper.close]),  # close then carry
    Segment(300, 462, pre_actions=[gripper.open]),   # open then retreat
]

success = execute_segments(
    segments,
    configs,                # List[np.ndarray] — full HPP configs
    times,                  # List[float] — timestamps
    joint_names,            # List[str] — arm joint names
    joint_indices=None,     # List[int] — arm DOF indices in HPP config
    max_velocity=None,      # float — velocity rescaling (None = skip)
    controller_topic="/joint_trajectory_controller/follow_joint_trajectory",
)
```

## `hpp_exec.Segment`

Dataclass pairing a trajectory slice with optional actions.

```python
@dataclass
class Segment:
    start_index: int                                    # inclusive
    end_index: int                                      # exclusive
    pre_actions: list[Callable[[], bool]] = []          # run before trajectory
    post_actions: list[Callable[[], bool]] = []         # run after trajectory
```

Actions are any callable returning `True` on success. Bound methods (`gripper.close`), lambdas, and plain functions all work.

## `hpp_exec.gripper.segments_from_graph()`

Auto-build segments from HPP constraint graph transitions.

```python
from hpp_exec.gripper import segments_from_graph

segments = segments_from_graph(
    configs,            # List[np.ndarray] — full HPP configs
    times,              # List[float] — timestamps
    graph,              # pyhpp.manipulation.Graph — constraint graph
    on_grasp=gripper.close,     # called when a grasp is acquired
    on_release=gripper.open,    # called when a grasp is released
)
```

Detects state changes by querying `graph.getStateFromConfiguration(q)` for each config. When the set of active grasps changes, it splits the trajectory and assigns `on_grasp`/`on_release` as pre-actions.

## `hpp_exec.gripper.extract_grasp_transitions()`

Detect grasp/release events without building segments.

```python
from hpp_exec.gripper import extract_grasp_transitions, GraspTransition

transitions = extract_grasp_transitions(configs, times, graph)
# Returns List[GraspTransition] with:
#   .config_index   — index in configs list
#   .time           — timestamp
#   .acquired       — set of new grasp names (close)
#   .released       — set of lost grasp names (open)
```

## `hpp_exec.configs_to_joint_trajectory()`

Convert HPP configs to a ROS2 `JointTrajectory` message (lower-level).

```python
from hpp_exec import configs_to_joint_trajectory

trajectory = configs_to_joint_trajectory(
    configs,            # List[np.ndarray]
    times,              # List[float] — must be in seconds
    joint_names,        # List[str]
    joint_indices=None, # List[int]
    velocities=None,    # List[np.ndarray] — optional per-point velocities
    accelerations=None, # List[np.ndarray] — optional per-point accelerations
)
```

## `hpp_exec.add_time_parameterization()`

Simple trapezoidal time scaling. Converts HPP path parameters to real seconds based on max joint displacement per step. For proper time parameterization, use HPP's `SimpleTimeParameterization` instead (see `examples/pick_and_place_franka_timed.py`).

```python
from hpp_exec import add_time_parameterization

real_times = add_time_parameterization(
    configs,                # List[np.ndarray]
    times,                  # List[float] — path parameters
    max_velocity=1.0,       # rad/s
    max_acceleration=0.5,   # rad/s^2
)
```

## Gripper Interface

hpp-exec does not include gripper controller implementations — those are robot-specific and belong in your project. Any object with `open() -> bool` and `close() -> bool` methods works with `segments_from_graph()` and `execute_segments()`.

```python
class MyGripper:
    def open(self) -> bool:
        # your implementation
        return True

    def close(self) -> bool:
        # your implementation
        return True
```

See `examples/gripper_controllers.py` for reference implementations (JointTrajectory, GripperCommand, and Franka-native).
