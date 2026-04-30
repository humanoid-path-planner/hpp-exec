//
// Copyright (c) 2026 CNRS
// Author: Paul Sardin
//
// BSD 2-Clause License

/// \mainpage hpp-exec
/// \anchor hpp_exec_documentation
///
/// \section hpp_exec_overview Overview
///
/// \c hpp-exec is a thin Python layer that takes a path produced by HPP and
/// runs it on a real or simulated robot through \c ros2_control. It does
/// **no planning** of its own: you write your HPP script using \c pyhpp,
/// sample the resulting path into a list of configurations, then hand them
/// over to \c hpp-exec to be packaged as a \c JointTrajectory message and
/// executed via the \c FollowJointTrajectory action.
///
/// The package source code is on
/// <a href="https://github.com/humanoid-path-planner/hpp-exec">GitHub</a>.
/// End-to-end examples are provided as tutorials 6 and 7 of
/// <a href="https://github.com/humanoid-path-planner/hpp_tutorial">hpp_tutorial</a>.
///
/// \section hpp_exec_workflow Typical workflow
///
/// A minimal usage looks like this:
///
/// \code{.py}
/// import numpy as np
/// from hpp_exec import send_trajectory
///
/// # 1. Plan with HPP (using pyhpp directly).
/// path = problem.solve()
///
/// # 2. Sample the path into discrete waypoints.
/// n = 100
/// configs = [np.array(path(t * path.length() / n)[0]) for t in range(n + 1)]
/// times   = [t * path.length() / n for t in range(n + 1)]
///
/// # 3. Send to ros2_control.
/// send_trajectory(
///     configs, times,
///     joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
///     time_parameterization="trapezoidal",
///     max_velocity=1.0,
/// )
/// \endcode
///
/// The \c configs / \c times lists are the only data structure exchanged
/// between HPP and \c hpp-exec. \c configs holds full HPP configuration
/// vectors (which may include object DOFs, gripper fingers, etc.); the
/// \c joint_indices argument selects which entries belong to the arm. The
/// \c times list is the timestamp of each config, and may be either real
/// seconds or path parameter values --- see the next section.
///
/// \section hpp_exec_api Quick API reference
///
/// Import the public API from \c hpp_exec:
///
/// \code{.py}
/// from hpp_exec import (
///     Segment,
///     GraspTransition,
///     send_trajectory,
///     send_trajectory_async,
///     execute_segments,
///     segments_from_graph,
///     extract_grasp_transitions,
///     configs_to_joint_trajectory,
///     add_time_parameterization,
///     extract_joint_config,
/// )
/// \endcode
///
/// Main execution functions:
///
/// \code{.py}
/// send_trajectory(
///     configs: list[np.ndarray],
///     times: list[float],
///     joint_names: list[str],
///     controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
///     time_parameterization: str = "none",
///     max_velocity: float = 1.0,
///     max_acceleration: float = 0.5,
///     joint_indices: list[int] | None = None,
/// ) -> bool
///
/// send_trajectory_async(
///     configs: list[np.ndarray],
///     times: list[float],
///     joint_names: list[str],
///     controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
///     time_parameterization: str = "none",
///     max_velocity: float = 1.0,
///     max_acceleration: float = 0.5,
///     joint_indices: list[int] | None = None,
/// )
///
/// execute_segments(
///     segments: list[Segment],
///     configs: list[np.ndarray],
///     times: list[float],
///     joint_names: list[str],
///     joint_indices: list[int] | None = None,
///     time_parameterization: str = "none",
///     max_velocity: float = 1.0,
///     max_acceleration: float = 0.5,
///     controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
/// ) -> bool
/// \endcode
///
/// Segment and grasp helpers:
///
/// \code{.py}
/// Segment(
///     start_index: int,
///     end_index: int,
///     pre_actions: list[Callable[[], bool]] = [],
///     post_actions: list[Callable[[], bool]] = [],
/// )
///
/// GraspTransition(
///     config_index: int,
///     time: float,
///     grasps_before: set[str],
///     grasps_after: set[str],
///     state_before: str = "",
///     state_after: str = "",
///     transition_name: str | None = None,
///     acquired: set[str] = set(),
///     released: set[str] = set(),
/// )
///
/// segments_from_graph(
///     configs: list[np.ndarray],
///     times: list[float],
///     graph,
///     on_grasp: Callable[[], bool] | Callable[[GraspTransition], bool] | dict,
///     on_release: Callable[[], bool] | Callable[[GraspTransition], bool] | dict,
/// ) -> list[Segment]
///
/// extract_grasp_transitions(
///     configs: list[np.ndarray],
///     times: list[float],
///     graph,
/// ) -> list[GraspTransition]
/// \endcode
///
/// Lower-level conversion utilities:
///
/// \code{.py}
/// configs_to_joint_trajectory(
///     configs: list[np.ndarray],
///     times: list[float],
///     joint_names: list[str],
///     joint_indices: list[int] | None = None,
///     velocities: list[np.ndarray] | None = None,
///     accelerations: list[np.ndarray] | None = None,
/// ) -> trajectory_msgs.msg.JointTrajectory
///
/// add_time_parameterization(
///     configs: list[np.ndarray],
///     times: list[float],
///     max_velocity: float = 1.0,
///     max_acceleration: float = 0.5,
/// ) -> list[float]
///
/// extract_joint_config(
///     hpp_config: np.ndarray,
///     n_joints: int,
///     offset: int = 0,
/// ) -> list[float]
/// \endcode
///
/// \section hpp_exec_time Time parameterization
///
/// HPP paths are parameterized by a path parameter \f$s\f$, not by time.
/// Before sending a trajectory to \c ros2_control, the parameter values
/// have to be turned into seconds. \c hpp-exec offers two strategies,
/// selected by the \c time_parameterization argument of \c send_trajectory:
///
/// \li \c "none" (default): \c times is assumed to already contain real
///     seconds. Use this when HPP itself has time-parameterized the path
///     (e.g.\ via the \c SimpleTimeParameterization path optimizer): the
///     parameter values returned by the optimized path are then proper
///     timestamps and no further rescaling is needed.
///
/// \li \c "trapezoidal": \c times is treated as path parameter values and
///     rescaled by \c add_time_parameterization. For each consecutive
///     pair of configurations, the time step is set to the largest joint
///     displacement divided by \c max_velocity, with a 10 ms floor.
///     This is a deliberately simple heuristic --- it does not actually
///     enforce \c max_acceleration, only avoids exceeding the joint
///     velocity bound. For high-quality time parameterization, prefer
///     letting HPP do the work and pass the result with \c "none".
///
/// In both cases the per-point velocity is set to zero only at the first
/// and last waypoint; intermediate velocities are left empty so that the
/// joint trajectory controller smooths between configurations.
///
/// \c send_trajectory returns \c True only when the action goal is accepted
/// and the controller reports a successful \c FollowJointTrajectory result.
/// It returns \c False when the action server is unavailable, the goal is
/// rejected, execution times out, or the controller reports a non-success
/// result code.
///
/// \subsection hpp_exec_time_async Async sending
///
/// \c send_trajectory blocks until the controller reports completion, which
/// is the right default for sequencing actions. When you need to send a
/// trajectory and do something else in parallel (e.g. monitor a sensor while
/// the arm is moving), \c send_trajectory_async returns immediately with the
/// \c (future, node) pair from the underlying \c FollowJointTrajectory action
/// client. The caller is responsible for spinning the node, reading the
/// future, and shutting things down. This is a convenience wrapper around
/// \c rclpy.action.ActionClient.send_goal_async; if you need finer control,
/// build the action client yourself.
///
/// \subsection hpp_exec_time_controller_topic The controller_topic argument
///
/// All sending functions default to
/// \c "/joint_trajectory_controller/follow_joint_trajectory", which is the
/// topic exposed by a \c ros2_control \c JointTrajectoryController spawned
/// under its default name. If your controller is registered under a different
/// name (e.g. \c gripper_controller, or a per-arm \c left_arm_controller in a
/// dual-arm setup), pass the corresponding topic explicitly. Use
/// \c "ros2 control list_controllers" to discover the active controllers and
/// their action topics.
///
/// \section hpp_exec_segments Segments and pre/post actions
///
/// A \c FollowJointTrajectory action is a one-shot command: once the goal
/// is sent, the controller executes it from beginning to end without any
/// way of pausing in the middle. Manipulation tasks, however, need to do
/// things between trajectory pieces --- most commonly close or open a
/// gripper at a grasp or release point. \c hpp-exec handles this by
/// splitting the trajectory into **segments** and attaching **actions** to
/// segment boundaries.
///
/// A \c Segment is a slice <tt>[start_index, end_index)</tt> of the
/// \c configs / \c times lists, with two optional lists of callables:
///
/// \li \c pre_actions are run **before** the segment's trajectory is sent.
/// \li \c post_actions are run **after** the segment's trajectory has
///     finished executing on the controller.
///
/// An action is any zero-argument callable returning a \c bool: bound
/// methods (e.g.\ <tt>gripper.close</tt>), free functions, or lambdas all
/// work. The action returns \c True on success and \c False on failure;
/// any \c False short-circuits \c execute_segments, which then aborts
/// the rest of the plan.
///
/// Concretely, \c execute_segments iterates over the segments and for
/// each one runs all \c pre_actions, sends the corresponding sub-
/// trajectory and waits for its completion, then runs all \c post_actions.
/// Segment timestamps are normalized so that each sub-trajectory starts
/// at <tt>t = 0</tt>, which is what \c FollowJointTrajectory expects.
/// A segment containing fewer than two configurations is treated as a
/// pure action point: the trajectory is skipped and only the actions run.
///
/// \c execute_segments accepts the same \c time_parameterization,
/// \c max_velocity and \c max_acceleration arguments as \c send_trajectory
/// and forwards them to every sub-trajectory it sends. Use \c "trapezoidal"
/// here if your \c times list holds path parameter values for the whole
/// path; \c hpp-exec will rescale each segment independently against the
/// joint velocity bound.
///
/// \section hpp_exec_grasps Grasp segments from a constraint graph
///
/// Building segments by hand is fine when grasp/release points are known
/// up front, but for paths produced by an HPP manipulation planner the
/// boundaries are encoded in the constraint graph. The
/// \c segments_from_graph helper queries the manipulation graph at
/// every configuration along the path, detects when the graph state
/// changes, identifies the transition crossed between the previous and new
/// states, and splits the trajectory accordingly. The two action specs
/// \c on_grasp and \c on_release (typically <tt>gripper.close</tt> and
/// <tt>gripper.open</tt>) are then attached as \c pre_actions of the
/// segments that begin with a grasp or release.
///
/// \code{.py}
/// from hpp_exec import execute_segments
/// from hpp_exec.gripper import segments_from_graph
///
/// segments = segments_from_graph(
///     configs, times, graph,
///     on_grasp=gripper.close,
///     on_release=gripper.open,
/// )
/// execute_segments(segments, configs, times, joint_names=[...])
/// \endcode
///
/// Grasp identity comes from the graph transition name, e.g.
/// <tt>"fr3/gripper > box/handle | f_01"</tt> for acquiring a grasp and
/// <tt>"fr3/gripper < box/handle | 10_f"</tt> for releasing it. This is
/// important when graph states encode combinations such as <tt>g1</tt> and
/// <tt>g1+g2</tt>: the destination state describes what is true after the
/// transition, but the transition itself describes which grasp event just
/// happened.
///
/// The fallback path triggers when the graph object does not expose
/// \c getTransitions() and \c getNodesConnectedByTransition(edge), or when
/// no edge between the previous and new state can be uniquely identified
/// (multiple edges connecting the same state pair). In those cases
/// \c transition_name is left as \c None and \c acquired / \c released are
/// derived by diffing the active-grasp sets parsed from the state names -
/// the legacy behavior, accurate when each state name lists its grasps but
/// ambiguous for combined states.
///
/// The lower-level \c extract_grasp_transitions function returns the raw
/// transition objects without building segments, which is useful for
/// diagnostics or for plugging in custom action logic. Each transition
/// carries:
///
/// \li \c config_index, \c time: where in the path the transition occurs.
/// \li \c state_before, \c state_after: graph state names returned by
///     \c graph.getStateFromConfiguration before and after the transition.
/// \li \c transition_name: graph edge name when uniquely identifiable, else
///     \c None.
/// \li \c grasps_before, \c grasps_after: active-grasp sets parsed from the
///     \c state_before / \c state_after names. These reflect the legacy
///     state-name parsing and are kept for diagnostics; they may not match
///     \c acquired / \c released in pathological state-name schemes.
/// \li \c acquired, \c released: the grasp events the segmentation logic
///     attributes to this transition. Computed from \c transition_name when
///     possible, otherwise from <tt>grasps_after - grasps_before</tt> and
///     <tt>grasps_before - grasps_after</tt>.
///
/// \subsection hpp_exec_grasps_initial Synchronizing the initial state
///
/// \c segments_from_graph attaches actions only at segment boundaries,
/// so the first segment has no pre-actions. If the real-world initial
/// state of the gripper is uncertain (a previous run that aborted mid-
/// trajectory, a simulator that spawned the fingers at an arbitrary
/// pose), the arm will start moving with the gripper in whatever state
/// it happens to be, which can collide with the object the planner
/// assumed was free to reach. The fix is to prepend the appropriate
/// action to the first segment's \c pre_actions:
///
/// \code{.py}
/// segments[0].pre_actions.insert(0, gripper.open)
/// \endcode
///
/// Because \c send_trajectory blocks until its action reports
/// completion, the arm motion will only start once the gripper has
/// reached the requested state.
///
/// \subsection hpp_exec_grasps_custom_actions Writing gripper actions
///
/// \c hpp-exec deliberately does not own the gripper implementation. Real
/// grippers, simulated grippers, detachable Gazebo objects, cameras, or
/// force sensors all have different ROS interfaces, so \c hpp-exec only
/// requires a small contract: a resolved action is a zero-argument callable
/// that returns \c True on success and \c False on failure.
///
/// For a Gazebo gripper driven by a \c JointTrajectoryController, the action
/// can call \c send_trajectory with the finger joint names and a short
/// two-point trajectory:
///
/// \code{.py}
/// def open_gripper():
///     return send_trajectory(
///         [np.array([0.0]), np.array([0.035])],
///         [0.0, 0.5],
///         joint_names=["fr3_finger_joint1"],
///         controller_topic="/gripper_controller/follow_joint_trajectory",
///     )
/// \endcode
///
/// For a real Franka gripper, the action can instead call the native
/// \c franka_msgs \c Move and \c Grasp actions. The segmentation code is the
/// same; only the callable changes:
///
/// \code{.py}
/// gripper = FrankaGripperController("fr3")
/// segments = segments_from_graph(
///     configs, times, graph,
///     on_grasp=gripper.close,
///     on_release=gripper.open,
/// )
/// segments[0].pre_actions.insert(0, gripper.open)
/// execute_segments(segments, configs, times, joint_names=[...])
/// \endcode
///
/// See \c examples/gripper_controllers.py for reference implementations of
/// a \c FollowJointTrajectory gripper, a \c GripperCommand gripper, and a
/// Franka-native gripper.
///
/// When several grippers or objects are present, pass a dictionary keyed by
/// transition name (or by grasp label) to run different actions for
/// different graph transitions:
///
/// \code{.py}
/// segments = segments_from_graph(
///     configs, times, graph,
///     on_grasp={
///         "left/gripper > box1/handle | f_01": left_gripper.close,
///         "right/gripper > box2/handle | 1_12": right_gripper.close,
///     },
///     on_release={
///         "left/gripper < box1/handle | 10_f": left_gripper.open,
///         "right/gripper < box2/handle | 12_1": right_gripper.open,
///     },
/// )
/// \endcode
///
/// Alternatively, pass a callable that accepts a \c GraspTransition if
/// the action needs to inspect \c transition.transition_name,
/// \c transition.acquired, or \c transition.released at runtime.
///
/// \section hpp_exec_low_level Lower-level helpers
///
/// \c configs_to_joint_trajectory is the function \c send_trajectory uses
/// internally to build the ROS message. Call it directly if you need the
/// \c trajectory_msgs.msg.JointTrajectory object (e.g. to publish to a
/// topic instead of an action, or to attach it to a custom goal). It
/// projects each HPP config down to \c joint_indices, sets per-point
/// velocities to zero only on the first and last waypoint (intermediate
/// waypoints are left empty so the controller smooths between them), and
/// passes \c times through unchanged - the caller is responsible for
/// providing real seconds.
///
/// \c add_time_parameterization is the trapezoidal heuristic invoked by
/// \c time_parameterization="trapezoidal". Call it directly if you want to
/// rescale a list of path-parameter values against a velocity bound without
/// going through \c send_trajectory.
///
/// \c extract_joint_config is a one-line slicing helper that returns
/// <tt>[hpp_config[offset + i] for i in range(n_joints)]</tt> as a list of
/// floats. \c send_trajectory and \c configs_to_joint_trajectory already
/// project configs through \c joint_indices, so most users will not need
/// this; it is exposed for callers that build messages by hand.
///
/// \section hpp_exec_troubleshooting Troubleshooting
///
/// \par "Trajectory controller not available"
/// The \c FollowJointTrajectory action server did not respond within the
/// 10 s wait. Check that the controller is loaded and active with
/// \c "ros2 control list_controllers", and that the topic name passed via
/// \c controller_topic matches one of the listed action endpoints.
///
/// \par "Trajectory goal rejected"
/// The controller accepted the connection but refused the goal. The most
/// common cause is a mismatch between \c joint_names and the joints the
/// controller manages; verify the spelling and order against the controller
/// configuration YAML.
///
/// \par execute_segments aborts on a pre-action
/// The pre-action returned \c False. Run the action standalone in a Python
/// REPL to see which step failed. For \c send_trajectory-based gripper
/// actions, the same "controller not available" / "goal rejected"
/// diagnostics apply against the gripper controller.
///
/// \par The arm starts before the gripper finishes
/// Make sure the gripper action is a synchronous \c send_trajectory call (it
/// blocks until completion) and that it returns \c True only on success. An
/// action returning \c True early - or a fire-and-forget topic publish -
/// will let the arm move before the gripper has reached its target.
///
/// \par segments_from_graph returns a single segment
/// No graph state change was detected along the path. Either HPP did not
/// cross a manipulation transition, or the path was sampled too coarsely
/// for \c graph.getStateFromConfiguration to see the change. Sample the
/// path more densely and re-check.
///
/// \section hpp_exec_code_map How the Python code is organized
///
/// The package is intentionally small:
///
/// \li \c trajectory_utils.py extracts selected joints from full HPP
///     configuration vectors and converts them to a ROS 2
///     \c JointTrajectory message.
/// \li \c ros2_sender.py creates the \c FollowJointTrajectory action client,
///     sends the trajectory, waits for the result, and implements the
///     \c Segment / \c execute_segments loop.
/// \li \c gripper.py reads HPP constraint-graph states, maps state changes
///     back to graph transition names, detects grasp transitions, and turns
///     them into executable \c Segment objects.
///
/// The important separation is that HPP remains responsible for planning and
/// time parameterization, \c hpp-exec is responsible for packaging and
/// sequencing execution, and the robot application remains responsible for
/// robot-specific actions such as opening a gripper or attaching a simulated
/// object.
