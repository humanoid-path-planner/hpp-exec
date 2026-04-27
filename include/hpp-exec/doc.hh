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
///     rescaled by \ref add_time_parameterization. For each consecutive
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
/// A \ref Segment is a slice <tt>[start_index, end_index)</tt> of the
/// \c configs / \c times lists, with two optional lists of callables:
///
/// \li \c pre_actions are run **before** the segment's trajectory is sent.
/// \li \c post_actions are run **after** the segment's trajectory has
///     finished executing on the controller.
///
/// An action is any zero-argument callable returning a \c bool: bound
/// methods (e.g.\ <tt>gripper.close</tt>), free functions, or lambdas all
/// work. The action returns \c True on success and \c False on failure;
/// any \c False short-circuits \ref execute_segments, which then aborts
/// the rest of the plan.
///
/// Concretely, \ref execute_segments iterates over the segments and for
/// each one runs all \c pre_actions, sends the corresponding sub-
/// trajectory and waits for its completion, then runs all \c post_actions.
/// Segment timestamps are normalized so that each sub-trajectory starts
/// at <tt>t = 0</tt>, which is what \c FollowJointTrajectory expects.
/// A segment containing fewer than two configurations is treated as a
/// pure action point: the trajectory is skipped and only the actions run.
///
/// \section hpp_exec_grasps Grasp segments from a constraint graph
///
/// Building segments by hand is fine when grasp/release points are known
/// up front, but for paths produced by an HPP manipulation planner the
/// boundaries are encoded in the constraint graph. The
/// \ref segments_from_graph helper queries the manipulation graph at
/// every configuration along the path, detects when the set of active
/// grasps changes, and splits the trajectory accordingly. The two
/// callables \c on_grasp and \c on_release (typically
/// <tt>gripper.close</tt> and <tt>gripper.open</tt>) are then attached as
/// \c pre_actions of the segments that begin with a grasp or release.
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
/// The lower-level \ref extract_grasp_transitions function returns the
/// raw transition objects (config index, time, acquired/released grasp
/// names) without building segments, which is useful for diagnostics or
/// for plugging in custom action logic.
