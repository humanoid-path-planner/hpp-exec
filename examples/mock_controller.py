#!/usr/bin/env python3
"""
Mock trajectory controller with RViz2 visualization.

Accepts FollowJointTrajectory goals and publishes /joint_states
so you can visualize trajectory execution in RViz2.

Usage:
    # Terminal 1: Start mock controller
    python3 examples/mock_controller.py --urdf robots/ur5/ur5.urdf

    # Terminal 2: Start RViz2
    rviz2

    # Terminal 3: Send trajectory
    python3 -c "
    from hpp_planner import send_trajectory
    import numpy as np

    waypoints = [np.array([0,0,0,0,0,0]), np.array([1,0.5,-0.5,1,-1,0.5])]
    times = [0.0, 3.0]

    send_trajectory(waypoints, times,
        joint_names=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
                     'wrist_1_joint','wrist_2_joint','wrist_3_joint'],
        max_velocity=0.5)
    "

RViz2 setup:
    - Add RobotModel display
    - Set Fixed Frame to "base_link" or "world"
    - Robot should appear and move when trajectory is sent
"""

import argparse
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class MockController(Node):
    def __init__(self, joint_names: list, urdf_content: str = None):
        super().__init__("mock_controller")

        self.joint_names = joint_names
        self.current_positions = [0.0] * len(joint_names)

        # Action server for trajectory execution
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
            self.execute_callback,
        )

        # Publisher for joint states (RViz2 visualization)
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)

        # Publish robot description if URDF provided
        if urdf_content:
            qos = rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
            )
            self.robot_desc_pub = self.create_publisher(String, "/robot_description", qos)
            msg = String()
            msg.data = urdf_content
            self.robot_desc_pub.publish(msg)
            self.get_logger().info("Published /robot_description")

        # Publish joint states at 50Hz
        self.timer = self.create_timer(0.02, self.publish_joint_state)

        self.get_logger().info("=" * 60)
        self.get_logger().info("Mock Controller Ready")
        self.get_logger().info(f"  Joints: {joint_names}")
        self.get_logger().info("  Publishing: /joint_states @ 50Hz")
        self.get_logger().info("  Action: /joint_trajectory_controller/follow_joint_trajectory")
        self.get_logger().info("")
        self.get_logger().info("  Open RViz2 and add RobotModel display")
        self.get_logger().info("=" * 60)

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        self.joint_state_pub.publish(msg)

    def execute_callback(self, goal_handle):
        traj = goal_handle.request.trajectory

        self.get_logger().info("")
        self.get_logger().info("=" * 50)
        self.get_logger().info("EXECUTING TRAJECTORY")
        self.get_logger().info(f"  Joints: {list(traj.joint_names)}")
        self.get_logger().info(f"  Points: {len(traj.points)}")

        if not traj.points:
            goal_handle.succeed()
            return FollowJointTrajectory.Result()

        # Map trajectory joint names to our joint indices
        joint_map = {}
        for i, name in enumerate(traj.joint_names):
            if name in self.joint_names:
                joint_map[i] = self.joint_names.index(name)

        first = traj.points[0]
        last = traj.points[-1]
        self.get_logger().info(f"  Start: {[f'{p:.2f}' for p in first.positions]}")
        self.get_logger().info(f"  Goal:  {[f'{p:.2f}' for p in last.positions]}")

        last_time = last.time_from_start
        duration = last_time.sec + last_time.nanosec * 1e-9
        self.get_logger().info(f"  Duration: {duration:.2f}s")

        # Execute trajectory in real-time
        start_time = time.time()
        point_idx = 0

        while point_idx < len(traj.points):
            elapsed = time.time() - start_time
            point = traj.points[point_idx]
            point_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9

            if elapsed >= point_time:
                # Update positions
                for traj_idx, our_idx in joint_map.items():
                    self.current_positions[our_idx] = point.positions[traj_idx]
                point_idx += 1

            time.sleep(0.005)

        self.get_logger().info("  COMPLETE")
        self.get_logger().info("=" * 50)

        goal_handle.succeed()
        return FollowJointTrajectory.Result()


def main():
    parser = argparse.ArgumentParser(description="Mock trajectory controller with RViz2 support")
    parser.add_argument("--urdf", type=str, help="URDF file (enables robot visualization)")
    parser.add_argument("--joints", type=str, nargs="+",
                        default=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                 "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
                        help="Joint names (default: UR5 joints)")
    args = parser.parse_args()

    # Read URDF if provided
    urdf_content = None
    if args.urdf:
        urdf_path = args.urdf
        if not os.path.isabs(urdf_path):
            urdf_path = os.path.join(os.path.dirname(__file__), "..", urdf_path)

        if os.path.exists(urdf_path):
            with open(urdf_path) as f:
                urdf_content = f.read()
            print(f"Loaded URDF: {urdf_path}")
        else:
            print(f"Warning: URDF not found: {urdf_path}")

    rclpy.init()
    node = MockController(args.joints, urdf_content)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
