# Gazebo Gripper Test

Test the gripper coordination code with FR3 in Gazebo simulation.

## Setup

```bash
cd ~/devel/hpp-exec
./run.sh
```

## Run

### Terminal 1 (inside container): Launch Gazebo

```bash
./hpp-exec/scripts/launch_gazebo_gripper.sh
```

Wait until you see "Gazebo + gripper ready!".

### Terminal 2: Run the test

```bash
docker exec -it hpp-exec bash
python3 ~/devel/hpp-exec/scripts/test_gripper_gazebo.py
```

## What you should see

The FR3 robot in Gazebo performs a pick-and-place:

1. Arm moves to pre-grasp pose (~3s)
2. Arm stops, gripper closes (~1s)
3. Arm moves to place pose (~4s)
4. Arm stops, gripper opens (~1s)
5. Arm retreats to home (~3s)

## Troubleshooting

If the test fails with "Trajectory controller not available", check controllers:

```bash
ros2 control list_controllers
ros2 action list
```

You should see both `joint_trajectory_controller` and `gripper_controller` active.
If not, spawn them manually:

```bash
SCRIPT_DIR=$HOME/devel/hpp-exec/scripts
ros2 param set /controller_manager joint_trajectory_controller.type "joint_trajectory_controller/JointTrajectoryController"
ros2 run controller_manager spawner joint_trajectory_controller --param-file "$SCRIPT_DIR/controllers_gripper.yaml"
ros2 param set /controller_manager gripper_controller.type "joint_trajectory_controller/JointTrajectoryController"
ros2 run controller_manager spawner gripper_controller --param-file "$SCRIPT_DIR/controllers_gripper.yaml"
```
