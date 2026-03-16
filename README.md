# teleop_slave

This folder is the main Linux ROS 2 teleoperation stack.

If you only read one README first, read this one.

## What This Package Does

It connects incoming MANUS glove and tracker data to:

- the Fairino FR5 arm
- the Tesollo DG-5F hand

For the arm, the default path is now:

1. receive wrist pose from `/manus/wrist_pose`
2. convert tracker motion into a safe FR5 TCP target
3. solve IK with the embedded Fairino SDK
4. stream joint commands to the arm with `ServoJ`

## Main Nodes

- `master_bridge_node`: receives UDP packets from the Windows MANUS client
- `fairino_slave_node`: tracker calibration, clutching, safety limits, publishes `/fr5/pose_target`
- `fairino_lowlevel_controller_node`: validates controller state, runs SDK IK, sends `ServoJ`
- `fairino_state_printer_node`: prints live arm state in the terminal
- `tesollo_slave_node`: direct finger-joint mapping for the DG-5F hand

## Simple Data Flow

```text
MANUS UDP
  -> master_bridge_node
  -> /manus/wrist_pose
  -> fairino_slave_node
  -> /fr5/pose_target
  -> fairino_lowlevel_controller_node
  -> Fairino SDK IK
  -> ServoJ
  -> FR5
```

## Environment Setup

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/nrel/Desktop/tesollo_manus_teleop
rosdep install --from-paths teleop_slave delto_m_ros2 fingertip_ik_retargeter --ignore-src -r -y
colcon build --packages-select teleop_slave --symlink-install
source install/setup.bash
```

## Important Config Files

- `config/fr5_tracker_teleop.yaml`: default FR5 teleop parameters
- `config/tesollo_params.yaml`: direct DG-5F finger mapping parameters
- `config/fr5.yml`: older FR5 config kept for legacy or experimental paths

Optional runtime override:

- `~/.ros/fr5_tracker_calibration.yaml`: tracker axes, signs, scales, and basis override

## Methodology

### Arm Method

The arm path is based on tracker motion to TCP motion, not direct joint control.

`fairino_slave_node`:

- waits for the current robot TCP pose
- lets the operator press `Space` to clutch the tracker to the robot
- applies axis remapping and scaling
- applies phased orientation handling:
  - `position_only`
  - `yaw_only`
  - `full_6dof`
- clamps the result inside a safe workspace

`fairino_lowlevel_controller_node`:

- checks the active TCP and workobject on the Fairino controller
- converts the ROS pose target into Fairino `DescPose`
- calls `GetInverseKinRef(type=0)`
- sends the solved joints with `ServoJ`

### Hand Method

You have two choices:

- direct MANUS joint mapping with `tesollo_slave_node`
- fingertip IK with the `fingertip_ik_retargeter` package

## Basic Run Commands

### Dummy arm test

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run teleop_slave fairino_lowlevel_controller_node \
  --ros-args --params-file teleop_slave/config/fr5_tracker_teleop.yaml -p dummy_mode:=true
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run teleop_slave master_bridge_node
```

Terminal 3:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run teleop_slave fairino_slave_node \
  --ros-args --params-file teleop_slave/config/fr5_tracker_teleop.yaml
```

### Real arm test

Before starting ROS nodes:

1. Make sure the Linux host is on `192.168.58.80`.
2. Open `http://192.168.58.2` in a browser.
3. Verify the Fairino controller is reachable.
4. Verify robot enable, automatic mode, alarm state, active TCP, and active workobject.

Then start:

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run teleop_slave fairino_lowlevel_controller_node \
  --ros-args --params-file teleop_slave/config/fr5_tracker_teleop.yaml -p robot_ip:=192.168.58.2
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run teleop_slave master_bridge_node
```

Terminal 3:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run teleop_slave fairino_slave_node \
  --ros-args --params-file teleop_slave/config/fr5_tracker_teleop.yaml
```

Press `Space` in `fairino_slave_node` to clutch the tracker to the current robot pose and begin streaming.

## Safety Notes

- Start with `orientation_mode: position_only`
- Move the tracker slowly at first
- Keep the robot e-stop accessible
- Stop immediately if the robot moves in the wrong direction

## Useful Topics

- `/manus/wrist_pose`
- `/fr5/pose_target`
- `/robot_joint_states`
- `/robot_pose`

## Useful Tests

Build:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select teleop_slave --symlink-install
```

Run the C++ helper tests:

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select teleop_slave --ctest-args -R test_fr5_teleop_utils
```
