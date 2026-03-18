# teleop_slave

`teleop_slave` is the main Linux ROS 2 teleoperation package in this workspace.

It is responsible for three jobs:

1. receiving the UDP stream from the Windows MANUS sender
2. converting tracker motion into FR5 arm commands
3. optionally converting MANUS glove joints into DG-5F hand commands

If you only read one package README on the Linux side, read this one.

## What This Package Provides

### MANUS UDP bridge

`master_bridge_node` receives UDP packets from `teleop_master` and republishes them as ROS topics.

### FR5 arm teleoperation

The FR5 path is split into two nodes:

- `fairino_slave_node` maps tracker motion into a target end-effector pose
- `fairino_lowlevel_controller_node` validates controller state, solves IK with the Fairino SDK, and streams `ServoJ`

### DG-5F direct joint mapping

`tesollo_slave_node` maps MANUS finger joints directly to DG-5F motor targets and publishes a `JointTrajectory` for the hand driver.

If you want fingertip-based hand teleop instead, use `fingertip_ik_retargeter` instead of `tesollo_slave_node`.

## Main Nodes

### `master_bridge_node`

- receives one UDP packet from the Windows MANUS client
- republishes:
  - `/manus/wrist_pose`
  - `/manus/finger_joints`
  - `/manus/fingertip_positions`

### `fairino_slave_node`

- subscribes to `/manus/wrist_pose`
- waits for the current robot pose from `/robot_pose`
- lets the operator press `Space` to clutch the tracker to the current robot pose
- applies axis mapping, orientation mapping, scaling, and workspace limits
- publishes `/fr5/pose_target`

### `fairino_lowlevel_controller_node`

- subscribes to `/fr5/pose_target`
- queries the Fairino controller state
- converts the ROS target pose into the Fairino `DescPose` format
- calls the built-in Fairino inverse kinematics
- sends `ServoJ` joint commands

### `fairino_state_printer_node`

- prints the live robot state to the terminal
- useful when debugging robot pose, joints, or streaming behavior

### `tesollo_slave_node`

- subscribes to `/manus/finger_joints`
- maps MANUS joint values to DG-5F joint targets
- supports single-pose and multi-pose calibration
- publishes to the DG-5F trajectory controller topic

## Data Flow

### FR5 arm path

```text
teleop_master
  -> UDP
  -> master_bridge_node
  -> /manus/wrist_pose
  -> fairino_slave_node
  -> /fr5/pose_target
  -> fairino_lowlevel_controller_node
  -> Fairino SDK IK
  -> ServoJ
  -> FR5
```

### DG-5F direct hand path

```text
teleop_master
  -> UDP
  -> master_bridge_node
  -> /manus/finger_joints
  -> tesollo_slave_node
  -> /dg5f_right/dg5f_right_controller/joint_trajectory
  -> dg5f_driver
  -> DG-5F
```

## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/nrel/Desktop/tesollo_manus_teleop
colcon build --packages-select teleop_slave --symlink-install
source install/setup.bash
```

## Important Configuration Files

### `config/fr5_tracker_teleop.yaml`

Main FR5 teleop parameters. This is the base config passed with `--params-file`. It includes:

- `orientation_mode`
- tracker-to-robot axis mapping
- tracker-to-robot orientation basis
- workspace limits
- IK safety settings

The repo default tracker orientation basis is still identity:

```yaml
tracker_to_robot_rpy_deg: [0.0, 0.0, 0.0]
```

That basis keeps tracker roll, pitch, and yaw aligned with the robot end-effector roll, pitch, and yaw axes if the tracker frame already matches the tool frame.

For hardware calibration, use the runtime override template in `config/fr5_tracker_calibration.yaml`. That file is not loaded automatically unless the `calibration_file` parameter points to it or you copy it to the default override path. It starts from this candidate basis:

```yaml
tracker_to_robot_rpy_deg: [0.0, -90.0, 180.0]
```

That candidate is meant for the observed symptom where tracker roll drove robot yaw, tracker yaw drove robot roll, and tracker pitch was inverted.

### `config/tesollo_params.yaml`

Main parameter file for direct DG-5F hand mapping. This includes:

- MANUS index to Tesollo joint mapping
- per-joint gains
- neutral pose targets
- motor min and max limits

### `~/.ros/fr5_tracker_calibration.yaml`

Optional runtime override file for FR5 tracker calibration. If present, it can override:

- axis mapping
- axis signs
- scaling
- orientation basis
- orientation mode

Runtime precedence is:

1. Load base FR5 parameters from `config/fr5_tracker_teleop.yaml` passed via `--params-file`
2. Read the `calibration_file` parameter from that base config
3. If the override file exists, replace overlapping calibration values with the override contents

That is why there are two FR5 YAML files:

- `config/fr5_tracker_teleop.yaml` is the main runtime config
- `config/fr5_tracker_calibration.yaml` is a calibration template for the optional override layer

You can create it by copying `config/fr5_tracker_calibration.yaml`, or point the node at the repo copy directly:

```bash
ros2 run teleop_slave fairino_slave_node \
  --ros-args --params-file teleop_slave/config/fr5_tracker_teleop.yaml \
  -p calibration_file:=/home/nrel/Desktop/tesollo_manus_teleop/teleop_slave/config/fr5_tracker_calibration.yaml
```

Suggested manual tuning loop:

1. Put the FR5 in a safe open pose and keep tracker motions small.
2. Clutch with the tracker held in the orientation you want to mean tool-neutral.
3. Test pure tracker roll, then pitch, then yaw with 10-15 degree motions.
4. For each test, note which robot tool axis moved and whether the sign matched.
5. Edit only `tracker_to_robot_rpy_deg`, restart `fairino_slave_node`, and repeat.
6. Once single-axis motion is correct, validate mixed motions and re-clutch from a different starting pose.

At startup, `fairino_slave_node` now logs:

- whether the calibration override file was found
- which source is active for calibration values
- the raw loaded `tracker_to_robot_rpy_deg`
- the effective stored quaternion as `basis_quat_xyzw`

This avoids confusion from converting the quaternion back into a different Euler-angle representation.

## Running The FR5 Arm

### Real hardware run

Before starting ROS nodes:

1. Make sure the Linux host can reach the FR5 controller.
2. Open the controller web UI and confirm the robot is reachable.
3. Verify the robot is enabled and ready.
4. Verify the correct TCP and workobject are active if your workflow depends on them.
5. Keep the e-stop accessible.

Then start:

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_slave fairino_lowlevel_controller_node \
  --ros-args --params-file teleop_slave/config/fr5_tracker_teleop.yaml -p robot_ip:=192.168.58.2
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_slave master_bridge_node
```

Terminal 3:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_slave fairino_slave_node \
  --ros-args --params-file teleop_slave/config/fr5_tracker_teleop.yaml
```

After all three are running, press `Space` in the `fairino_slave_node` terminal. That clutches the tracker to the current robot pose and starts pose streaming.

### Dummy FR5 run

Use this when you want to test the ROS teleop path without commanding the real arm.

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_slave fairino_lowlevel_controller_node \
  --ros-args --params-file teleop_slave/config/fr5_tracker_teleop.yaml -p dummy_mode:=true
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_slave master_bridge_node
```

Terminal 3:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_slave fairino_slave_node \
  --ros-args --params-file teleop_slave/config/fr5_tracker_teleop.yaml
```

## Running The DG-5F Hand With Direct Mapping

This path uses MANUS joint angles directly. It does not use fingertip IK.

### Real hardware

Start the DG-5F driver first in a separate terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch dg5f_driver dg5f_right_driver.launch.py delto_ip:=169.254.186.72 delto_port:=502
```

Then start the teleop side:

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_slave master_bridge_node
```

Terminal 3:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_slave tesollo_slave_node \
  --ros-args --params-file teleop_slave/config/tesollo_params.yaml
```

### Real hardware with the vendor retarget mapping

This path uses the Tesollo vendor retarget logic adapted to the current workspace.
It subscribes to `/manus/finger_joints` and publishes a `JointTrajectory` to the
DG-5F controller.

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch dg5f_driver dg5f_right_driver.launch.py delto_ip:=169.254.186.72 delto_port:=502
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_slave master_bridge_node
```

Terminal 3:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_slave manus_retarget_vendor
```

If you are driving the left hand instead, set:

```bash
ros2 run teleop_slave manus_retarget_vendor --ros-args -p hand_side:=left
```

### Dummy mode

If you want to test the mapping without the real DG-5F driver:

```bash
ros2 run teleop_slave tesollo_slave_node \
  --ros-args --params-file teleop_slave/config/tesollo_params.yaml -p dummy_mode:=true
```

In dummy mode the node publishes to `/joint_trajectory_controller/joint_trajectory` instead of the real DG-5F controller topic.

For the vendor retarget node, use:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_slave manus_retarget_vendor --ros-args -p dummy_mode:=true
```

## DG-5F Direct-Mapping Calibration

`tesollo_slave_node` supports both offset-only and multi-pose calibration.

### Offset-only neutral calibration

1. Move the hand to the neutral reference pose.
2. Call:

```bash
ros2 service call /calibrate_neutral std_srvs/srv/Trigger "{}"
```

3. Match your real hand to the requested neutral pose.
4. Capture:

```bash
ros2 service call /calibrate_neutral_capture std_srvs/srv/Trigger "{}"
```

### Multi-pose calibration

Start:

```bash
ros2 service call /calibrate_multipose_start std_srvs/srv/Trigger "{}"
```

Follow the prompts printed by the node as it steps through the calibration poses.

### Reset calibration

```bash
ros2 service call /calibrate_reset std_srvs/srv/Trigger "{}"
ros2 service call /calibrate_multipose_reset std_srvs/srv/Trigger "{}"
```

## Hand Mode Choice

You have two hand teleop options in this workspace:

- direct MANUS joint mapping with `tesollo_slave_node`
- fingertip IK with `fingertip_ik_retargeter`

Do not run both at the same time, because both publish DG-5F trajectory commands.

## Useful Topics

- `/manus/wrist_pose`
- `/manus/finger_joints`
- `/manus/fingertip_positions`
- `/fr5/pose_target`
- `/robot_joint_states`
- `/robot_pose`

## Useful Tests

Build:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select teleop_slave --symlink-install
```

Run the utility test target:

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select teleop_slave --ctest-args -R test_fr5_teleop_utils
```

## Safety Notes

- Start with slow, deliberate motions.
- Keep the robot e-stop accessible.
- Stop immediately if the robot moves in the wrong direction.
- Re-clutch with `Space` whenever the tracker reference feels wrong.
- For first tests, dummy mode is the safest place to validate topic flow and calibration.

## Troubleshooting

### No MANUS data on Linux

Check:

- `teleop_master` is running on Windows
- the Linux target IP in `teleop_master` is correct
- UDP port `12345` is not blocked
- `master_bridge_node` is running

### FR5 does not move

Check:

- `fairino_lowlevel_controller_node` is connected
- `fairino_slave_node` has been clutched with `Space`
- `/robot_pose` is being published
- the controller is enabled and reachable

### DG-5F does not move

Check:

- `dg5f_driver` is running
- `tesollo_slave_node` or `fingertip_ik_retargeter` is running
- only one hand publisher is active
- `master_bridge_node` is publishing the required hand topic
