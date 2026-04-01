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
- `fairino_lowlevel_controller_node` validates controller state, solves IK with the Fairino SDK, and streams `ServoCart` or `ServoJ`

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
  - `/manus/hand_landmarks`

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
- sends `ServoCart` or `ServoJ` commands, depending on `stream_command_mode`

### `tesollo_slave_node`

- subscribes to `/manus/finger_joints`
- maps MANUS joint values to DG-5F joint targets
- supports single-pose and multi-pose calibration
- publishes to the DG-5F trajectory controller topic

### `teleop_csv_logger_node`

- subscribes to synchronized Tesollo joint states and the FR5 `/robot_pose`
- records only synchronized Tesollo/Fairino message pairs
- writes CSV logs at the configured logging rate
- toggles recording with `Space`

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
  -> ServoCart or ServoJ
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

Main FR5 teleop parameter file. This is the single config passed with `--params-file`. It includes:

- `orientation_mode`
- tracker-to-robot axis mapping
- tracker-to-robot orientation basis
- workspace limits
- global FR5 controller speed
- IK safety settings
- controller connection and ServoCart/ServoJ settings

`fairino_lowlevel_controller_node` now reads `global_speed_percent` from this YAML and applies it with `SetSpeed(...)` during startup. The current YAML default is `30`.

The tracker orientation basis now lives directly in this file. The current candidate basis is:

```yaml
tracker_to_robot_rpy_deg: [-90.0, 180.0, 0.0]
```

That candidate is meant for the observed symptom where tracker roll drove robot yaw, tracker yaw drove robot roll, and tracker pitch was inverted. Edit `tracker_to_robot_rpy_deg` in this file directly when tuning on hardware.

### `config/tesollo_params.yaml`

Main parameter file for direct DG-5F hand mapping. This includes:

- MANUS index to Tesollo joint mapping
- per-joint gains
- neutral pose targets
- motor min and max limits

Suggested manual tuning loop:

1. Put the FR5 in a safe open pose and keep tracker motions small.
2. Clutch with the tracker held in the orientation you want to mean tool-neutral.
3. Test pure tracker roll, then pitch, then yaw with 10-15 degree motions.
4. For each test, note which robot tool axis moved and whether the sign matched.
5. Edit only `tracker_to_robot_rpy_deg`, restart `fairino_slave_node`, and repeat.
6. Once single-axis motion is correct, validate mixed motions and re-clutch from a different starting pose.

At startup, `fairino_slave_node` now logs:

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

## Current NREL Hardware Workflow

This is the current full Linux runtime flow for the setup you are actively using: FR5 arm teleop, DG-5F keyvector retargeting, and CSV logging.

Before starting the Linux terminals below:

1. Start `teleop_master` on Windows and confirm it is sending UDP packets to this Linux host.
2. Verify the FR5 controller is reachable at `192.168.58.2`.
3. Verify the DG-5F is reachable at `169.254.186.72`.
4. Keep the robot e-stop accessible.

Terminal 1: FR5 low-level controller

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_slave fairino_lowlevel_controller_node \
  --ros-args --params-file teleop_slave/config/fr5_tracker_teleop.yaml \
  -p robot_ip:=192.168.58.2
```

Terminal 2: FR5 tracker teleop mapper

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_slave fairino_slave_node \
  --ros-args --params-file teleop_slave/config/fr5_tracker_teleop.yaml
```

Terminal 3: MANUS UDP bridge

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_slave master_bridge_node
```

Terminal 4: DG-5F right-hand driver

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch dg5f_driver dg5f_right_driver.launch.py delto_ip:=169.254.186.72
```

Terminal 5: DG-5F keyvector retargeter

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch keyvector_retargeter keyvector_retarget.launch.py
```

Terminal 6: Teleop CSV logger

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_slave teleop_csv_logger_node
```

After startup:

1. Press `Space` in the `fairino_slave_node` terminal to clutch the tracker to the current robot pose.
2. Press `Space` in the `teleop_csv_logger_node` terminal to start and stop CSV recording.
3. Do not run `tesollo_slave_node`, `manus_retarget_vendor`, `fingertip_ik_retargeter`, and `keyvector_retargeter` together. Only one DG-5F command source should be active.

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
- keyvector retargeting with `keyvector_retargeter`

Do not run more than one at the same time, because they publish competing DG-5F trajectory commands.

## Useful Topics

- `/manus/wrist_pose`
- `/manus/finger_joints`
- `/manus/fingertip_positions`
- `/manus/hand_landmarks`
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
