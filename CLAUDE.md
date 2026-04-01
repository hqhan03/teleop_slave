# CLAUDE.md

## Project Overview

Real-time teleoperation system: **Manus VR glove** -> **Fairino FR5 arm** (6 DOF) + **Tesollo DG-5F gripper** (20 DOF). Built on **ROS 2 Jazzy**.

## Building & Testing

```bash
conda deactivate && source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon build --symlink-install
colcon build --packages-select teleop_slave --symlink-install       # single package
colcon build --symlink-install                                       # all packages
colcon test --packages-select teleop_slave --event-handlers console_direct+  # run tests
```

## Architecture

```
Manus VR Glove (UDP 50Hz, port 12345)
    |
[master_bridge_node]
    |-- /manus/wrist_pose (PoseStamped)
    |-- /manus/finger_joints (JointState, 20 DOF)
    +-- /manus/fingertip_positions (PoseArray, 5 fingertips)

ARM:  /manus/wrist_pose -> [fairino_slave_node] -> /fr5/pose_target
      -> [fairino_lowlevel_controller_node] (SDK IK + 250Hz ServoJ) -> FR5

GRIPPER (option A): /manus/finger_joints -> [tesollo_slave_node] (gain+offset) -> DG-5F
GRIPPER (option B): /manus/fingertip_positions -> [fingertip_ik_node] (PyBullet IK) -> DG-5F
```

Only run ONE gripper controller at a time. Both publish to `/dg5f_right/dg5f_right_controller/joint_trajectory`.

## Packages

| Package | Purpose |
|---------|---------|
| `teleop_slave/` | Main C++ nodes + Python calibration scripts |
| `fingertip_ik_retargeter/` | PyBullet IK fingertip retargeting (Python) |
| `delto_m_ros2/` | Tesollo gripper drivers (submodule) |
| `teleop_master/` | Manus VR glove driver (submodule) |

## Running the System

**Dummy mode (no hardware):**
```bash
ros2 launch teleop_slave teleop_rviz.launch.py                          # RViz
ros2 run teleop_slave fairino_lowlevel_controller_node --ros-args -p dummy_mode:=true
ros2 run teleop_slave master_bridge_node
ros2 run teleop_slave tesollo_slave_node --ros-args -p dummy_mode:=true --params-file teleop_slave/config/tesollo_params.yaml
```

**Hardware mode:**
```bash
ros2 run teleop_slave fairino_lowlevel_controller_node --ros-args --params-file teleop_slave/config/fr5_tracker_teleop.yaml -p robot_ip:=192.168.58.2
ros2 run teleop_slave fairino_slave_node --ros-args --params-file teleop_slave/config/fr5_tracker_teleop.yaml
ros2 run teleop_slave master_bridge_node
ros2 launch dg5f_driver dg5f_right_driver.launch.py delto_ip:=169.254.186.72
ros2 run teleop_slave tesollo_slave_node   # or: ros2 run fingertip_ik_retargeter fingertip_ik_node
```

**Bring-up check:** Open `http://192.168.58.2` from host `192.168.58.80`, verify robot enable, automatic mode, no alarms, correct TCP/workobject.

## Tracker Calibration

Calibrates the axis mapping and orientation basis between a Vive tracker and the FR5 end-effector. Mount tracker rigidly on EE, then:

```bash
# Terminal 1: Controller     # Terminal 2: Bridge          # Terminal 3: Calibration
ros2 run teleop_slave \      ros2 run teleop_slave \       ros2 run teleop_slave \
  fairino_lowlevel_controller_node \  master_bridge_node    calibrate_tracker_frame.py
  --ros-args --params-file teleop_slave/config/fr5_tracker_teleop.yaml -p robot_ip:=192.168.58.2
```

Do NOT run `fairino_slave_node` during calibration. The script jogs through 13 poses (~2-3 min), computes `tracker_to_robot_axes`, `tracker_to_robot_signs`, and `tracker_to_robot_rpy_deg`. Copy results into `fr5_tracker_teleop.yaml`.

## Gripper Calibration

**Linear mapping** (tesollo_slave_node): Call `/calibrate_neutral`, `/calibrate_neutral_capture` services. Offsets saved to `~/.ros/tesollo_offsets.yaml`.

**IK retargeting** (fingertip_ik_node):
```bash
ros2 run fingertip_ik_retargeter calibrate_ik_frame       # Stage 1: axis mapping + hand_scale
ros2 run fingertip_ik_retargeter calibrate_ik_multipose    # Stage 2: per-finger scale fitting
```

## Hardware Addresses

| Device | Address |
|--------|---------|
| Fairino FR5 | `192.168.58.2` |
| ROS host | `192.168.58.80` |
| Tesollo DG-5F | `169.254.186.72:502` |
| Manus UDP | `0.0.0.0:12345` |

## Key Config: `teleop_slave/config/fr5_tracker_teleop.yaml`

**Arm smoothing** (fairino_slave_node): `filter_cutoff_hz` (Butterworth LPF, default 6.0), `orientation_smoothing_alpha` (SLERP, default 0.4)

**Arm safety** (fairino_slave_node): `max_linear_step_m` (0.05), `max_angular_step_deg` (10.0), `workspace_radius` (0.85)

**Servo control** (fairino_lowlevel_controller_node): `servo_cmd_period_sec` (0.004 = 250Hz), `servoj_max_step_deg` (2.0), `interpolate_stream` (true, cubic Hermite)

**Unit conventions:** Fairino SDK uses mm; ROS uses meters. All quaternions normalized with w >= 0.

**Timing:** Manus 50Hz | Arm ServoJ 250Hz | Gripper 100-500Hz | IK retargeter ~50Hz
