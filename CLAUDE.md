# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Real-time teleoperation system for a **Fairino FR5 robot arm** (6 DOF) + **Tesollo DG-5F gripper** (20 DOF) using a **Manus VR glove** as input. Built on **ROS 2 Jazzy** with an in-workspace **Fairino SDK IK + ServoJ** arm path and optional GPU tooling for experiments.

## ROS 2 Environment & Building

```bash
# Source ROS 2 Jazzy and source the workspace overlay
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# Build a single package
colcon build --packages-select teleop_slave --symlink-install

# Build all packages
colcon build --symlink-install

# Install dependencies
rosdep install --from-paths teleop_slave delto_m_ros2 fingertip_ik_retargeter --ignore-src -r -y
```

## Running Tests

```bash
# C++ tests (fr5_teleop_utils)
colcon build --packages-select teleop_slave --symlink-install
colcon test --packages-select teleop_slave --event-handlers console_direct+

# Python tests (fingertip IK calibration)
colcon build --packages-select fingertip_ik_retargeter --symlink-install
colcon test --packages-select fingertip_ik_retargeter --event-handlers console_direct+

# Run a single C++ test directly
./build/teleop_slave/test_fr5_teleop_utils

# Run a single Python test directly
cd fingertip_ik_retargeter && python -m pytest tests/test_frame_calibration.py -v
```

## Running the System

**Simulation/Dummy mode (no hardware):**
```bash
# Terminal 1: RViz visualization
ros2 launch teleop_slave teleop_rviz.launch.py

# Terminal 2: Arm controller (dummy mode)
ros2 run teleop_slave fairino_lowlevel_controller_node --ros-args -p dummy_mode:=true

# Terminal 3: Manus bridge (UDP receiver)
ros2 run teleop_slave master_bridge_node

# Terminal 4: Gripper controller (gain+offset mapping)
ros2 run teleop_slave tesollo_slave_node --ros-args -p dummy_mode:=true --params-file teleop_slave/config/tesollo_params.yaml

# Terminal 4 (alternative): IK-based gripper retargeter
ros2 launch fingertip_ik_retargeter fingertip_ik.launch.py
```

**Hardware mode:**
```bash
ros2 run teleop_slave fairino_lowlevel_controller_node --ros-args --params-file teleop_slave/config/fr5_tracker_teleop.yaml -p robot_ip:=192.168.58.2
ros2 run teleop_slave fairino_slave_node --ros-args --params-file teleop_slave/config/fr5_tracker_teleop.yaml
ros2 launch dg5f_driver dg5f_right_driver.launch.py delto_ip:=169.254.186.72
ros2 run teleop_slave tesollo_slave_node
```

## Architecture & Data Flow

```
Manus VR Glove (UDP 50Hz, port 12345)
    |
[master_bridge_node]
    |-- /manus/wrist_pose (PoseStamped)
    |-- /manus/finger_joints (JointState, 20 joints)
    +-- /manus/fingertip_positions (PoseArray, 5 fingertips in MANUS palm frame)

ARM PATH:
/manus/wrist_pose -> [fairino_slave_node] (clutch zeroing + tracker calibration + safety clamps)
    -> /fr5/pose_target
    -> [fairino_lowlevel_controller_node] (Fairino SDK GetInverseKinRef + 250Hz ServoJ)
    -> Fairino FR5 via libfairino SDK

LEGACY ARM COMPAT:
/fr5/pose_target is also mirrored to /curobo/pose_target for backward compatibility during transition

GRIPPER PATH (option A — linear mapping):
/manus/finger_joints -> [tesollo_slave_node] (per-joint gain+offset)
    -> JointTrajectory -> [delto_hardware] -> Tesollo DG-5F (TCP:502)

GRIPPER PATH (option B — IK retargeting):
/manus/fingertip_positions -> [fingertip_ik_node] (PyBullet SDLS IK)
    -> JointTrajectory -> [delto_hardware] -> Tesollo DG-5F (TCP:502)
```

## Package Structure

| Package | Location | Purpose |
|---------|----------|---------|
| `teleop_slave` | `teleop_slave/` | Main teleoperation nodes (C++ + Python) |
| `fingertip_ik_retargeter` | `fingertip_ik_retargeter/` | PyBullet IK-based fingertip retargeting (Python) |
| `delto_m_ros2/*` | `delto_m_ros2/` | Tesollo gripper driver packages |
| `teleop_master` | `teleop_master/` | Manus VR glove driver |

## Key Source Files

**teleop_slave/src/**
- `master_bridge_node.cpp` — UDP receiver for Manus glove; publishes wrist pose, 20 finger joints, and 5 fingertip positions
- `fairino_slave_node.cpp` — Tracker clutching, frame calibration, phased orientation mapping, workspace limits; publishes `/fr5/pose_target`
- `fairino_lowlevel_controller_node.cpp` — Fairino SDK interface; validates active TCP/workobject, solves pose targets with SDK IK, streams ServoJ; dummy mode
- `tesollo_slave_node.cpp` — 20-DOF finger joint mapping with gain+offset calibration; per-joint gain via ROS params
- `fr5_teleop_utils.cpp` — Shared utilities: axis mapping, orientation computation, pose clamping, IK solving with fallback, unit conversion (meters<->mm for Fairino SDK)
- `fairino_robot_interface.cpp` — Abstract `IFairinoRobot` interface + `FairinoRobotSdkAdapter` wrapping libfairino calls; enables mock testing via `FakeFairinoRobot`
- `scripts/curobo_mppi_solver.py` — Legacy / experimental GPU motion planning path; no longer required for the default FR5 arm teleop flow

**fingertip_ik_retargeter/**
- `fingertip_ik_node.py` — Subscribes `/manus/fingertip_positions`, solves per-finger 4-DOF IK, publishes JointTrajectory
- `dg5f_ik_solver.py` — Headless PyBullet SDLS solver with warm-starting; strips URDF meshes for speed
- `frame_calibration.py` — Axis remapping, calibration I/O, hand basis computation, 5-pose reference definitions
- `calibrate_ik_frame.py` — Stage 1: open-hand capture -> axis mapping + hand_scale
- `calibrate_ik_multipose.py` — Stage 2: 5-pose capture -> per-finger scale fitting via least-squares

**Embedded SDKs:**
- `teleop_slave/libfairino/` — Fairino FR5 SDK v2.3.3
- `teleop_slave/libdelto/` — Tesollo hardware plugin + TCP comm (Boost.ASIO)

## Important Implementation Details

**UDP packet format** (master_bridge_node.hpp):
```c
#pragma pack(push, 1)
struct HandDataPacket {
    uint32_t frame;           // Frame counter
    float wristPos[3];        // XYZ meters
    float wristQuaternion[4]; // w, x, y, z
    float fingerFlexion[20];  // 20 joint angles (degrees)
    float fingertipPos[15];   // 5 fingertips x 3 (XYZ, palm-local frame)
};
#pragma pack(pop)
```
Older packets without fingertip data are handled gracefully (zeroed fields). Non-blocking socket with `O_NONBLOCK`.

**Unit conversions:** Fairino SDK uses millimeters; ROS uses meters. `PoseToDescPose()` multiplies by 1000, `DescPoseToPose()` divides. All quaternions are normalized with w >= 0.

**Gripper calibration** (tesollo_slave_node):
- Runtime mapping: `target[i] = clamp(gain[i] * manus[i] + offset[i], min, max)`
- Per-joint gains configured in `config/tesollo_params.yaml`
- Services (all `std_srvs/Trigger`):
  - `/calibrate_neutral` — Move Tesollo to neutral pose
  - `/calibrate_neutral_capture` — Capture 1-sec Manus average, compute offsets
  - `/calibrate_reset` — Reset offsets to zero
  - `/calibrate_multipose_start` — Begin 5-pose least-squares calibration
  - `/calibrate_multipose_capture` — Capture current pose (repeat for all 5 poses)
  - `/calibrate_multipose_reset` — Abort multi-pose calibration
- Offsets saved to `~/.ros/tesollo_offsets.yaml`; full calibration to `~/.ros/tesollo_calibration.yaml`

**IK retargeter parameters** (fingertip_ik_retargeter):
- `input_frame_mode` — `palm_local` (recommended) or `legacy_tracker_world`
- `calibration_file` — Optional runtime frame calibration YAML (`~/.ros/manus_dg5f_ik_frame.yaml`)
- `hand_scale` — Scale Manus fingertip workspace to DG-5F
- `workspace_axis_scale` — Axis-specific gain after the palm-frame remap; `Y` is most important for spread/abduction
- `finger_target_scales` — Per-finger scalar gains fitted by the stage-2 multi-pose calibration
- `palm_offset` — XYZ offset for palm frame alignment
- `joint_damping` — SDLS damping (higher = smoother)
- Per-finger enable flags for debugging

**Fairino arm services** (fairino_lowlevel_controller_node):
- `/enable_streaming` (SetBool) — Start/stop 250Hz ServoJ loop
- `/execute_trajectory` (Trigger) — Execute trajectory queue

**Fairino arm safety mechanisms:**
- IK orientation fallback: if position+orientation fails, retries with current orientation
- IK position backoff: steps target back toward current TCP (up to 4 increments)
- Streaming auto-stop: 5+ consecutive IK failures over 0.75s disables streaming
- ServoJ max step: 2.0 deg per command prevents servo errors
- Workspace limits: XYZ bounding box + planar radius constraint (r <= 0.85m)
- Linear step limit: 0.05m max per cycle; angular step limit: 10 deg

**FR5 tracker teleop config**:
- `teleop_slave/config/fr5_tracker_teleop.yaml` — default `/fr5/pose_target`, safety, and controller validation parameters
- `~/.ros/fr5_tracker_calibration.yaml` — optional tracker axes/signs/scales/basis overrides loaded by `fairino_slave_node`

**Tracker -> robot frame mapping** (fairino_slave_node):
- Configurable signed axis permutation (`tracker_to_robot_axes`, `tracker_to_robot_signs`)
- Per-axis translation scaling (`translation_scale_xyz`)
- Orientation basis rotation (`tracker_to_robot_rpy_deg`) applied as: B * delta_tracker * B^-1
- Orientation modes: `position_only`, `yaw_only`, `full_6dof`
- Clutch (SPACE key): captures tracker pose as zero reference; subsequent motion is delta from that point

**Joint naming:**
- Fairino FR5: `j1`-`j6`
- DG-5F: `rj_dg_1_1` through `rj_dg_5_4` (finger x joint)
- Manus indices 0-19 mapped to Tesollo motors; motor 17 unmapped (pinky CMC abduction, gain=0); motor 19 combines pinky PIP[18]+DIP[19]

**Timing:** Manus input 50Hz | Arm control 250Hz | Gripper 100-500Hz | IK retargeter ~50Hz (Python/PyBullet overhead)

## Hardware Addresses (defaults)

- Fairino FR5: `192.168.58.2`
- ROS host used during bring-up: `192.168.58.80`
- Tesollo DG-5F: `169.254.186.72:502`
- Manus UDP: `0.0.0.0:12345`

**Bring-up check:** before ROS streaming, open `http://192.168.58.2` from the host on `192.168.58.80`, log into the Fairino web UI, and verify robot enable, automatic mode, alarms, active TCP, and active workobject.

## Running the Fingertip IK Retargeter (Hardware)

```
Manus Glove (UDP:12345)
  -> [master_bridge_node] -> /manus/fingertip_positions (PoseArray)
  -> [fingertip_ik_node] (PyBullet SDLS IK)
  -> /dg5f_right/dg5f_right_controller/joint_trajectory (JointTrajectory, 20 DOF)
  -> [dg5f_right_controller] -> Tesollo DG-5F (TCP 169.254.186.72:502)
```

**Important:** Run only ONE gripper controller — either `tesollo_slave_node` (linear mapping) OR `fingertip_ik_node` (IK retargeting). They both publish to `/dg5f_right/dg5f_right_controller/joint_trajectory`.

**Topic routing:** The `dummy_mode` parameter switches the output topic:
- `dummy_mode:=true` -> `/joint_trajectory_controller/joint_trajectory` (RViz/sim)
- `dummy_mode:=false` (default) -> `/dg5f_right/dg5f_right_controller/joint_trajectory` (real hardware)

**Terminal 1 — DG-5F gripper driver:**
```bash
ros2 launch dg5f_driver dg5f_right_driver.launch.py delto_ip:=169.254.186.72
```

**Terminal 2 — Manus glove bridge:**
```bash
ros2 run teleop_slave master_bridge_node
```

**Terminal 3 — Fingertip IK retargeter (hardware mode, no dummy_mode flag):**
```bash
ros2 run fingertip_ik_retargeter fingertip_ik_node
# or: ros2 launch fingertip_ik_retargeter fingertip_ik.launch.py
```

**Calibration workflow:**
```bash
# Stage 1: palm-frame / axis / global scale calibration (open hand)
ros2 run fingertip_ik_retargeter calibrate_ik_frame

# Stage 2: 5-pose gain calibration (robot moves through fixed poses)
ros2 run fingertip_ik_retargeter calibrate_ik_multipose
```

**Terminal 4 (optional) — Monitor output:**
```bash
ros2 topic echo /dg5f_right/dg5f_right_controller/joint_trajectory --no-arr
```

**Safety:**
- Start with slow hand movements — verify each finger maps in the correct direction
- Keep the gripper e-stop accessible

**Tuning parameters:**
- `hand_scale` — decrease if DG-5F is smaller than your hand (e.g. 0.8)
- `palm_offset` — XYZ offset (meters) if fingertips are systematically shifted
- `joint_damping` — increase for smoother but slower tracking (default 0.1)
- Per-finger enable flags: `enable_thumb`, `enable_index`, `enable_middle`, `enable_ring`, `enable_pinky`

**Verify:**
```bash
ros2 topic hz /dg5f_right/dg5f_right_controller/joint_trajectory  # expect ~50Hz
ros2 topic echo /manus/fingertip_positions  # confirm Manus data flowing
```
