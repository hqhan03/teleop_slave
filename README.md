# Tesollo Manus Teleop Workspace

This repository is a ROS 2 workspace for teleoperating a Fairino FR5 arm and a Tesollo DG-5F hand from a MANUS glove and a VIVE tracker.

The project is split across a Windows sender and a Linux ROS 2 stack:

- the Windows side reads MANUS data and sends it over UDP
- the Linux side receives that data and turns it into robot commands
- the hand can be driven by direct joint mapping, fingertip IK, or keyvector retargeting

## Repository Layout

### `teleop_slave/`

The main Linux ROS 2 teleoperation package. This is where the FR5 arm teleop logic lives, along with the Linux-side MANUS receiver and the direct DG-5F joint mapping node.

### `external_snapshots/teleop_master/`

A snapshot backup of the Windows MANUS client source. It connects to MANUS Core, collects glove and tracker data, and streams it to Linux over UDP.

### `fingertip_ik_retargeter/`

A Python ROS 2 package that converts MANUS fingertip targets into DG-5F joint trajectories using a PyBullet IK solver.

### `keyvector_retargeter/`

A Python ROS 2 package that converts MANUS hand landmarks into DG-5F joint trajectories using a keyvector loss and a bounded least-squares solver.

### `delto_m_ros2/`

Tesollo-provided ROS 2 packages for DG grippers. In this workspace it is mainly used for the DG-5F driver, URDFs, and simulation. This directory is tracked as a submodule in the root workspace backup.

### `Tesollo DG-5F Hand/`

Vendor reference material such as manuals and original package files.

## High-Level System Flow

```text
MANUS Core on Windows
  -> teleop_master
  -> UDP packet
  -> teleop_slave/master_bridge_node
  -> ROS topics
  -> FR5 arm teleop and/or DG-5F hand teleop
```

More specifically:

1. `teleop_master` sends wrist pose, glove joints, fingertip positions, and hand landmarks to Linux.
2. `master_bridge_node` republishes that data as ROS topics such as:
   - `/manus/wrist_pose`
   - `/manus/finger_joints`
   - `/manus/fingertip_positions`
3. The FR5 arm path uses:
   - `fairino_slave_node` to map tracker motion to a safe TCP target
   - `fairino_lowlevel_controller_node` to solve robot IK and stream `ServoCart` or `ServoJ`
4. The DG-5F hand path uses either:
   - `tesollo_slave_node` for direct MANUS joint mapping
   - `fingertip_ik_retargeter` for fingertip-based IK
   - `keyvector_retargeter` for hand-landmark-based retargeting

## Typical Use Cases

### FR5 arm only

Use `teleop_slave` with:

- `master_bridge_node`
- `fairino_slave_node`
- `fairino_lowlevel_controller_node`

### DG-5F hand only, direct mapping

Use:

- `master_bridge_node`
- `tesollo_slave_node`
- the DG-5F driver from `delto_m_ros2`

### DG-5F hand only, fingertip IK

Use:

- `master_bridge_node`
- `fingertip_ik_retargeter`
- the DG-5F driver from `delto_m_ros2`

### DG-5F hand only, keyvector retargeting

Use:

- `master_bridge_node`
- `keyvector_retargeter`
- the DG-5F driver from `delto_m_ros2`

### Full arm + hand teleop

Run the FR5 path and one hand path at the same time. Do not run both hand publishers together.

## Workspace Setup

Clone with submodules, then build:

```bash
git clone --recurse-submodules https://github.com/hqhan03/tesollo_manus_teleop.git
cd tesollo_manus_teleop
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths teleop_slave delto_m_ros2 fingertip_ik_retargeter --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If you are using the keyvector path too, include it in dependency installation:

```bash
rosdep install --from-paths teleop_slave delto_m_ros2 fingertip_ik_retargeter keyvector_retargeter --ignore-src -r -y
```

If you already cloned without `--recurse-submodules`, initialize the submodules manually:

```bash
git submodule update --init --recursive
```

## GitHub Backup Scope

The goal of this repository backup is that another machine can clone `hqhan03/tesollo_manus_teleop`, initialize submodules, and run the current FR5 + DG-5F workflow from this one project tree.

Included directly in this repository:

- `teleop_slave/`
- `fingertip_ik_retargeter/`
- `keyvector_retargeter/`
- `teleop_slave_curobo/`
- `external_snapshots/teleop_master/`
- `resources/`

Included through submodules:

- `delto_m_ros2/`

Important notes for reproducibility:

- The live local Windows sender repo may also exist separately at `teleop_master/`, but the GitHub backup mirrors its current source under `external_snapshots/teleop_master/` because this environment could not authenticate a direct `git push`.
- `keyvector_retargeter` and `fingertip_ik_retargeter` now carry a local backup copy of the modified `dg5f_right.urdf`, so their retargeting path does not depend on an unpushed local edit inside `delto_m_ros2`.
- The exact diffs from local external vendor trees are backed up under `resources/external_patches/`.
- `fairino-cpp-sdk/` and `frcobot_ros2/` are not required for the documented FR5 tracker teleop + DG-5F keyvector + CSV logger flow. They remain optional local reference trees and are not part of the root repo checkout.
- `teleop_slave_curobo/` is included in this backup, but its current config still expects an external `frcobot_ros2` checkout for some Curobo/asset paths.

If you are working only on one package, you can also build it selectively with `colcon build --packages-select ...`.

## Where To Start

If you are new to the workspace, read these in order:

1. `teleop_slave/README.md`
2. `external_snapshots/teleop_master/Readme.md`
3. `fingertip_ik_retargeter/README.md`
4. `keyvector_retargeter/README.md`

That sequence mirrors the actual runtime architecture:

- Linux teleop stack first
- Windows MANUS sender second
- optional hand retarget path third

## Current Hardware Workflow

This is the current Linux-side runtime flow for FR5 arm teleop + DG-5F keyvector retargeting + CSV logging.

Before opening the Linux terminals below:

1. Start the Windows `teleop_master` sender and confirm UDP packets are being sent to the Linux host.
2. Make sure the FR5 controller at `192.168.58.2` is reachable, enabled, and in a safe state.
3. Make sure the DG-5F is reachable at `169.254.186.72`.
4. Keep the robot e-stop accessible.

Terminal 1: FR5 low-level controller

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_slave fairino_lowlevel_controller_node \
  --ros-args --params-file teleop_slave/config/fr5_tracker_teleop.yaml \
  -p robot_ip:=192.168.58.2
```

Terminal 2: FR5 tracker mapper

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

1. Press `Space` in the `fairino_slave_node` terminal to clutch the tracker to the current FR5 pose.
2. Press `Space` in the `teleop_csv_logger_node` terminal to start and stop CSV recording.
3. Do not run `tesollo_slave_node`, `fingertip_ik_node`, `manus_retarget_vendor`, and `keyvector_retargeter` together. Pick only one DG-5F command source.

## Notes

- The DG-5F driver and simulations live in `delto_m_ros2/`, but those package READMEs are kept separate.
- Vendor files in `Tesollo DG-5F Hand/` are useful as references, but the active development code in this workspace is under `teleop_slave/`, `teleop_master/`, `fingertip_ik_retargeter/`, and `keyvector_retargeter/`.
