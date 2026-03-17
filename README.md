# Tesollo Manus Teleop Workspace

This repository is a ROS 2 workspace for teleoperating a Fairino FR5 arm and a Tesollo DG-5F hand from a MANUS glove and a VIVE tracker.

The project is split across a Windows sender and a Linux ROS 2 stack:

- the Windows side reads MANUS data and sends it over UDP
- the Linux side receives that data and turns it into robot commands
- the hand can be driven either by direct joint mapping or by fingertip IK

## Repository Layout

### `teleop_slave/`

The main Linux ROS 2 teleoperation package. This is where the FR5 arm teleop logic lives, along with the Linux-side MANUS receiver and the direct DG-5F joint mapping node.

### `teleop_master/`

The Windows MANUS client. It connects to MANUS Core, collects glove and tracker data, and streams it to Linux over UDP.

### `fingertip_ik_retargeter/`

A Python ROS 2 package that converts MANUS fingertip targets into DG-5F joint trajectories using a PyBullet IK solver.

### `delto_m_ros2/`

Tesollo-provided ROS 2 packages for DG grippers. In this workspace it is mainly used for the DG-5F driver, URDFs, and simulation.

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

1. `teleop_master` sends wrist pose, glove joints, and fingertip positions to Linux.
2. `master_bridge_node` republishes that data as ROS topics such as:
   - `/manus/wrist_pose`
   - `/manus/finger_joints`
   - `/manus/fingertip_positions`
3. The FR5 arm path uses:
   - `fairino_slave_node` to map tracker motion to a safe TCP target
   - `fairino_lowlevel_controller_node` to solve robot IK and stream `ServoJ`
4. The DG-5F hand path uses either:
   - `tesollo_slave_node` for direct MANUS joint mapping
   - `fingertip_ik_retargeter` for fingertip-based IK

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

### Full arm + hand teleop

Run the FR5 path and one hand path at the same time. Do not run both hand publishers together.

## Workspace Setup

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/nrel/Desktop/tesollo_manus_teleop
rosdep install --from-paths teleop_slave delto_m_ros2 fingertip_ik_retargeter --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If you are working only on one package, you can also build it selectively with `colcon build --packages-select ...`.

## Where To Start

If you are new to the workspace, read these in order:

1. `teleop_slave/README.md`
2. `teleop_master/README.md`
3. `fingertip_ik_retargeter/README.md`

That sequence mirrors the actual runtime architecture:

- Linux teleop stack first
- Windows MANUS sender second
- optional fingertip IK path third

## Notes

- The DG-5F driver and simulations live in `delto_m_ros2/`, but those package READMEs are kept separate.
- Vendor files in `Tesollo DG-5F Hand/` are useful as references, but the active development code in this workspace is under `teleop_slave/`, `teleop_master/`, and `fingertip_ik_retargeter/`.
