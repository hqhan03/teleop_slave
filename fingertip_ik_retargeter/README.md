# fingertip_ik_retargeter

`fingertip_ik_retargeter` converts MANUS fingertip targets into DG-5F joint trajectories.

Use this package when you want the DG-5F hand to follow fingertip motion rather than directly copying MANUS joint angles.

## Why This Package Exists

Direct joint mapping is simple, but it assumes the human hand and robot hand are close enough in geometry that copying joint values is useful.

That is often not the case. The DG-5F and a human hand differ in:

- link lengths
- joint limits
- joint layout
- available finger coupling

This package avoids that by focusing on fingertip targets instead of raw joint matching.

## What The Node Does

The live node:

1. subscribes to `/manus/fingertip_positions`
2. interprets the targets in the MANUS palm frame
3. remaps those targets into the DG-5F palm frame
4. solves per-finger IK in PyBullet
5. publishes a `JointTrajectory` for the DG-5F controller

## Required Inputs And Outputs

### Input topics

- `/manus/fingertip_positions`
- optionally `/manus/wrist_pose` if you are using legacy world-frame input mode

### Output topic

By default:

- `/dg5f_right/dg5f_right_controller/joint_trajectory`

In dummy mode:

- `/joint_trajectory_controller/joint_trajectory`

## Key Files

### `fingertip_ik_retargeter/fingertip_ik_node.py`

Main ROS 2 node that performs the live conversion from MANUS fingertip targets to DG-5F commands.

### `fingertip_ik_retargeter/dg5f_ik_solver.py`

PyBullet-based DG-5F solver.

### `fingertip_ik_retargeter/frame_calibration.py`

Shared utilities for frame calibration and axis remapping.

### `fingertip_ik_retargeter/calibrate_ik_frame.py`

Stage 1 frame calibration utility.

### `fingertip_ik_retargeter/calibrate_ik_multipose.py`

Stage 2 multi-pose scale calibration utility.

### `config/ik_params.yaml`

Main runtime parameter file for the retargeter.

## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/nrel/Desktop/tesollo_manus_teleop
colcon build --packages-select fingertip_ik_retargeter --symlink-install
source install/setup.bash
```

## Python Dependency

This package uses PyBullet. If PyBullet is missing, install it into the same Python environment used by ROS 2 on the Linux machine.

## Common Run Setup

For live DG-5F hardware you normally need three pieces running:

1. the DG-5F driver from `delto_m_ros2`
2. `teleop_slave/master_bridge_node`
3. this package

### Real hardware example

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
ros2 launch fingertip_ik_retargeter fingertip_ik.launch.py
```

### Dummy mode example

If you want to test the retargeting without the real hand driver, set `dummy_mode:=true` when launching the node or in the parameter file.

## Calibration

This package supports a two-stage calibration process.

### Stage 1: frame calibration

Use this to estimate the basic palm frame mapping and axis conventions.

```bash
ros2 run fingertip_ik_retargeter calibrate_ik_frame
```

### Stage 2: multi-pose gain calibration

Use this to refine scale and per-finger behavior across several poses.

```bash
ros2 run fingertip_ik_retargeter calibrate_ik_multipose
```

## Important Parameters

The main runtime parameters live in `config/ik_params.yaml`. Useful ones include:

- `dummy_mode`
- `input_frame_mode`
- `calibration_file`
- `hand_scale`
- `workspace_axis_scale`
- `finger_target_scales`
- `thumb_target_scale`
- `palm_offset`
- `joint_damping`
- `current_pose_weight`

## When To Use This Instead Of Direct Mapping

Use `fingertip_ik_retargeter` when:

- direct joint mapping feels anatomically wrong
- fingertip placement matters more than matching joint angles
- the DG-5F hand shape differs too much from the operator’s hand
- you want a more geometry-aware retargeting path

Use `tesollo_slave_node` instead when:

- you want the simplest hand teleop path
- you are tuning direct joint gains and offsets

Do not run both at the same time.

## Troubleshooting

### No fingertip data

Check:

- `master_bridge_node` is running
- the Windows MANUS sender is active
- `/manus/fingertip_positions` is non-empty

### No DG-5F motion

Check:

- `dg5f_driver` is running
- the retargeter is publishing to the expected trajectory topic
- direct mapping is not running at the same time

### Solver starts but targets feel wrong

Check:

- `input_frame_mode`
- calibration file contents
- frame calibration stage
- hand scale and thumb scale parameters
