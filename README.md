# Teleop Slave Package

This ROS 2 (Humble) package enables real-time teleoperation of a **Fairino FR5 manipulator** and a **Tesollo Delto M Gripper** using a Manus VR glove as the master device. It features a continuous joint state streaming architecture and uses **NVIDIA cuRobo** for GPU-accelerated, collision-free motion generation (MPPI solver).

## Features
- **Real-Time Teleoperation Pipeline**: Receives end-effector poses from a master device (e.g., Manus VR glove) via `master_bridge_node` and maps it to target end-effector coordinates.
- **Continuous Joint Streaming**: A custom streaming mode (`ServoJ`) in the `fairino_lowlevel_controller_node` executing at 250Hz.
- **NVIDIA cuRobo MPPI Solver**: High-performance, CUDA-accelerated parallel trajectory optimization solver ensuring the manipulator avoids self-collisions and environmental collisions dynamically.
- **Embedded SDKs**: Contains self-contained, embedded C++ SDKs for both the Fairino FR5 (`libfairino.so.2.3.3`) and Tesollo Delto M gripper (`delto_hardware`, `delto_tcp_comm`). No external hardware configuration repositories are required for building.
- **Simulation/Dummy Mode**: Run the entire pipeline in an RViz simulation safely without connecting to physical hardware.

## Architecture & Nodes
- `master_bridge_node`: Receives raw VR glove tracking signals via UDP and maps them to ROS 2 target poses and joint states.
- `fairino_slave_node`: High-level controller capturing Cartesian objectives, preventing ground collisions, and publishing to cuRobo.
- `curobo_mppi_solver.py`: Receives cartesian pose objectives (`/curobo/pose_target`) and solves for a continuous smooth stream of joint targets via MPPI.
- `fairino_lowlevel_controller_node`: Connects directly to the robot hardware via the Fairino C++ SDK (`executeServoJ()`) or acts as a mocked publisher (`dummy_mode:=true`).
- `tesollo_slave_node`: Handles finger motion replication to the Tesollo Delto M gripper.

## Setup & Installation
### Prerequisites
- Ubuntu 22.04 + ROS 2 Humble
- NVIDIA GPU with CUDA Toolkit installed
- **NVIDIA cuRobo** (Must be installed in a Python Virtual Environment):
  ```bash
  sudo apt install python3.10-venv
  python3 -m venv ~/curobo_env
  source ~/curobo_env/bin/activate
  pip install torch torchvision torchaudio
  pip install curobo
  ```
- Fairino MoveIt2 config packages (`fairino5_v6_moveit2_config` for RViz visualization/robot description)

### Build
Because the SDKs for the hardware targets are native to this repository, you only need to run a standard colcon build.

```bash
cd ~/your_ros2_ws
colcon build --packages-select teleop_slave --symlink-install
source install/setup.bash
```

## Running the Simulation in RViz
You can verify the teleoperation pipeline works correctly using a simulated visualization (RViz). The simulation runs the underlying low-level controller in `dummy_mode`, meaning it bypasses the physical hardware connection attempts.

```bash
ros2 launch teleop_slave teleop_rviz.launch.py
```
This launch file will only start `rviz2` and the `robot_state_publisher` for the `fairino5_v6` robot.

### Running the Nodes Individually
For easier debugging and error checking during development, it is recommended to run each node in its own terminal. **In every new terminal, you must source the workspaces before running the node.**

Below is the step-by-step command sequence you should paste into each newly opened terminal.

**Terminal 1: RViz Visualization**
```bash
source ~/ros2_ws/install/setup.bash
source ~/teleop_slave/install/setup.bash
ros2 launch teleop_slave teleop_rviz.launch.py
```

**Terminal 2: Low-Level Controller (Simulation Mode)**
```bash
source ~/ros2_ws/install/setup.bash
source ~/teleop_slave/install/setup.bash
ros2 run teleop_slave fairino_lowlevel_controller_node --ros-args -p dummy_mode:=true
```

**Terminal 3: Master Bridge (UDP Receiver)**
```bash
source ~/ros2_ws/install/setup.bash
source ~/teleop_slave/install/setup.bash
ros2 run teleop_slave master_bridge_node
```

**Terminal 4: cuRobo MPPI Solver (Motion Generation)**
```bash
source ~/curobo_env/bin/activate
source ~/ros2_ws/install/setup.bash
source ~/teleop_slave/install/setup.bash
ros2 run teleop_slave curobo_mppi_solver.py
```

**Terminal 5: Fairino Slave Node (Arm High-Level)**
```bash
source ~/ros2_ws/install/setup.bash
source ~/teleop_slave/install/setup.bash
ros2 run teleop_slave fairino_slave_node
```

**Terminal 6: Tesollo Slave Node (Gripper High-Level)**
```bash
source ~/ros2_ws/install/setup.bash
source ~/teleop_slave/install/setup.bash
ros2 run teleop_slave tesollo_slave_node
```
