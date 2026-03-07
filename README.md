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

## Running the Simulation in RViz (Fairino FR5)
You can verify the teleoperation pipeline works correctly for the arm using a simulated visualization (RViz). The simulation runs the underlying low-level controller in `dummy_mode`, meaning it bypasses the physical hardware connection attempts.

```bash
ros2 launch teleop_slave teleop_rviz.launch.py
```
This launch file will only start `rviz2` and the `robot_state_publisher` for the `fairino5_v6` robot.

## Running the Simulation in Gazebo (Tesollo DG-5F)
To verify the complex 20-DOF finger mappings of the Tesollo gripper, you can run the official `dg5f_gz` package from the Tesollo repository.

```bash
# Launch the right-hand Gripper in Gazebo
source ~/ros2_ws/install/setup.bash
ros2 launch dg5f_gz dg5f_right_gz.launch.py
```
When running the `tesollo_slave_node`, ensure you append `--ros-args -p dummy_mode:=true` to automatically bypass the Modbus TCP and pipe the trajectory directly to the Gazebo controllers.

### Running the Nodes Individually (6 Terminals)
For easier debugging and error checking during development, it is recommended to run each node in its own terminal.
**Important:** In every new terminal, you must source the workspaces before running the node (`ros2_ws` and `teleop_slave`).

#### Terminal 1: RViz Visualization
Starts the URDF `robot_state_publisher` and RViz so you can verify the robot model and constraints visually.
```bash
source ~/ros2_ws/install/setup.bash
source ~/teleop_slave/install/setup.bash
ros2 launch teleop_slave teleop_rviz.launch.py
```

#### Terminal 2: Low-Level Controller (Fairino Hardware Interface)
Connects directly to the robot hardware via the Fairino C++ SDK (`executeServoJ()`) or acts as a mocked publisher (`dummy_mode:=true`).
**For RViz Simulation (Hardware OFF):**
```bash
source ~/ros2_ws/install/setup.bash
source ~/teleop_slave/install/setup.bash
ros2 run teleop_slave fairino_lowlevel_controller_node --ros-args -p dummy_mode:=true
```
*(If you are connected to the physical real robot, omit the `--ros-args -p dummy_mode:=true` argument)*

#### Terminal 3: Master Bridge (MANUS UDP Receiver)
Receives the raw UDP packets from the VR glove on port 12345.
```bash
source ~/ros2_ws/install/setup.bash
source ~/teleop_slave/install/setup.bash
ros2 run teleop_slave master_bridge_node
```

#### Terminal 4: cuRobo MPPI Solver (GPU Motion Generation)
Listens for Cartesian target poses (`/curobo/pose_target`) and generates a collision-free joint stream (`/servo_target`).
*Requires the Python virtual environment.*
```bash
source ~/curobo_env/bin/activate
source ~/ros2_ws/install/setup.bash
source ~/teleop_slave/install/setup.bash
ros2 run teleop_slave curobo_mppi_solver.py
```

#### Terminal 5: Fairino Slave Node (Arm High-Level Logic)
Maps Manus Wrist pose to the Robot Base Frame, applies safety constraints, and sends targets to the cuRobo MPPI Solver.
```bash
source ~/ros2_ws/install/setup.bash
source ~/teleop_slave/install/setup.bash
ros2 run teleop_slave fairino_slave_node
```

#### Terminal 6: Tesollo Slave Node (Gripper Hardware Interface)
Maps the 20-DOF Manus finger positions directly to the Tesollo DG-5F Gripper via ModbusTCP.
**For Gazebo Simulation (Hardware OFF):**
```bash
source ~/ros2_ws/install/setup.bash
source ~/teleop_slave/install/setup.bash
ros2 run teleop_slave tesollo_slave_node --ros-args -p dummy_mode:=true
```
*(If you are connected to the physical DG-5F gripper, omit the `--ros-args -p dummy_mode:=true` argument. The default IP is `169.254.186.72` and Port is `502`. To change network settings, add `-p ip:="192.168.0.x" -p port:=502`)*
