# Teleop Slave

ROS 2 (Humble) 기반 실시간 원격조작 패키지.  
**Fairino FR5** 6-DOF 매니퓰레이터와 **Tesollo Delto M** 손가락 그리퍼를 Manus VR 글러브로 제어한다.  
GPU 가속 IK는 **NVIDIA cuRobo** (`IKSolver`)로 수행하며, 로봇 하드웨어 SDK는 패키지 내부에 포함되어 있다.

---

## Architecture

```
[Manus VR Glove]
       |  UDP:12345
       v
 master_bridge_node          → /manus/wrist_pose       (PoseStamped)
                             → /manus/finger_joints     (JointState, 20-DOF)
       |
       v
 fairino_slave_node          ← /manus/wrist_pose
                             ← /robot_pose              (현재 TCP, 영점잡기용)
                             → /curobo/pose_target      (PoseStamped)
                             → /enable_streaming        (SetBool service)
       |
       v
 curobo_mppi_solver.py       ← /curobo/pose_target
                             ← /robot_joint_states      (현재 관절각, seed용)
                             → /servo_target            (JointState, rad)
       |
       v
 fairino_lowlevel_controller_node
                             ← /servo_target            (스트리밍 제어)
                             ← /trajectory_points       (Float64MultiArray)
                             → /robot_joint_states      (JointState, rad)  ← hardware
                             → /joint_states            (JointState, rad)  ← dummy_mode
                             → /robot_pose              (PoseStamped, m)
                             srv: /enable_streaming     (SetBool)
                             srv: /execute_trajectory   (Trigger)

 fairino_state_printer_node  ← /robot_joint_states
                             ← /robot_pose
                             (터미널에 10Hz로 출력)

 tesollo_slave_node          ← /manus/finger_joints
                             → /joint_trajectory_controller/joint_trajectory
```

---

## Nodes

| Node | 언어 | 역할 |
|---|---|---|
| `master_bridge_node` | C++ | Manus SDK UDP 수신 → `/manus/wrist_pose`, `/manus/finger_joints` 발행 |
| `fairino_slave_node` | C++ | 손목 위치를 로봇 프레임으로 변환, 안전 제한 적용, cuRobo로 전달 |
| `curobo_mppi_solver.py` | Python | cuRobo IK Solver로 TCP 목표 → 관절 각도 변환, `/servo_target` 발행 |
| `fairino_lowlevel_controller_node` | C++ | Fairino SDK로 ServoJ 실행, 관절 상태·TCP Pose를 ROS2로 발행 |
| `fairino_state_printer_node` | C++ | `/robot_joint_states`, `/robot_pose` 구독 → 터미널 대시보드 출력 |
| `tesollo_slave_node` | C++ | 손가락 20-DOF 매핑 → Tesollo DG-5F 그리퍼 제어 |

---

## Topics & Services

### Published Topics

| Topic | Type | Publisher | 설명 |
|---|---|---|---|
| `/manus/wrist_pose` | `geometry_msgs/PoseStamped` | `master_bridge_node` | 손목 위치·자세 |
| `/manus/finger_joints` | `sensor_msgs/JointState` | `master_bridge_node` | 손가락 20관절 (deg→rad) |
| `/curobo/pose_target` | `geometry_msgs/PoseStamped` | `fairino_slave_node` | 안전 제한 적용된 TCP 목표 |
| `/servo_target` | `sensor_msgs/JointState` | `curobo_mppi_solver.py` | IK 결과 관절 목표 (rad) |
| `/robot_joint_states` | `sensor_msgs/JointState` | `fairino_lowlevel_controller_node` | 실제 관절 각도 (rad) |
| `/joint_states` | `sensor_msgs/JointState` | `fairino_lowlevel_controller_node` | dummy\_mode 전용 |
| `/robot_pose` | `geometry_msgs/PoseStamped` | `fairino_lowlevel_controller_node` | 실제 TCP Pose (m, quaternion) |

### Services

| Service | Type | Server | 설명 |
|---|---|---|---|
| `/enable_streaming` | `std_srvs/SetBool` | `fairino_lowlevel_controller_node` | ServoJ 스트리밍 on/off |
| `/execute_trajectory` | `std_srvs/Trigger` | `fairino_lowlevel_controller_node` | 사전 로드된 궤적 실행 |

---

## Key Parameters

### `fairino_lowlevel_controller_node`
| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `robot_ip` | `192.168.58.2` | 로봇 IP |
| `dummy_mode` | `false` | `true`시 물리 연결 없이 시뮬레이션 |

### `fairino_slave_node`
| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `workspace_radius` | `0.85` (m) | 최대 작업 반경 |
| `min_z` | `0.05` (m) | 최소 높이 (바닥 충돌 방지) |

### `fairino_state_printer_node`
| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `joint_topic` | `/robot_joint_states` | 구독할 JointState 토픽 |
| `pose_topic` | `/robot_pose` | 구독할 PoseStamped 토픽 |

### `curobo_mppi_solver.py`
- 로봇 설정 파일 경로: `/home/hq/teleop_slave/config/fr5.yml`
- IK seeds: 30개, position threshold: 5mm, rotation threshold: ~3°

---

## Setup & Installation

### Prerequisites
- Ubuntu 22.04 + ROS 2 Humble
- NVIDIA GPU + CUDA Toolkit
- `fairino5_v6_moveit2_config` 패키지 (RViz 시각화용)

### cuRobo 설치
```bash
sudo apt install python3.10-venv
python3 -m venv ~/curobo_env
source ~/curobo_env/bin/activate
# CUDA 버전에 맞는 PyTorch 먼저 설치
pip install torch torchvision torchaudio
# 소스에서 cuRobo 설치 (pip install curobo는 더미 패키지)
pip install --no-build-isolation git+https://github.com/NVlabs/curobo.git
```

### Build
```bash
cd ~/teleop_slave
colcon build --packages-select teleop_slave
source install/setup.bash
```

---

## `fairino_slave_node` 조작 방법

| 키 | 동작 |
|---|---|
| `Space` | 현재 로봇 TCP 위치로 영점 잡고 스트리밍 시작 |
| `Space` (스트리밍 중) | 스트리밍 중지 |
| `q` / `ESC` | 노드 종료 |

> **영점 잡기**: Space 누르는 순간의 로봇 실제 위치(`/robot_pose`)와 손 위치를 매핑하여 이후 손 움직임이 로봇의 델타 움직임이 된다.

---

## 터미널별 실행 명령어

> 모든 터미널에서 공통으로 아래 source 명령을 먼저 수행한다.
> ```bash
> source ~/teleop_slave/install/setup.bash
> ```

---

### Terminal 1 — RViz 시각화 (선택)

```bash
source ~/teleop_slave/install/setup.bash
ros2 launch teleop_slave teleop_rviz.launch.py
```

> `fairino5_v6_moveit2_config` 패키지의 `robot_state_publisher`와 RViz를 함께 실행한다.

---

### Terminal 2 — Fairino 로우레벨 컨트롤러

**실제 하드웨어 연결 시:**
```bash
source ~/teleop_slave/install/setup.bash
ros2 run teleop_slave fairino_lowlevel_controller_node \
  --ros-args -p robot_ip:=192.168.58.2
```

**시뮬레이션 (하드웨어 없음):**
```bash
source ~/teleop_slave/install/setup.bash
ros2 run teleop_slave fairino_lowlevel_controller_node \
  --ros-args -p dummy_mode:=true
```

> `/robot_joint_states`, `/robot_pose` 발행.  
> dummy_mode에서는 `/joint_states` 발행 (`/robot_joint_states` 대신).

---

### Terminal 3 — Manus 브릿지 (UDP 수신)

```bash
source ~/teleop_slave/install/setup.bash
ros2 run teleop_slave master_bridge_node
```

> UDP port `12345`에서 Manus Core 패킷 수신 → `/manus/wrist_pose`, `/manus/finger_joints` 발행.

---

### Terminal 4 — cuRobo IK Solver (GPU 필요)

```bash
source ~/curobo_env/bin/activate
source ~/teleop_slave/install/setup.bash
ros2 run teleop_slave curobo_mppi_solver.py
```

> `/curobo/pose_target` 구독 → cuRobo IK 풀기 → `/servo_target` 발행.  
> 초기화에 약 10초 소요.

---

### Terminal 5 — Fairino 슬레이브 (고레벨 제어)

```bash
source ~/teleop_slave/install/setup.bash
ros2 run teleop_slave fairino_slave_node
```

> `/manus/wrist_pose` 수신 → 안전 제한 적용 → `/curobo/pose_target` 발행.  
> **Space** 키로 스트리밍 시작/중지.

---

### Terminal 6 — Tesollo 그리퍼

**시뮬레이션 (Gazebo, 하드웨어 없음):**
```bash
source ~/teleop_slave/install/setup.bash
ros2 run teleop_slave tesollo_slave_node --ros-args -p dummy_mode:=true
```

**실제 하드웨어 연결 시 (2개 터미널):**

Terminal 6-A — DG5F ros2_control 드라이버 시작:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch dg5f_driver dg5f_right_driver.launch.py \
  delto_ip:=169.254.186.72 delto_port:=502
```

Terminal 6-B — 텔레오퍼레이션 노드:
```bash
source ~/teleop_slave/install/setup.bash
ros2 run teleop_slave tesollo_slave_node
```

---

### Terminal 7 — 로봇 상태 모니터 (선택)

**실제 하드웨어 모드:**
```bash
source ~/teleop_slave/install/setup.bash
ros2 run teleop_slave fairino_state_printer_node
```

**dummy_mode (컨트롤러가 `/joint_states` 발행 시):**
```bash
source ~/teleop_slave/install/setup.bash
ros2 run teleop_slave fairino_state_printer_node \
  --ros-args -p joint_topic:=/joint_states
```

> 10Hz로 터미널을 갱신하며 관절 각도(deg)와 TCP 위치/자세(m, deg)를 출력한다.  
> `fairino_lowlevel_controller_node`가 실행 중이어야 데이터가 표시된다.
