# Manus SDK Minimal Client (Windows) - UDP Teleop Broadcaster

This project is a customized, minimal C++ client built on top of the Manus SDK v3.1.1. It is designed to run on Windows and continuously stream hand tracking and ergonomics (finger joint) data over a UDP connection. 

This is particularly useful for robotic teleoperation (e.g., streaming MANUS Glove data to a ROS2 master or other robotics middlewares like Ubuntu/Linux).

## Overview

The client connects locally to the **Manus Core** software, registers necessary callbacks for tracker and ergonomics streams, and then processes the incoming data in real-time. 

Specifically, it extracts:
1. **Right Wrist Tracker Data:** 
   - 3D Position (`X, Y, Z`)
   - 3D Rotation (Quaternion: `W, X, Y, Z`)
2. **Right Glove Ergonomics Data (20 Joints):**
   - Extracts 4 metrics per finger (MCP Spread, MCP Stretch, PIP Stretch, DIP Stretch).
   - Thumb, Index, Middle, Ring, Pinky.
3. **Right Glove Palm-Local Fingertip Positions:**
   - 5 fingertip positions in the MANUS raw-skeleton palm frame.

The extracted data is packed into a custom `HandDataPacket` array structured as follows and broadcasted via UDP at ~50Hz:

```cpp
#pragma pack(push, 1)
struct HandDataPacket {
    uint32_t frame;
    float wristPos[3];   // X, Y, Z (meters)
    float wristQuaternion[4]; // w, x, y, z
    float fingerFlexion[20]; // Thumb, Index, Middle, Ring, Pinky (MCP_Sp, MCP_St, PIP, DIP)
    float fingertipPos[15]; // 5 fingertips × (X, Y, Z) in MANUS palm-local frame
};
#pragma pack(pop)
```
*Total Packet Size: 4 + 12 + 16 + 80 + 60 = 172 bytes.*

## Core Files

* `TeleopMasterClient.hpp` / `TeleopMasterClient.cpp`: Contains the main SDK logic, Core connection, Callback handlers, UDP Socket setup, and the main run-loop.
* `ManusSDK/`: Pre-compiled libraries, DLLs, and header files for Manus SDK.
* `.vcxproj` / `.sln`: Visual Studio 2022 project files for building the client.

## Network Configuration

By default, the UDP packets are sent to:
* **Target IP**: `192.168.0.112` (Modify in `TeleopMasterClient.cpp::Run()`)
* **Target Port**: `12345`
* **Frequency**: ~50Hz (20ms sleep per cycle)

## How to Build & Run

### Prerequisites
* Windows 10/11
* Visual Studio 2022 (with Desktop development with C++)
* Manus Core installed and running in the background with a paired right glove and tracker.

### Building via MSBuild / PowerShell
```powershell
cd TeleopMasterClient_Windows
# Auto-detect MSBuild path using vswhere
$MSBuildPath = & "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe" -latest -requires Microsoft.Component.MSBuild -find MSBuild\**\Bin\MSBuild.exe
& $MSBuildPath TeleopMasterClient.vcxproj /p:Configuration=Release /p:Platform=x64
```
*(The MSBuild path is automatically detected using `vswhere.exe`.)*

### Running
Once built successfully, the executable will be placed in the `Output/x64/Release/` folder.
```powershell
.\Output\x64\Release\TeleopMasterClient_Windows.exe
```

Upon execution, the client follows a step-by-step startup sequence with progress indicators (`[1/3]`, `[2/3]`, `[3/3]`). Error messages include SDK return codes for easier debugging and pause for user acknowledgement.

Once connected to Manus Core, the terminal dynamically displays the parsed Right Wrist positional and rotational data, along with the real-time angles of all 20 finger joints.

**Diagnostic Node Dump:** After the first raw skeleton frame is received, a one-time node dump is printed to the console showing all skeleton node metadata — node IDs, parent IDs, chain types, and finger joint types. The console avoids clearing for 10 seconds so you can read this output. This is useful for verifying that the SDK is reporting the expected skeleton structure.

- Press `Spacebar` to zero the VIVE Tracker's position and rotation.
- Press `Esc` to exit gracefully.



## 🧤 Manus Glove Data Index

| Finger | Joint 1 | Joint 2 | Joint 3 | Joint 4 |
| :--- | :--- | :--- | :--- | :--- |
| **엄지 (Thumb)** | CMC_Fl(+)/Ex(-) | CMC_Ab(+)/Ad(-) | MCP_Fl(+)/Ex(-) | IP_Fl(+)/Ex(-) |
| **검지 (Index)** | MCP_Ab(+)/Ad(-) | MCP_Fl(+)/Ex(-) | PIP_Fl(+)/Ex(-) | DIP_Fl(+)/Ex(-) |
| **중지 (Middle)**| MCP_Ab(+)/Ad(-) | MCP_Fl(+)/Ex(-) | PIP_Fl(+)/Ex(-) | DIP_Fl(+)/Ex(-) |
| **약지 (Ring)**  | MCP_Ab(+)/Ad(-) | MCP_Fl(+)/Ex(-) | PIP_Fl(+)/Ex(-) | DIP_Fl(+)/Ex(-) |
| **소지 (Pinky)** | MCP_Ab(+)/Ad(-) | MCP_Fl(+)/Ex(-) | PIP_Fl(+)/Ex(-) | DIP_Fl(+)/Ex(-) |
| 손바닥을 보았을 때 시계방향이 Ab, 반시계방향이 Ad |

---


## 🤖 Tesollo Hand Motor Index & Range

| Finger | Motor Index | Range (Degrees) | Joint Type |
| :--- | :---: | :---: | :--- |
| **엄지 (Thumb)** | Motor 1 | `-22 ~ 77` | CMC_Ab(-)/Ad(+) |
| | Motor 2 | `0 ~ 155` | CMC_Fl(-)/Ex(+) |
| | Motor 3 | `-90 ~ 90` | MCP_Fl(+)/Ex(-) |
| | Motor 4 | `-90 ~ 90` | IP_Fl(+)/Ex(-) |
| **검지 (Index)** | Motor 5 | `-31 ~ 20` | MCP_Ab(-)/Ad(+) |
| | Motor 6 | `0 ~ 115` | MCP_Fl(+)/Ex(-) |
| | Motor 7 | `-90 ~ 90` | PIP_Fl(+)/Ex(-) |
| | Motor 8 | `-90 ~ 90` | DIP_Fl(+)/Ex(-) |
| **중지 (Middle)**| Motor 9 | `-30 ~ 30` | MCP_Ab(-)/Ad(+) |
| | Motor 10 | `0 ~ 115` | MCP_Fl(+)/Ex(-) |
| | Motor 11 | `-90 ~ 90` | PIP_Fl(+)/Ex(-) |
| | Motor 12 | `-90 ~ 90` | DIP_Fl(+)/Ex(-) |
| **약지 (Ring)** | Motor 13 | `-15 ~ 32` | MCP_Ab(-)/Ad(+) |
| | Motor 14 | `0 ~ 110` | MCP_Fl(+)/Ex(-) |
| | Motor 15 | `-90 ~ 90` | PIP_Fl(+)/Ex(-) |
| | Motor 16 | `-90 ~ 90` | DIP_Fl(+)/Ex(-) |
| **소지 (Pinky)** | Motor 17 | `0 ~ 60` | 위(손가락)에서 봤을 때 반시계가 (+) |
| | Motor 18 | `-15 ~ 90` | MCP_Ab(-)/Ad(+) |
| | Motor 19 | `-90 ~ 90` | MCP_Fl(+)/Ex(-) |
| | Motor 20 | `-90 ~ 90` | IP_Fl(+)/Ex(-) |
| 손바닥을 보았을 때 시계방향이 Ab, 반시계방향이 Ad |

---

## 📊 종합 맵핑 비교표 (All Motors)

Manus Glove 데이터와 Tesollo Hand 모터 간의 통합된 비교표입니다.

| 손가락 (Finger) | 동작 (Joint Type) | Manus 배열 (Joint) | Manus 측정 방향 | Tesollo Motor | Tesollo 가동 범위 | Tesollo 측정 방향 | 비고 (Mapping 주의점) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **엄지 (Thumb)** | CMC Ab/Ad | Joint 2 | `Ab(+) / Ad(-)` | Motor 1 | `-22 ~ 77` | `Ab(-) / Ad(+)` | **부호 반전 (-1)** |
| | CMC Fl/Ex | Joint 1 | `Fl(+) / Ex(-)` | Motor 2 | `0 ~ 155` | `Fl(-) / Ex(+)` | **순서 교차 & 부호 반전 (-1)** |
| | MCP Fl/Ex | Joint 3 | `Fl(+) / Ex(-)` | Motor 3 | `-90 ~ 90` | `Fl(+) / Ex(-)` | |
| | IP Fl/Ex | Joint 4 | `Fl(+) / Ex(-)` | Motor 4 | `-90 ~ 90` | `Fl(+) / Ex(-)` | |
| **검지 (Index)** | MCP Ab/Ad | Joint 1 | `Ab(+) / Ad(-)` | Motor 5 | `-31 ~ 20` | `Ab(-) / Ad(+)` | **부호 반전 (-1)** |
| | MCP Fl/Ex | Joint 2 | `Fl(+) / Ex(-)` | Motor 6 | `0 ~ 115` | `Fl(+) / Ex(-)` | |
| | PIP Fl/Ex | Joint 3 | `Fl(+) / Ex(-)` | Motor 7 | `-90 ~ 90` | `Fl(+) / Ex(-)` | |
| | DIP Fl/Ex | Joint 4 | `Fl(+) / Ex(-)` | Motor 8 | `-90 ~ 90` | `Fl(+) / Ex(-)` | |
| **중지 (Middle)**| MCP Ab/Ad | Joint 1 | `Ab(+) / Ad(-)` | Motor 9 | `-30 ~ 30` | `Ab(-) / Ad(+)` | **부호 반전 (-1)** |
| | MCP Fl/Ex | Joint 2 | `Fl(+) / Ex(-)` | Motor 10 | `0 ~ 115` | `Fl(+) / Ex(-)` | |
| | PIP Fl/Ex | Joint 3 | `Fl(+) / Ex(-)` | Motor 11 | `-90 ~ 90` | `Fl(+) / Ex(-)` | |
| | DIP Fl/Ex | Joint 4 | `Fl(+) / Ex(-)` | Motor 12 | `-90 ~ 90` | `Fl(+) / Ex(-)` | |
| **약지 (Ring)** | MCP Ab/Ad | Joint 1 | `Ab(+) / Ad(-)` | Motor 13 | `-15 ~ 32` | `Ab(-) / Ad(+)` | **부호 반전 (-1)** |
| | MCP Fl/Ex | Joint 2 | `Fl(+) / Ex(-)` | Motor 14 | `0 ~ 110` | `Fl(+) / Ex(-)` | |
| | PIP Fl/Ex | Joint 3 | `Fl(+) / Ex(-)` | Motor 15 | `-90 ~ 90` | `Fl(+) / Ex(-)` | |
| | DIP Fl/Ex | Joint 4 | `Fl(+) / Ex(-)` | Motor 16 | `-90 ~ 90` | `Fl(+) / Ex(-)` | |
| **소지 (Pinky)** | 부가 관절 (벌림) | - | - | Motor 17 | `0 ~ 60` | 반시계방향(+) | Manus 측정값 없음 |
| | MCP Ab/Ad | Joint 1 | `Ab(+) / Ad(-)` | Motor 18 | `-15 ~ 90` | `Ab(-) / Ad(+)` | **부호 반전 (-1)** |
| | MCP Fl/Ex | Joint 2 | `Fl(+) / Ex(-)` | Motor 19 | `-90 ~ 90` | `Fl(+) / Ex(-)` | |
| | IP Fl/Ex | Joint 3, 4 | `Fl(+) / Ex(-)` | Motor 20 | `-90 ~ 90` | `Fl(+) / Ex(-)` | Tesollo는 IP 관절 1개 |
