# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Windows C++ client that connects to **Manus Core** (glove tracking software), captures right-hand wrist tracker + finger ergonomics + fingertip positions from the Manus SDK, and broadcasts the data over UDP at ~50Hz. Designed for robotic teleoperation (e.g., streaming to a ROS2 node on Ubuntu).

## Build & Run

**Prerequisites:** Windows 10/11, Visual Studio 2022 (Desktop C++ workload), Manus Core running with a paired right glove.

```powershell
# Build (Release x64) — auto-detect MSBuild via vswhere
$MSBuildPath = & "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe" -latest -requires Microsoft.Component.MSBuild -find MSBuild\**\Bin\MSBuild.exe
& $MSBuildPath TeleopMasterClient.vcxproj /p:Configuration=Release /p:Platform=x64

# Run
.\Output\x64\Release\TeleopMasterClient_Windows.exe
```

Build system is a `.vcxproj` (Visual Studio), **not** CMake. Platform toolset is v145, C++17, x64 only. The post-build step copies `ManusSDK.dll` to the output directory.

There are no tests or linting configured.

## Architecture

**Single-class design:** `TeleopMasterClient` (inherits `SDKClientPlatformSpecific`) handles everything — SDK init, callback registration, UDP socket, main loop.

**Data flow:**
1. `Initialize()` → platform init + SDK init + coordinate system config + callback registration
2. `Run()` → opens UDP socket → connects to Manus Core host → calls `CoreSdk_SetRawSkeletonHandMotion(HandMotion_Auto)` → enters 50Hz main loop
3. Four **static callbacks** receive data asynchronously from the SDK (all guard with `m_DataMutex`):
   - `OnLandscapeCallback` — discovers right glove ID
   - `OnErgonomicsCallback` — receives 40-element finger joint angle array (right glove data at offset 20, 4 joints × 5 fingers)
   - `OnTrackerStreamCallback` — receives VIVE tracker position/rotation for the right wrist
   - `OnRawSkeletonStreamCallback` — on first frame, dumps all skeleton node metadata (IDs, parent IDs, chain types, finger joint types) to console for diagnostics, then resolves tip node IDs and wrist node ID (`m_WristNodeResolved` flag). Subsequent frames extract palm-local fingertip positions (world-to-palm transform via inverse quaternion rotation)
4. Main loop reads cached data under mutex, applies calibration offsets (spacebar zeroing), packs into `HandDataPacket` (172 bytes, `#pragma pack(1)`), sends via UDP

**Startup & diagnostics:**
- `main()` prints step indicators (`[1/3]`, `[2/3]`, `[3/3]`) for initialization, main loop entry, and shutdown
- Each SDK call logs its progress and includes SDK return codes on failure, followed by `system("pause")` for error visibility
- `m_NodeDumpTime` records when the one-time node dump was printed; the main loop skips `cls` for 10 seconds after the dump so users can read the diagnostic output

**Singleton pattern:** `s_Instance` static pointer lets static callbacks access instance data.

## UDP Packet Structure

```
HandDataPacket (172 bytes, packed):
  uint32_t frame           [4B]
  float wristPos[3]        [12B] — X, Y, Z meters
  float wristQuaternion[4] [16B] — W, X, Y, Z
  float fingerFlexion[20]  [80B] — 5 fingers × 4 joints (MCP_Spread, MCP_Stretch, PIP, DIP)
  float fingertipPos[15]   [60B] — 5 fingertips × (X, Y, Z) in palm-local frame
```

Target IP/port are hardcoded in `Run()` (default: `192.168.0.112:12345`).

## Key Conventions

- Comments are primarily in Korean (한국어)
- Manus SDK naming: prefix `t_` for local variables, `p_` for parameters, `m_` for members, `s_` for statics
- The ergonomics data array uses **offset 20** for right-hand finger joints (SDK convention)
- Finger order everywhere: Thumb, Index, Middle, Ring, Pinky
- Sign conventions differ between Manus glove and Tesollo robot hand (see mapping table in Readme.md) — Ab/Ad signs are inverted for all fingers, and Thumb CMC joints have swapped order + sign inversion

## Platform Abstraction

`PlatformSpecific/` provides `SDKClientPlatformSpecific` base class with keyboard input, console management, and filesystem helpers. Windows implementation under `PlatformSpecific/Windows/`, Linux stub under `PlatformSpecific/Linux/`. Only Windows is actively used.
