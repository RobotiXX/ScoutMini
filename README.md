# AgileX Scout Mini
This repository contains resources for RobotiXX Lab's Scout Mini including ROS 2 packages, instructions, applications and changelog

## Overview

[Online documentation](https://agilexrobotics.gitbook.io/scout-mini/1-scout-mini-introduction)

[Downloadable version](https://global.agilex.ai/pages/download-manual)

## Hardware Setup
### Onboard computer
Jetson Orin AGX Developer Kit 64GB with Jetpack 6.2

#TODO: add notes for customizing kernel build

## ROS 2 Packages

Main packages:
- [ugv_sdk](https://github.com/RobotiXX/ugv_sdk): our fork for C++ interface to communicate with the robot.
- [scout_ros2](https://github.com/RobotiXX/scout_ros2): our fork for ROS 2 wrapper including our own URDF models with sensor box and mounts.

## Workspace Setup
### Clone

Clone the repository with submodules:

```bash
git clone --recurse-submodules git@github.com:RobotiXX/ScoutMini.git
cd ScoutMini
```

If you already cloned the repository without submodules:

```bash
git submodule update --init --recursive
```

### Pull Latest Changes

Pull the main repository and update submodules to the commits referenced by the repo:

```bash
git pull --recurse-submodules
git submodule sync --recursive
git submodule update --init --recursive
```

### Build ROS 2 Workspace

Build the workspace with the provided helper script:

```bash
./scripts/build_ros2_ws.sh
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

Notes:
- The build script uses `--symlink-install`, `Release` mode, and disables tests by default.
- ZED packages are skipped automatically if the ZED SDK is not installed under `/usr/local/zed`.
- `zed_debug` is skipped by default. Use `./scripts/build_ros2_ws.sh --with-zed-debug` if needed.
- If another workspace on your machine also contains `ugv_sdk`, the version from the workspace you source last is the one ROS 2 will use.

## Teleoperation
### AgileX Remote Controller
Refer to [online](https://agilexrobotics.gitbook.io/scout-mini/1-scout-mini-introduction) or [downloadable](https://global.agilex.ai/pages/download-manual) documentation.

### PS4 Dual Shock Controller
Based on [ds4_driver](https://github.com/RobotiXX/ds4_driver/tree/humble-devel), refer to [socnav_data_collection](https://github.com/RobotiXX/socnav_data_collection) for example mappings and usage.

## Applications
### Scenario-based social navigation data collection (2/2025 - 5/2025) 

Refer to this repo: [socnav_data_collection](https://github.com/RobotiXX/socnav_data_collection)

## Changelog
3/2025: Upgraded Jetson AGX Orin to JetPack 6.2 and ROS 2 Humble stacks to avoid end-of-life OS and ROS.

3/2025: Switched to use Waveshare SN65HVD230 CAN Transceiver with Jetson I/O pins as Jetpack 6.2 does not work with gs_usb kernel for Geschwister Schneider CAN USB.



