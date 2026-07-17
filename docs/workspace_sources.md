# Workspace Source Manifest

This workspace is a ROS 2 overlay for PiperX + RGB-D camera + AnyGrasp
integration. Several directories are upstream repositories kept in-place for
local hardware integration.

## Root Repository

```text
https://github.com/huzhaoyi/piperx_humble_ws.git
```

The root repository stores workspace-level scripts, configuration, documents,
tests, calibration results, and the local AnyGrasp grasp-management packages.

Generated ROS 2 outputs are intentionally ignored:

```text
build/
install/
log/
Log/
logs/
```

## External Source Repositories

```text
src/agx_arm_ros
  remote: https://github.com/agilexrobotics/agx_arm_ros.git
  branch: ros2
  local patch: patches/agx_arm_ros_local_changes.patch

src/aruco_ros
  remote: https://github.com/pal-robotics/aruco_ros.git
  branch: humble-devel

src/OrbbecSDK_ROS2_dabaiA_test
  remote: https://github.com/orbbec/OrbbecSDK_ROS2.git
  branch: dabaiA_test

src/OrbbecSDK_ROS2_main_legacy
  remote: https://github.com/orbbec/OrbbecSDK_ROS2.git
  branch: main-legacy

src/handeye_calibration_ros
  remote: https://github.com/agilexrobotics/handeye_calibration_ros.git
  branch: humble
  local patch: patches/handeye_calibration_ros_local_changes.patch

sdk/piper_sdk
  remote: https://github.com/agilexrobotics/piper_sdk.git
  branch: 1_0_0_beta

sdk/pyAgxArm
  remote: https://github.com/agilexrobotics/pyAgxArm.git
  branch: master
```

## Local Packages

These packages are maintained in the root workspace:

```text
src/grasp_interfaces
src/grasp_task_manager
```

## Rebuild

After cloning or restoring external repositories, rebuild the local interfaces
and task manager first:

```bash
colcon build --packages-select grasp_interfaces grasp_task_manager --symlink-install
source install/setup.bash
```

Then build the hardware stack as needed for the connected PiperX and camera.
