# PiperX ROS 2 Humble 工作区说明

本工作区用于在 Ubuntu 22.04 + ROS 2 Humble 环境下控制松灵 AgileX PiperX 机械臂，并已接入 RGB-D 相机、手眼标定、MoveIt 和 AnyGrasp 抓取测试脚本。当前目录保留 ROS 2 Humble 核心驱动、必要 Python SDK、相机测试包和本地联调脚本，不再保留 ROS 1/catkin 示例、仿真示例或非 ROS2 教程工程。

机械臂到手后的上电、CAN 激活、RViz 显示、MoveIt 规划执行流程见 [USAGE.md](./USAGE.md)。

## 当前保留内容

```text
piperx_humble_ws/
├── src/agx_arm_ros/        # AgileX 机械臂 ROS2 驱动源码
├── src/grasp_interfaces/   # AnyGrasp 抓取候选、目标和执行 Action 接口
├── src/grasp_task_manager/ # 抓取任务状态机和后续执行调度入口
├── src/OrbbecSDK_ROS2_*    # Orbbec / Dabai RGB-D 相机测试包
├── sdk/pyAgxArm/           # AgileX 官方 Python SDK
├── sdk/piper_sdk/          # Piper 专用 Python SDK
├── scripts/                # 相机、标定、AnyGrasp、MoveIt 联调脚本
├── calibration_results/    # 手眼标定结果
├── logs/anygrasp_runs/     # AnyGrasp 抓取选择和规划记录
├── build/                  # colcon 构建中间产物
├── install/                # colcon 安装空间
└── log/                    # colcon 构建和列表日志
```

日常机械臂核心构建至少应识别以下 ROS2 包：

```text
agx_arm_ctrl
agx_arm_description
agx_arm_moveit
agx_arm_msgs
aruco_ros
grasp_interfaces
grasp_task_manager
handeye_calibration_ros
```

检查命令：

```bash
cd ~/piperx_humble_ws
colcon list --base-paths src/agx_arm_ros/src src/aruco_ros src/handeye_calibration_ros --names-only
```

## 目录功能说明

### `src/agx_arm_ros`

AgileX 机械臂 ROS2 驱动主仓库，包含 Piper、PiperX、Nero 等机械臂的 ROS2 节点、消息、URDF、MoveIt 配置、脚本和文档。

常用子目录：

```text
src/agx_arm_ros/
├── src/        # ROS2 包源码
├── scripts/    # CAN 激活、依赖安装等脚本
├── docs/       # CAN、TCP offset、Q&A 等官方说明
├── test/       # 官方测试和调试脚本
└── assets/     # README 或文档使用的图片资源
```

优先阅读：

```text
src/agx_arm_ros/README.md
src/agx_arm_ros/docs/CAN_USER.md
src/agx_arm_ros/docs/tcp_offset/TCP_OFFSET.md
src/agx_arm_ros/src/agx_arm_moveit/README.md
```

### `src/agx_arm_ros/src/agx_arm_ctrl`

机械臂 ROS2 控制节点包，构建类型为 `ament_python`。

主要职责：

- 连接机械臂 CAN 接口。
- 发布机械臂状态。
- 接收 ROS2 控制命令并下发到 SDK。
- 提供单机械臂启动入口。

关键文件：

```text
agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py
agx_arm_ctrl/launch/start_single_agx_arm.launch.py
agx_arm_ctrl/launch/start_single_agx_arm_rviz.launch.py
agx_arm_ctrl/launch/start_single_agx_arm_moveit.launch.py
```

常用启动命令：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
    can_port:=can0 \
    arm_type:=piper \
    effector_type:=none \
    tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

如果实际使用 PiperX，需要按官方支持的参数设置 `arm_type`，并确认当前驱动文档中对应的型号名。启动前必须确认 CAN 口、末端类型和 TCP 偏移都与真实硬件一致。

### `src/agx_arm_ros/src/agx_arm_msgs`

自定义 ROS2 消息包，构建类型为 `ament_cmake`。

主要职责：

- 定义机械臂状态消息。
- 定义夹爪、灵巧手、MIT 控制相关消息。
- 为 Python/C++ ROS2 节点生成消息接口。

消息文件：

```text
AgxArmStatus.msg
GripperStatus.msg
HandCmd.msg
HandPositionTimeCmd.msg
HandStatus.msg
MoveMITMsg.msg
```

这个包是其它控制包的基础依赖。修改消息定义后必须重新构建整个工作区，并重新 `source install/setup.bash`。

### `src/agx_arm_ros/src/agx_arm_description`

机械臂模型描述包，构建类型为 `ament_cmake`。

主要职责：

- 安装 Piper、PiperX、Nero、Revo2 等 URDF/Xacro 和网格模型。
- 提供 RViz 模型显示启动文件。
- 为 MoveIt 和 robot_state_publisher 提供机器人描述。

关键目录：

```text
agx_arm_description/agx_arm_urdf/
agx_arm_description/launch/
agx_arm_description/rviz/
```

模型显示示例：

```bash
ros2 launch agx_arm_description display.launch.py
```

### `src/agx_arm_ros/src/agx_arm_moveit`

MoveIt2 配置包，构建类型为 `ament_cmake`。

主要职责：

- 提供 MoveIt robot description、SRDF、运动学、关节限制和控制器配置。
- 启动 `move_group`、RViz、controller manager 等 MoveIt 相关组件。
- 根据机械臂型号、末端执行器和命名空间生成配置。

关键目录：

```text
agx_arm_moveit/config/
agx_arm_moveit/launch/
agx_arm_moveit/scripts/
```

常见配置文件：

```text
joint_limits.yaml
kinematics.yaml
ros2_controllers.yaml
moveit_controllers_none.yaml
moveit_controllers_gripper.yaml
sensors_3d.yaml
```

注意：`warehouse_ros_mongo` 在当前 Humble apt 源里可能不可用。该功能只和 MoveIt warehouse 数据库有关，默认 `db:=false` 时不启动数据库，正常机械臂控制和基础规划可以不使用它。

### `src/agx_arm_ros/scripts`

官方辅助脚本目录。

常用脚本：

```text
agx_arm_install_deps.sh    # 官方依赖安装脚本
can_activate.sh            # CAN 模块快速激活脚本
```

如果只接一块 CAN 模块，可以参考官方文档执行：

```bash
cd ~/piperx_humble_ws/src/agx_arm_ros/scripts
bash can_activate.sh
```

执行 CAN 激活脚本前，确认机械臂处于安全状态，周围无人员和障碍物。不要在机械臂未固定、急停不可用或 CAN 口不确定时启动运动相关节点。

### `src/agx_arm_ros/docs`

官方文档目录。

重点文档：

```text
CAN_USER.md
Q&A.md
tcp_offset/TCP_OFFSET.md
```

排查硬件连接、CAN 通信、TCP 偏移、MoveIt 配置前应先查这里。

### `sdk/pyAgxArm`

AgileX 官方 Python SDK，ROS2 驱动包会通过它和底层机械臂通信。

主要职责：

- 封装 AgileX 机械臂 CAN 协议。
- 提供 Piper、PiperX、Piper-H、Piper-L、Nero 等型号的 Python API。
- 提供官方 demos、协议实现和工具脚本。

安装方式：

```bash
cd ~/piperx_humble_ws/sdk/pyAgxArm
pip3 install --user .
```

验证：

```bash
python3 -c "import pyAgxArm; print('pyAgxArm ok')"
```

### `sdk/piper_sdk`

Piper 专用 Python SDK，适合直接运行 Piper 相关 Python 示例或低层 API 测试。

主要职责：

- 封装 Piper 机械臂协议。
- 提供 Piper API、运动学、消息结构、参数映射和硬件端口访问。
- 提供 `piper_sdk` Python 包。

安装方式：

```bash
cd ~/piperx_humble_ws
pip3 install --user ./sdk/piper_sdk
```

验证：

```bash
python3 -c "import importlib.util; assert importlib.util.find_spec('piper_sdk') is not None; print('piper_sdk installed')"
```

注意：`import piper_sdk` 会触发 SDK 顶层初始化，并可能尝试在包目录下创建日志目录。如果导入时报日志目录权限问题，优先检查是否把 Python 包安装到了只读路径，或 SDK 是否尝试在安装目录内创建日志。一般应使用 `--user` 安装，避免写入系统 Python 目录。

### `build`

`colcon build` 生成的中间构建目录。

特点：

- 可以删除后重新生成。
- 不建议手动修改。
- 如果切换过 `--symlink-install` 和普通安装方式，可能出现旧缓存冲突，可以清理相关包的 `build/<package>` 后重建。

例如：

```bash
rm -rf build/agx_arm_msgs build/agx_arm_ctrl build/agx_arm_description build/agx_arm_moveit
colcon build --base-paths src/agx_arm_ros/src
```

### `install`

`colcon build` 生成的安装空间。

主要用途：

- 存放构建后的 ROS2 包、launch 文件、Python 包入口和环境脚本。
- 运行 ROS2 包前需要 source。

常用命令：

```bash
source ~/piperx_humble_ws/install/setup.bash
```

### `log`

`colcon` 日志目录。

主要用途：

- 查看每次构建、测试和包列表命令的详细日志。
- 排查 CMake、Python、依赖或构建顺序问题。

可以删除，下一次 `colcon` 命令会重新生成。

## 环境依赖

基础环境：

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- `colcon`

推荐 apt 依赖：

```bash
sudo apt update
sudo apt install -y \
    can-utils \
    ethtool \
    python3-colcon-common-extensions \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-topic-tools \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-moveit \
    ros-humble-moveit-ros-perception \
    ros-humble-warehouse-ros-sqlite
```

Python 依赖：

```bash
pip3 install --user python-can scipy numpy websockets typing_extensions
pip3 install --user ./sdk/pyAgxArm
pip3 install --user ./sdk/piper_sdk
```

说明：

- 当前使用 Orbbec / Dabai RGB-D 相机做 AnyGrasp 输入，相机包和图像旋转脚本是联调链路的一部分。
- 当前不保留 ROS1/catkin 示例，因此不需要安装 `catkin`、`rospy`、`roscpp` 或 ROS1 bridge。
- `ros-humble-warehouse-ros-mongo` 在当前源中可能不存在，默认不启用 MoveIt warehouse 数据库时可以忽略。

## 构建流程

进入工作区：

```bash
cd ~/piperx_humble_ws
```

加载 ROS2 Humble：

```bash
source /opt/ros/humble/setup.bash
```

构建核心 ROS2 包：

```bash
colcon build --base-paths src/agx_arm_ros/src
```

加载工作区：

```bash
source install/setup.bash
```

确认包可见：

```bash
ros2 pkg list | grep agx_arm
```

预期至少包含：

```text
agx_arm_ctrl
agx_arm_description
agx_arm_moveit
agx_arm_msgs
```

## 运行前检查

硬件启动前建议逐项确认：

- 机械臂电源和急停状态正常。
- 机械臂固定可靠，运动空间内无人员和障碍物。
- CAN 模块已连接，接口名确认无误，例如 `can0`。
- `can-utils` 已安装，可以使用 `ip link`、`candump` 等工具排查。
- `arm_type`、`effector_type`、`tcp_offset` 参数与真实硬件一致。
- 第一次启动时先使用低风险姿态和低速度测试，不要直接运行大范围轨迹。

查看 CAN 口：

```bash
ip link show
```

查看 CAN 数据：

```bash
candump can0
```

## 常用启动命令

启动单机械臂控制节点：

```bash
source /opt/ros/humble/setup.bash
source ~/piperx_humble_ws/install/setup.bash

ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
    can_port:=can0 \
    arm_type:=piper \
    effector_type:=none \
    tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

直接运行控制节点：

```bash
ros2 run agx_arm_ctrl agx_arm_ctrl_single --ros-args \
    -p can_port:=can0 \
    -p arm_type:=piper \
    -p effector_type:=none \
    -p tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

启动 RViz 可视化：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py \
    can_port:=can0 \
    arm_type:=piper \
    effector_type:=none \
    tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

启动 MoveIt：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
    can_port:=can0 \
    arm_type:=piper \
    effector_type:=none \
    tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]' \
    db:=false
```

## AnyGrasp 抓取联调状态

当前联调链路：

```text
Orbbec / Dabai RGB-D 相机
  -> /camera_rotated/color/image_raw
  -> /camera_rotated/depth/image_raw
  -> AnyGrasp Top-N 推理
  -> base/world 坐标候选
  -> PiperX TCP 轴映射和夹爪中心补偿
  -> MoveIt IK + 桌面碰撞体 + pregrasp/approach/lift 规划
  -> PiperX FollowJointTrajectory 和 gripper action
```

关键话题：

```text
/camera_rotated/color/image_raw
/camera_rotated/depth/image_raw
/camera_rotated/color/camera_info
/anygrasp/grasp_candidates_base
/anygrasp/grasp_markers
/anygrasp/frozen_grasp_markers
/anygrasp/safety_status
/feedback/tcp_pose
/feedback/arm_status
/feedback/gripper_status
```

关键脚本：

```text
scripts/rotate_rgbd_publisher.py       # RGB-D 图像旋转发布
scripts/anygrasp_ros_infer.py          # AnyGrasp ROS2 推理桥
scripts/anygrasp_safe_pick.py          # 候选安全筛选和 RViz marker
scripts/anygrasp_apply_scene.py        # MoveIt 桌面碰撞体
scripts/anygrasp_full_pipeline.py      # plan-only / frozen execute 完整抓取流水线
scripts/anygrasp_roi_selector.py       # OpenCV ROI 选择器
scripts/moveit_ik_diagnostics.py       # MoveIt FK/IK 只读诊断
scripts/piperx_grasp_preflight.py      # 抓取测试前硬件和 MoveIt 只读预检
```

第一阶段正式包：

```text
src/grasp_interfaces/
  msg/GraspCandidate.msg
  msg/GraspCandidateArray.msg
  msg/GraspTarget.msg
  srv/DetectGrasps.srv
  action/ExecuteGrasp.action

src/grasp_task_manager/
  grasp_task_manager/state_machine.py
  grasp_task_manager/grasp_task_manager_node.py
```

当前 `grasp_task_manager` 已落地纯状态机和 ROS2 节点入口，先用于固化锁定、执行、空夹恢复和 abort/fault 状态流。它暂时不直接发送真实机械臂命令，后续再逐步接入 frozen candidate、plan-only 和 ExecuteGrasp action。

`grasp_task_manager` 默认启用 `require_home_before_detection=true`。调用 `~/start` 后会先进入 `HOMING`，不能直接跳到检测和执行阶段。真实回 Home 仍应通过 PiperX 驱动提供的 `/move_home` 服务或等价关节目标完成，避免用 Home 精确 TCP 位姿反求 IK。

当前实测结论：

- AnyGrasp 到 PiperX 的位姿链路已经跑通。
- PiperX 真实夹爪前向按 `tcp_link +Z` 处理，当前轴映射使用 `anygrasp_x_to_tcp_z`。
- 官方夹爪中心补偿方向按 `tcp_grasp_offset_z=0.12` 作为默认值。
- 由于 `0.12m` 对 PiperX 腕部姿态可达率较低，完整流水线支持从 `0.12,0.10,0.08,0.06` 做 plan-only 扫描，选择第一个完整可达结果。
- 最近一次可达结果使用 `tcp_grasp_offset_z=0.10`，并保存为 `logs/anygrasp_runs/frozen_selected_latest.json`。
- 夹爪默认打开宽度已调到 `0.08m`，避免 `0.06m` 开口不够导致空夹。
- 轨迹默认降速为 `velocity_scale=0.05`、`acceleration_scale=0.03`，减少真实机械臂抖动。
- Home 姿态下 `/feedback/tcp_pose` 与 MoveIt FK 一致，但 KDL IK 对当前精确 Home TCP 位姿返回 `-31`；这不是碰撞问题，因为 `avoid_collisions=false/true` 都失败。
- 当前 TCP 沿 `+Z` 偏移 `0.08m` 后，IK 和 plan-only 均成功。因此启动安全姿态应走 `/move_home` 或关节目标，抓取健康检查应使用偏移后的 pregrasp 探针。

推荐执行流程：

```bash
cd ~/piperx_humble_ws
source install/setup.bash

# 1. 只读预检：检查 arm/gripper 状态、当前 TCP、MoveIt IK pregrasp 探针
python3 scripts/piperx_grasp_preflight.py

# 2. 可选 plan-only 探针：从当前 TCP 上方 8cm 做 MoveIt 规划，不执行
python3 scripts/anygrasp_moveit_pregrasp.py \
    --target-source current_tcp \
    --offset-z 0.08 \
    --min-target-z 0.12 \
    --velocity-scale 0.05 \
    --acceleration-scale 0.03

# 3. 加入桌面碰撞体
python3 scripts/anygrasp_apply_scene.py

# 4. plan-only，Top-50 候选中优先使用官方 0.12m offset，失败后回退扫描
python3 scripts/anygrasp_full_pipeline.py \
    --stop-at-first \
    --max-candidates 50 \
    --tcp-grasp-offset-candidates 0.12,0.10,0.08,0.06 \
    --save-selected-path logs/anygrasp_runs/frozen_selected_latest.json \
    --report-path logs/anygrasp_runs/full_pipeline_report_latest.json

# 5. 只在 plan-only 通过且现场安全时，使用同一个 frozen candidate 执行
python3 scripts/anygrasp_full_pipeline.py \
    --frozen-selection-path logs/anygrasp_runs/frozen_selected_latest.json \
    --tcp-grasp-offset-z 0.10 \
    --execute \
    --report-path logs/anygrasp_runs/frozen_execute_report.json
```

注意：

- `--tcp-grasp-offset-z` 必须与 frozen 文件里记录的 `tcp_grasp_offset_z` 一致；否则同一个 raw grasp 会被转换成不同 TCP 目标。
- 不要让 plan 和 execute 分别订阅不同 AnyGrasp 新帧。必须先保存 frozen candidate，再用同一份 frozen 文件执行。
- 真实执行前必须确认 `/feedback/arm_status` 中 `err_status=0`、关节限位和通信错误为 false。
- 如果夹爪反馈 `width` 接近 0 且 `force` 很小，通常表示空夹。

暂停测试时停止相关进程：

```bash
ps -eo pid,args | rg '[a]nygrasp_ros_infer.py|[a]nygrasp_safe_pick.py|[a]nygrasp_full_pipeline.py|[a]nygrasp_roi_selector.py|[a]nygrasp_moveit_pregrasp.py'
kill <pid...>
```

构建第一阶段正式包：

```bash
cd ~/piperx_humble_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select grasp_interfaces grasp_task_manager --symlink-install
source install/setup.bash
ros2 interface show grasp_interfaces/msg/GraspCandidate
ros2 pkg executables grasp_task_manager
```

## 维护建议

- 日常机械臂核心调试优先构建 `src/agx_arm_ros/src`；需要手眼标定时再加入 `src/aruco_ros` 和 `src/handeye_calibration_ros`。
- 新增依赖时优先写入对应 ROS2 包的 `package.xml` 或 Python `setup.py`。
- 不要把 ROS1/catkin 包放回该工作区；如果确实需要 ROS1 示例，应另建 ROS1 工作区。
- 相机和 AnyGrasp 联调脚本可以保留在当前工作区；仿真、强化学习等重依赖示例仍建议单独建工作区。
- 硬件调试时先确认通信链路，再判断上层控制问题，避免把 CAN/驱动问题误判为 MoveIt 问题。
- 清理构建产物时可以删除 `build/`、`install/`、`log/`，但不要删除 `src/` 和 `sdk/`。

## 当前精简状态

已移除：

- ROS1/catkin 示例包。
- `examples/Agilex-College` 下的非 ROS2 示例、仿真和教学工程。

保留：

- PiperX ROS2 Humble 核心驱动。
- 第一阶段正式抓取接口包 `grasp_interfaces`。
- 第一阶段抓取任务状态机包 `grasp_task_manager`。
- Orbbec / Dabai RGB-D 相机测试包和旋转图像发布脚本。
- AnyGrasp、ROI、安全筛选、MoveIt planning scene 和完整抓取流水线脚本。
- 官方 ROS2 消息、控制、URDF、MoveIt 配置。
- `pyAgxArm` 和 `piper_sdk` Python SDK。
