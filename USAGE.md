# PiperX 到手后的使用流程

本文档按真实机械臂到手后的操作顺序编写，适用于当前工作区：

```text
~/piperx_humble_ws
```

当前工作区只保留 ROS 2 Humble 核心驱动和 SDK，不包含 Orbbec 相机、ROS1 示例或仿真示例。

## 0. 总体流程

第一次使用不要直接进入 MoveIt 规划执行。推荐顺序：

```text
硬件检查
  -> CAN 激活
  -> 基础驱动启动
  -> 反馈话题确认
  -> RViz 只显示
  -> MoveIt 只规划
  -> MoveIt 小范围执行
```

每一步确认正常后再进入下一步。机械臂是执行机构，启动前必须保证急停、固定、线缆和运动空间都安全。

## 1. 硬件上电前检查

确认以下项目：

- 机械臂底座固定牢靠。
- 机械臂运动范围内没有人员、工具、线缆或其它障碍物。
- 急停按钮可用，且你知道如何立即断电或急停。
- 机械臂电源规格正确。
- CAN 模块为机械臂配套官方 CAN 模块。
- CAN-H、CAN-L、GND 接线可靠。
- USB CAN 模块已插入电脑。
- 末端执行器类型确认清楚：无末端、AgileX 夹爪或 Revo2 灵巧手。

PiperX 在本驱动中的 `arm_type` 使用：

```text
piper_x
```

常见末端参数：

```text
none         # 无末端执行器
agx_gripper  # AgileX 夹爪
revo2        # Revo2 灵巧手
```

如果不确定末端类型，先用：

```text
effector_type:=none
```

## 2. 打开终端并加载环境

每个新终端都需要执行：

```bash
cd ~/piperx_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

确认当前只识别核心 ROS2 包：

```bash
colcon list --base-paths src --names-only
```

预期输出：

```text
agx_arm_ctrl
agx_arm_description
agx_arm_moveit
agx_arm_msgs
```

## 3. 激活 CAN

进入脚本目录：

```bash
cd ~/piperx_humble_ws/src/agx_arm_ros/scripts
```

如果电脑只连接了一个官方 CAN 模块，执行：

```bash
bash can_activate.sh can0 1000000
```

检查 CAN 口：

```bash
ip link show can0
```

如果想确认是否有 CAN 帧：

```bash
candump can0
```

如果没有 `candump`：

```bash
sudo apt install -y can-utils
```

如果有多个 CAN 模块，先查 USB 端口：

```bash
bash find_all_can_port.sh
```

然后按官方文档 `src/agx_arm_ros/docs/CAN_USER.md` 绑定固定 USB 端口和 CAN 名称，避免重启或换 USB 口后 `can0/can1` 变化。

## 4. 第一次启动基础驱动

先不要启动 MoveIt，只启动机械臂控制节点。

无末端执行器：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
    can_port:=can0 \
    arm_type:=piper_x \
    effector_type:=none \
    speed_percent:=20 \
    tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

带 AgileX 夹爪：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
    can_port:=can0 \
    arm_type:=piper_x \
    effector_type:=agx_gripper \
    speed_percent:=20 \
    tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

说明：

- `speed_percent:=20` 用于第一次低速联调，确认稳定后再提高。
- `auto_enable` 默认是 `true`，启动后会尝试自动使能机械臂。
- 如果只想先看通信和状态，不希望启动时自动使能，可以加 `auto_enable:=false`。

低风险启动示例：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
    can_port:=can0 \
    arm_type:=piper_x \
    effector_type:=none \
    auto_enable:=false \
    speed_percent:=20
```

## 5. 确认反馈话题

另开一个终端：

```bash
cd ~/piperx_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

查看话题：

```bash
ros2 topic list
```

重点确认这些反馈话题：

```text
/feedback/joint_states
/feedback/tcp_pose
/feedback/arm_status
```

查看关节反馈：

```bash
ros2 topic echo /feedback/joint_states
```

如果关节数据持续刷新，说明 ROS2 节点已经能收到机械臂状态。

查看机械臂状态：

```bash
ros2 topic echo /feedback/arm_status
```

如果没有反馈，优先排查：

- CAN 接口名是否正确。
- CAN 波特率是否为 `1000000`。
- 机械臂是否上电。
- CAN 线是否接反。
- 是否使用了官方 CAN 模块。
- `can_port:=can0` 是否和真实接口一致。

## 6. RViz 只显示真实机械臂状态

确认基础反馈正常后，再启动 RViz 显示。

建议第一次使用 `control:=false`，只显示，不从 RViz 发控制：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py \
    can_port:=can0 \
    arm_type:=piper_x \
    effector_type:=none \
    follow:=true \
    control:=false \
    speed_percent:=20 \
    tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

带夹爪：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py \
    can_port:=can0 \
    arm_type:=piper_x \
    effector_type:=agx_gripper \
    follow:=true \
    control:=false \
    speed_percent:=20 \
    tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

参数含义：

- `follow:=true`：RViz 模型跟随 `/feedback/joint_states`。
- `control:=false`：RViz 不发布控制话题。

这一步只用于确认 URDF 模型、关节方向和真实机械臂反馈大致一致。

## 7. MoveIt 仿真规划，不连接真机

在真正控制机械臂前，可以先只启动 MoveIt 模型规划：

```bash
ros2 launch agx_arm_moveit demo.launch.py \
    arm_type:=piper_x \
    effector_type:=none \
    db:=false
```

带夹爪：

```bash
ros2 launch agx_arm_moveit demo.launch.py \
    arm_type:=piper_x \
    effector_type:=agx_gripper \
    db:=false
```

这一步不会启动 `agx_arm_ctrl` 真机控制节点，适合熟悉 RViz MotionPlanning 面板。

## 8. MoveIt + RViz 控制真实机械臂

真机推荐使用一键启动：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
    can_port:=can0 \
    arm_type:=piper_x \
    effector_type:=none \
    speed_percent:=20 \
    auto_control_gate:=true \
    db:=false \
    tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

带夹爪：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
    can_port:=can0 \
    arm_type:=piper_x \
    effector_type:=agx_gripper \
    speed_percent:=20 \
    auto_control_gate:=true \
    db:=false \
    tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

关键参数：

- `follow:=true`：该 launch 默认启用，MoveIt 使用真实反馈作为当前状态。
- `auto_control_gate:=true`：只在 MoveIt 执行轨迹阶段打开控制门控。
- `db:=false`：不启动 MoveIt warehouse 数据库，避免 `warehouse_ros_mongo` 依赖问题。
- `speed_percent:=20`：第一次执行建议低速。

## 9. 在 RViz 中规划和执行

打开 RViz 后，在左侧 MotionPlanning 面板操作：

1. `Planning Group` 选择 `arm`。
2. 在场景中拖动末端 6D 交互标记，设置目标位姿。
3. 或在 `Goal State` 中选择预设状态，例如 `home`。
4. 先点击 `Plan`，只生成轨迹，不执行。
5. 观察轨迹是否合理，确认不会撞桌面、线缆、夹具或自身。
6. 确认安全后，再点击 `Execute` 或 `Plan & Execute`。

第一次真机执行建议：

- 只移动很小距离。
- 不要直接拖到机械臂工作空间边界。
- 不要让末端靠近桌面。
- 不要使用过高速度。
- 手放在急停附近。

如果带夹爪：

- `Planning Group` 可以切换到 `gripper`。
- `Goal State` 可选择 `gripper_open`、`gripper_half`、`gripper_close`。
- 第一次测试夹爪时不要夹硬物或手指，先空载测试。

## 10. 手动门控

如果启动时使用：

```bash
auto_control_gate:=true
```

控制门控服务默认为：

```text
/control_enable
```

手动开门：

```bash
ros2 service call /control_enable std_srvs/srv/SetBool "{data: true}"
```

手动关门：

```bash
ros2 service call /control_enable std_srvs/srv/SetBool "{data: false}"
```

如果使用了命名空间，例如：

```bash
namespace:=arm1
```

服务名通常变为：

```text
/arm1/control_enable
```

## 11. 常用检查命令

查看节点：

```bash
ros2 node list
```

查看话题：

```bash
ros2 topic list
```

查看服务：

```bash
ros2 service list
```

查看真实关节反馈：

```bash
ros2 topic echo /feedback/joint_states
```

查看 MoveIt/ros2_control 输出到驱动的控制话题：

```bash
ros2 topic echo /control/joint_states
```

查看 ros2_control 控制器：

```bash
ros2 control list_controllers
```

查看 `arm_controller` Action：

```bash
ros2 action list | grep follow_joint_trajectory
```

## 12. 常见问题排查

### 没有 `/feedback/joint_states`

优先检查：

- `source install/setup.bash` 是否执行。
- `agx_arm_ctrl` 是否启动成功。
- `can_port` 是否正确。
- CAN 是否激活。
- 机械臂是否上电。
- 终端日志是否有 CAN 打开失败、使能失败或超时。

### `candump can0` 没有数据

优先检查硬件和链路：

- CAN-H/CAN-L 是否接反。
- CAN 模块是否为官方模块。
- 波特率是否为 `1000000`。
- 机械臂电源是否正常。
- USB 口是否松动。

### RViz 模型不动

检查：

```bash
ros2 topic echo /feedback/joint_states
```

如果反馈正常，但 RViz 不动，确认启动参数：

```text
follow:=true
feedback_topic:=feedback/joint_states
```

### MoveIt 可以 Plan，但 Execute 不动

检查：

```bash
ros2 control list_controllers
ros2 topic echo /control/joint_states
ros2 service list | grep control_enable
```

如果使用了 `auto_control_gate:=true`，确认执行阶段门控会打开，或手动测试：

```bash
ros2 service call /control_enable std_srvs/srv/SetBool "{data: true}"
```

### MoveIt 起点状态不匹配

真机控制建议使用：

```text
follow:=true
```

这样 MoveIt 会订阅真实反馈 `/feedback/joint_states`。如果机械臂被手动移动或刚启动，等待几秒让状态刷新后再规划。

### `warehouse_ros_mongo` 缺失

当前工作区建议使用：

```text
db:=false
```

该数据库只影响 MoveIt warehouse，不影响基础规划和真机控制。

## 13. 推荐首次联调命令组合

终端 1：激活 CAN。

```bash
cd ~/piperx_humble_ws/src/agx_arm_ros/scripts
bash can_activate.sh can0 1000000
```

终端 2：启动基础驱动。

```bash
cd ~/piperx_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
    can_port:=can0 \
    arm_type:=piper_x \
    effector_type:=none \
    auto_enable:=false \
    speed_percent:=20
```

终端 3：看反馈。

```bash
cd ~/piperx_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /feedback/joint_states
```

反馈正常后，再关闭基础驱动，改用 MoveIt 一键启动：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
    can_port:=can0 \
    arm_type:=piper_x \
    effector_type:=none \
    speed_percent:=20 \
    auto_control_gate:=true \
    db:=false
```

## 14. 后续开发建议

- 把 `arm_type:=piper_x`、`can_port:=can0`、`effector_type`、`tcp_offset` 这些实际参数记录下来。
- 如果加装工具，先更新 `tcp_offset`，再进行 MoveIt 规划。
- 如果要多机械臂，使用 `namespace` 区分实例。
- 如果要写自己的控制节点，优先通过 ROS2 topic/service/action 接口集成，不要直接绕过安全门控。
- 调试时先看 CAN 和 `/feedback/*`，再看 MoveIt；不要把底层通信问题误判为规划问题。

