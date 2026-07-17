# PiperX RViz 和 MoveIt 测试流程

本文档记录 `/home/joey/piperx_humble_ws` 工作区中 PiperX 机械臂的 CAN 接入、反馈检查、RViz 跟随、MoveIt IK 规划与执行测试流程。

```bash
/home/joey/piperx_humble_ws
```

## 硬件环境

- 机械臂：AgileX PiperX
- 末端执行器：AgileX 夹爪
- CAN 适配器：USB-CAN，走 SocketCAN
- CAN 驱动：`gs_usb`
- CAN 接口：`can0`
- CAN 波特率：`1000000`
- ROS 版本：ROS 2 Humble

## 安全注意事项

当前设备已经观察到：按下急停后机械臂会突然下坠，并可能剐蹭周围物体。这个问题必须按严重安全风险处理。

- 不要把急停当作日常停止按钮使用。
- 急停只用于真实危险场景。
- 测试前清空机械臂工作空间，避免下坠路径上有设备、桌面边缘或人员。
- 测试前尽量让机械臂处于低风险姿态，例如重心靠近底座、末端不要大幅悬伸。
- 第一次测试使用低速，例如 `speed_percent:=5` 或 `speed_percent:=10`。
- 第一次只测试小幅运动。
- 初次联调时不要同时测试机械臂运动和夹爪动作。
- 确认夹爪负载和末端负载没有超限。
- 检查硬件是否带抱闸，以及急停时是切伺服扭矩、切抱闸供电，还是两者都会切。

正常停机应优先使用软件停止、规划到安全姿态、再禁用机械臂。急停可能直接切掉扭矩，导致重力下坠，不能靠 MoveIt 规划解决。

## 1. 加载工作区环境

每打开一个新终端，都需要先执行：

```bash
cd /home/joey/piperx_humble_ws
source install/setup.bash
```

如果没有 source 当前工作区，ROS 只会搜索 `/opt/ros/humble`，会出现：

```text
Package 'agx_arm_ctrl' not found
```

## 2. 激活 CAN

`can_activate.sh` 的参数顺序是：

```bash
bash can_activate.sh [CAN接口名] [波特率]
```

正确命令：

```bash
cd /home/joey/piperx_humble_ws/src/agx_arm_ros/scripts
bash can_activate.sh can0 1000000
```

不要执行下面这种命令：

```bash
bash can_activate.sh 1000000
```

这样会把 `1000000` 当作接口名，导致 `can0` 被重命名成 `1000000`。

检查 CAN 接口：

```bash
ip -details link show can0
```

期望结果：

- 能看到 `can0`
- CAN 状态为活动状态，例如 `ERROR-ACTIVE`
- bitrate 为 `1000000`

可选：监听 CAN 总线：

```bash
candump can0
```

## 3. 反馈只读测试

先启动驱动，但不自动使能，也不接收控制命令：

```bash
cd /home/joey/piperx_humble_ws
source install/setup.bash

ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
    can_port:=can0 \
    arm_type:=piper_x \
    effector_type:=agx_gripper \
    auto_enable:=false \
    control_enabled:=false
```

期望日志：

```text
can_port: can0
arm_type: piper_x
effector_type: agx_gripper
firmware version: ...
Agx_arm feedback is ready
```

另开一个终端检查话题：

```bash
cd /home/joey/piperx_humble_ws
source install/setup.bash

ros2 topic list | grep -E "feedback|joint|gripper|hand"
ros2 topic echo /feedback/joint_states --once
```

如果反馈正常，说明 USB-CAN、`can0`、驱动、机械臂反馈链路已经打通。

## 4. RViz 只跟随真实机械臂

如果只想让 RViz 显示真实机械臂状态，不发送控制命令，使用：

```bash
cd /home/joey/piperx_humble_ws
source install/setup.bash

ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py \
    can_port:=can0 \
    arm_type:=piper_x \
    effector_type:=agx_gripper \
    auto_enable:=false \
    control_enabled:=false \
    follow:=true \
    control:=false
```

期望现象：

- RViz 模型跟随真实机械臂。
- RViz 不发布控制命令。
- 适合检查反馈、模型方向、关节映射是否正确。

## 5. MoveIt IK 规划和执行测试

如果需要 RViz + MoveIt，进行 IK plan 和 execute，使用：

```bash
cd /home/joey/piperx_humble_ws
source install/setup.bash

ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
    can_port:=can0 \
    arm_type:=piper_x \
    effector_type:=agx_gripper \
    auto_enable:=true \
    control_enabled:=true \
    speed_percent:=10
```

首次测试顺序：

1. 确认机械臂已经使能，反馈持续刷新。
2. 在 RViz 中轻微移动末端目标位姿。
3. 先点击 `Plan`。
4. 检查规划轨迹是否合理，是否有大幅绕行或突跳。
5. 轨迹确认正常后，再点击 `Execute`。
6. 第一次运动幅度建议控制在 2 cm 到 3 cm。

不要一开始就把末端目标拖到很远的位置。

## 6. 常用诊断命令

查看 controller 状态：

```bash
ros2 control list_controllers
```

查看相关话题：

```bash
ros2 topic list | grep -E "joint|feedback|control|trajectory|gripper|hand"
```

查看相关服务：

```bash
ros2 service list | grep -E "enable|controller|move_group|execute"
```

查看一次关节反馈：

```bash
ros2 topic echo /feedback/joint_states --once
```

## 7. 常见问题

### 找不到 agx_arm_ctrl 包

现象：

```text
Package 'agx_arm_ctrl' not found
```

原因：

当前终端没有 source 工作区。

处理：

```bash
cd /home/joey/piperx_humble_ws
source install/setup.bash
```

### can0 不存在

现象：

```text
Device "can0" does not exist.
SIOCGIFINDEX: No such device
```

可能原因：

CAN 激活脚本参数传错，例如：

```bash
bash can_activate.sh 1000000
```

这会把 `can0` 重命名成 `1000000`。

处理：

```bash
cd /home/joey/piperx_humble_ws/src/agx_arm_ros/scripts
bash can_activate.sh can0 1000000
```

### 急停后机械臂下坠

现象：

按下急停后机械臂突然下坠，可能剐蹭桌面、设备或夹具。

可能原因：

急停切掉了伺服扭矩或驱动供电，机械臂无法保持当前位置。如果设备没有抱闸，或者抱闸没有及时闭合，关节会在重力和末端负载作用下运动。

处理建议：

- 不要把急停作为常规停止方式。
- 调试前把机械臂放在下坠也不会碰撞的位置。
- 降低测试速度，先做小幅运动。
- 增加物理避让、软垫、限位或机械支撑。
- 检查机械臂是否带抱闸，以及急停时抱闸逻辑是否正确。
- 正常结束测试时，优先通过软件停止或规划到安全姿态，再禁用机械臂。

