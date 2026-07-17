# PiperX + AnyGrasp 智能抓取软件设计方案

本文档记录 PiperX + RGB-D 相机 + AnyGrasp + ROS 2 Humble + MoveIt 2 的正式抓取软件架构。当前仓库中的 `scripts/anygrasp_*` 属于联调验证脚本；后续正式开发应按本文拆包，逐步替换为稳定的 ROS 2 节点、接口和状态机。

## 1. 系统目标

系统目标流程：

```text
相机采集场景
  -> 点云预处理
  -> AnyGrasp 生成 Top-N 抓取候选
  -> 多帧稳定与候选锁定
  -> IK、碰撞和轨迹可行性筛选
  -> PiperX 运动到预抓取点
  -> 二次视觉修正
  -> 直线接近
  -> 夹爪闭合
  -> 抬升
  -> 抓取成功检测
```

模块职责边界：

```text
AnyGrasp：判断哪里适合夹
IK：判断机械臂能不能到
Planning Scene：判断机械臂会不会撞
运动规划：判断怎么安全到
执行状态机：决定什么时候锁定、执行、取消和重试
夹爪反馈：判断到底抓住没有
```

## 2. 总体架构

```text
RGB-D Camera
  -> pointcloud_processor
      -> AnyGrasp 输入点云
      -> 环境碰撞点云 / 物体模型
  -> anygrasp_server
  -> planning_scene_node
  -> grasp_manager
      -> 坐标转换
      -> 多帧聚类和稳定检测
      -> 候选锁定
      -> 综合评分
      -> IK / 碰撞 / 接近路径检查
  -> piper_grasp_mtc
      -> pre-grasp
      -> approach
      -> close
      -> attach
      -> lift
      -> place
  -> MoveIt 2 / PiperX ROS2 Driver
```

控制原则：

- AnyGrasp 候选可以刷新，但执行目标必须先锁定。
- 执行过程中不能让每一帧候选直接修改机械臂轨迹。
- 到达 pre-grasp 后只允许一次受限视觉修正。
- 进入最终直线接近后禁止候选更新。
- 所有真实控制只能由唯一执行节点发送，RViz 只观察。

## 3. ROS 2 包划分

推荐工作空间结构：

```text
piper_anygrasp_ws/src/
├── piper_grasp_bringup/
├── grasp_interfaces/
├── pointcloud_processor/
├── anygrasp_ros/
├── grasp_stabilizer/
├── grasp_evaluator/
├── grasp_scene_manager/
├── piper_grasp_mtc/
├── grasp_task_manager/
├── grasp_safety_monitor/
└── grasp_demo/
```

推荐语言：

```text
AnyGrasp 推理节点：Python
点云处理：C++ 或 Python
候选稳定与筛选：C++
MoveIt / MTC 任务：C++
任务状态机：C++
可视化与调试工具：Python
```

AnyGrasp 的 CUDA / Conda 推理环境应与 MoveIt、PiperX 控制环境隔离，不要把模型推理、规划和真实控制塞进同一个 Python 进程。

## 4. 接口设计

### 4.1 `GraspCandidate.msg`

```text
std_msgs/Header header
uint32 candidate_id
geometry_msgs/Pose pose
geometry_msgs/Vector3 approach
float32 score
float32 width
float32 height
float32 depth
float32 stability_score
float32 planning_score
float32 final_score
bool ik_valid
bool pregrasp_ik_valid
bool approach_valid
bool collision_free
```

约定：

- `header.frame_id` 为 `camera_link`、`camera_color_optical_frame`、`base_link` 或统一世界坐标系。
- `pose` 表示转换后的夹爪 TCP 目标位姿，不是原始 AnyGrasp 坐标。
- `approach` 表示夹爪接近物体的单位向量。
- `width` 表示候选建议的夹爪开口。

### 4.2 `GraspCandidateArray.msg`

```text
std_msgs/Header header
uint32 detection_seq
GraspCandidate[] candidates
```

### 4.3 `GraspTarget.msg`

```text
std_msgs/Header header
uint32 target_id
geometry_msgs/Pose grasp_pose
geometry_msgs/Pose pregrasp_pose
geometry_msgs/Pose lift_pose
float32 gripper_width
float32 confidence
bool locked
```

### 4.4 `ExecuteGrasp.action`

```text
# Goal
GraspTarget target
bool enable_refine
bool enable_place

---
# Result
bool success
uint8 error_code
string message

---
# Feedback
uint8 state
float32 progress
string description
```

### 4.5 `DetectGrasps.srv`

```text
sensor_msgs/PointCloud2 cloud
sensor_msgs/Image target_mask
---
GraspCandidateArray candidates
bool success
string message
```

## 5. TF 和坐标系

推荐坐标树：

```text
world
└── piper_base_link
    └── joints
        └── flange_link
            └── tcp_link

piper_base_link
└── camera_link
    └── camera_color_optical_frame
    └── camera_depth_optical_frame
```

固定外部相机：

```text
T_base_camera = 手眼标定得到的固定变换
```

眼在手上：

```text
T_flange_camera = 眼在手上的手眼标定结果
```

AnyGrasp 到 PiperX TCP：

```text
T_base_graspnet = T_base_camera(t) * T_camera_graspnet
T_base_tcp = T_base_graspnet * T_graspnet_tcp
```

当前实测约定：

- AnyGrasp 接近轴按抓取坐标系 `+X` 处理。
- PiperX 真实夹爪前向按 `tcp_link +Z` 处理。
- 当前轴映射使用 `anygrasp_x_to_tcp_z`。
- 官方夹爪中心补偿默认 `tcp_grasp_offset_z=0.12`，但当前实测可达结果常需要回退到 `0.10`。

## 6. 点云处理

节点：`pointcloud_processor`

输入：

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
/camera/depth/color/points
/tf
/tf_static
```

输出：

```text
/grasp/perception/cloud_raw
/grasp/perception/cloud_filtered
/grasp/perception/object_cloud
/grasp/perception/environment_cloud
/grasp/perception/target_mask
```

处理流程：

```text
RGB-D 同步
  -> 深度有效值过滤
  -> 工作空间裁剪
  -> VoxelGrid 降采样
  -> 离群点滤波
  -> 机器人自体点云过滤
  -> 桌面 / 地面平面分割
  -> 目标物体分割
  -> 输出 AnyGrasp 点云
  -> 输出环境碰撞点云
```

第一版参数：

```yaml
pointcloud:
  voxel_size: 0.005
  min_depth: 0.15
  max_depth: 1.50
  workspace:
    x_min: -0.60
    x_max: 0.60
    y_min: -0.60
    y_max: 0.60
    z_min: 0.00
    z_max: 1.00
  remove_plane: true
  plane_distance_threshold: 0.008
  statistical_filter:
    mean_k: 30
    stddev_mul: 1.0
```

点云要拆成：

```text
object_cloud：送入 AnyGrasp，包含目标物体表面
environment_cloud：送入 Planning Scene，包含桌面、墙面和其他障碍物
```

不要把目标物体和全部原始噪声直接塞进 OctoMap，否则夹爪可能无法接近目标。

## 7. AnyGrasp 节点

节点：`anygrasp_server`

推荐采用服务或 Action 触发推理，而不是相机每来一帧就推理一次。

运行策略：

```text
DETECTING：2-5 Hz
执行到 pre-grasp：停止连续全局推理
REFINING：采集 3-5 帧后重新推理一次
APPROACH 之后：停止更新
```

配置建议：

```yaml
anygrasp:
  top_n: 30
  dense_grasp: false
  collision_detection: true
  use_region_steering: true
  use_approach_steering: true
  max_gripper_width: 0.07
  inference_rate: 3.0
```

当前联调脚本临时使用 Top-50，是为了提高 plan-only 选到可达候选的概率；正式架构应通过稳定、方向约束和评分减少无效候选，而不是无限增大 Top-N。

## 8. 候选稳定与锁定

节点：`grasp_stabilizer`

不要直接使用每一帧 Top-1。推荐缓存最近 5 帧，每帧 Top20-30，总计约 100-150 个候选。

跨帧匹配条件：

```text
位置距离 < 20 mm
姿态距离 < 15 deg
夹爪宽度差 < 10 mm
接近方向夹角 < 15 deg
```

稳定参数：

```yaml
stability:
  window_frames: 5
  required_hits: 4
  position_threshold: 0.020
  orientation_threshold_deg: 15.0
  width_threshold: 0.010
  max_position_std: 0.010
  max_orientation_std_deg: 8.0
```

锁定条件：

```text
最近 5 帧中至少出现 4 帧
位置标准差 < 10 mm
姿态标准差 < 8 deg
IK 初步检查通过
```

迟滞切换：

```text
B.final_score > A.final_score + 0.10
并且连续 3 次更优
才允许替换当前最优候选
```

锁定后生成 `locked_grasp`，执行状态下普通候选不能直接修改它。

## 9. 候选筛选和评分

节点：`grasp_evaluator`

筛选顺序：

```text
夹爪宽度检查
工作空间检查
桌面安全高度检查
接近方向检查
pre-grasp / grasp / lift 关键位姿生成
IK 检查
自碰撞和环境碰撞检查
接近路径采样检查
综合评分
```

关键位姿：

```text
p_pregrasp = p_grasp - d_pre * approach
p_retreat  = p_grasp - d_retreat * approach
p_lift     = p_grasp + d_lift * z_world
```

运动参数：

```yaml
motion:
  pregrasp_distance: 0.10
  approach_distance: 0.10
  retreat_distance: 0.08
  lift_distance: 0.10
```

综合评分：

```text
final_score =
    0.30 * anygrasp_score
  + 0.20 * stability_score
  + 0.15 * direction_score
  + 0.15 * joint_margin_score
  + 0.10 * path_clearance_score
  + 0.10 * minimal_motion_score
```

最终选择稳定、可达、可规划、运动代价合理的候选，而不是单帧 AnyGrasp 最高分。

## 10. Planning Scene

节点：`grasp_scene_manager`

场景对象分类：

```text
固定碰撞物：桌面、底座附近结构、相机支架、墙面、治具
动态环境障碍物：其他物体、未知杂物、动态进入工作空间的障碍物
目标物体：单独 CollisionObject
```

阶段切换：

```text
规划到 pre-grasp：目标物体参与碰撞
直线接近：只允许夹爪指定 link 接触目标
夹爪闭合：允许两侧手指与目标物体接触
确认抓住：目标物体变为 AttachedCollisionObject
抬升和放置：物体作为夹爪一部分参与碰撞检查
```

## 11. MoveIt Task Constructor

节点：`piper_grasp_mtc`

任务阶段：

```text
CurrentState
CheckArmState
OpenGripper
ConnectToPreGrasp
MoveToPreGrasp
AllowGripperObjectCollision
CartesianApproach
CloseGripper
AttachObject
CartesianLift
ConnectToPlace
MoveToPlace
CartesianLower
OpenGripper
DetachObject
CartesianRetreat
ReturnHome
```

规划器分工：

```text
当前位置 -> pre-grasp：OMPL 自由空间规划
pre-grasp -> grasp：笛卡尔直线规划
grasp -> lift：笛卡尔直线规划
lift -> place：OMPL 自由空间规划
```

## 12. 执行状态机

节点：`grasp_task_manager`

状态：

```text
IDLE
DETECTING
STABILIZING
LOCKED
EVALUATING
PLANNING_PREGRASP
EXECUTING_PREGRASP
REFINING
PLANNING_APPROACH
APPROACHING
CLOSING
VERIFYING_GRASP
ATTACHING
LIFTING
PLACING
COMPLETED
RECOVERING
ABORTED
FAULT
```

状态流：

```text
IDLE
  -> DETECTING
  -> STABILIZING
  -> LOCKED
  -> EVALUATING
  -> PLANNING_PREGRASP
  -> EXECUTING_PREGRASP
  -> REFINING
  -> PLANNING_APPROACH
  -> APPROACHING
  -> CLOSING
  -> VERIFYING_GRASP
  -> ATTACHING
  -> LIFTING
  -> PLACING / COMPLETED
```

候选刷新策略：

```text
DETECTING：连续运行，允许修改目标
STABILIZING：连续运行，允许修改目标
LOCKED：可后台检测，不允许普通替换
EVALUATING：可停止，不允许修改
PLANNING_PREGRASP：停止，不允许修改
EXECUTING_PREGRASP：低频检测物体移动，仅允许取消
REFINING：重新检测 3-5 帧，仅允许小范围修正
APPROACHING：停止，禁止修改
CLOSING：停止，禁止修改
LIFTING：停止，禁止修改
```

二次修正：

```yaml
refine:
  enable: true
  frame_count: 5
  max_position_change: 0.025
  max_orientation_change_deg: 10.0
  object_lost_frames: 3
```

超过阈值不追踪，取消当前抓取并退回安全位置重新检测。

## 13. 夹爪控制和成功检测

夹爪策略：

```text
预抓取前：打开至目标宽度 + 安全余量
接近结束：低速闭合
接触后：保持夹持力
抬升后：再次检查状态
```

打开宽度：

```text
open_width = min(max_gripper_width, candidate.width + 0.01 ~ 0.02 m)
```

成功检测：

```text
夹爪没有闭合到空载最小宽度
并且力 / 电流超过接触阈值
并且抬升完成
并且视觉上目标不再留在桌面
```

当前联调阶段可先用：

```text
夹爪位置反馈 + 抬升后视觉确认
```

## 14. 安全监控

节点：`grasp_safety_monitor`

监控内容：

```text
机械臂是否使能
关节状态更新时间
CAN 通信状态
规划轨迹是否过期
实际关节角与目标关节角误差
关节速度和加速度
碰撞场景更新时间
相机数据更新时间
TF 更新时间
夹爪反馈
急停输入
```

立即停止条件：

```text
关节反馈超时
CAN 断开
机械臂异常状态
实际轨迹偏差持续过大
出现新碰撞
目标进入禁区
TF 异常跳变
操作者急停
```

建议阈值：

```yaml
safety:
  joint_state_timeout: 0.20
  camera_timeout: 1.0
  tf_timeout: 0.20
  max_joint_tracking_error: 0.10
  max_cartesian_refine_jump: 0.03
  planning_scene_max_age: 0.50
```

## 15. 推荐配置

```yaml
system:
  base_frame: piper_base_link
  camera_frame: camera_color_optical_frame
  tcp_frame: tcp_link
  planning_group: arm
  gripper_group: gripper

anygrasp:
  detection_rate: 3.0
  top_n: 30
  dense_grasp: false
  collision_detection: true
  max_gripper_width: 0.07

stability:
  window_frames: 5
  required_hits: 4
  position_threshold: 0.020
  orientation_threshold_deg: 15.0
  width_threshold: 0.010
  switch_score_margin: 0.10
  switch_confirm_frames: 3

filter:
  min_grasp_score: 0.35
  min_table_clearance: 0.015
  allow_top_grasp: true
  allow_side_grasp: true
  reject_bottom_grasp: true

motion:
  pregrasp_distance: 0.10
  lift_distance: 0.10
  retreat_distance: 0.08
  free_space_velocity_scale: 0.30
  free_space_acceleration_scale: 0.25
  approach_velocity_scale: 0.10
  approach_acceleration_scale: 0.10
  lift_velocity_scale: 0.15
  lift_acceleration_scale: 0.10

refine:
  enable: true
  frame_count: 5
  max_position_change: 0.025
  max_orientation_change_deg: 10.0

planning:
  max_candidate_evaluations: 20
  ik_timeout: 0.03
  planning_time: 3.0
  planning_attempts: 5
  cartesian_step: 0.005
  min_cartesian_fraction: 0.95

retry:
  max_grasp_attempts: 3
  max_plan_attempts_per_candidate: 2
  retreat_before_retry: true
```

## 16. 启动顺序

```text
1. 激活 CAN
2. 启动 PiperX 驱动
3. 启动 robot_state_publisher 和 TF
4. 启动 MoveIt 2
5. 启动 RGB-D 相机
6. 启动点云处理
7. 启动 Planning Scene
8. 启动 AnyGrasp 推理服务
9. 启动候选稳定和筛选节点
10. 启动 MTC 抓取节点
11. 启动任务状态机
12. 启动安全监控
13. 启动 RViz
```

PiperX 启动示意：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
    can_port:=can0 \
    arm_type:=piper_x \
    effector_type:=agx_gripper \
    follow:=true \
    control:=false \
    tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

实际 `tcp_offset` 必须替换为夹爪指尖中心的标定值。

## 17. 分阶段开发计划

### 阶段 A：PiperX 运动控制

验收：

```text
关节反馈正确
URDF 与真机一致
TCP 正确
MoveIt 规划和执行正确
夹爪开合正确
```

### 阶段 B：相机和坐标标定

验收：

```text
点云在 base_link 下位置正确
已知物体视觉位置误差 < 10 mm
TCP 到物体的实际误差可重复
```

### 阶段 C：AnyGrasp 离线验证

验收：

```text
点云能够正常推理
候选坐标方向正确
夹爪宽度正确
Open3D / RViz 显示正确
```

### 阶段 D：候选稳定锁定

验收：

```text
静态物体候选不会每帧切换
Top1 变化时锁定目标仍保持稳定
物体移动超过阈值可以解除锁定
```

### 阶段 E：规划与碰撞

验收：

```text
桌面不会被机械臂穿过
pre-grasp、grasp 和 lift 均有完整检查
不可达候选会自动切换到下一候选
```

### 阶段 F：真实抓取

验收：

```text
运动到 pre-grasp
二次识别
直线接近
闭合夹爪
抬升
失败时安全退回
```

### 阶段 G：可靠性提升

增加：

```text
目标实例分割
OctoMap
抓取成功检测
失败重试
放置规划
动态场景变化检测
日志与数据回放
```

## 18. 第一阶段最小可运行版本

第一阶段先实现：

```text
固定相机
固定桌面
单个静态物体
人工指定 ROI
AnyGrasp Top-N
多帧稳定锁定
pre-grasp 和 grasp IK
桌面碰撞模型
规划到 pre-grasp
直线接近
夹爪闭合
垂直抬升
人工确认是否成功
```

暂不实现：

```text
复杂实例分割
动态物体跟踪
全场景 OctoMap
自动放置
抓取失败视觉判断
在线伺服追踪
```

## 19. 当前仓库落地建议

当前仓库已经有一套验证脚本：

```text
scripts/anygrasp_ros_infer.py
scripts/anygrasp_safe_pick.py
scripts/anygrasp_apply_scene.py
scripts/anygrasp_full_pipeline.py
scripts/anygrasp_roi_selector.py
```

第一阶段已开始落地：

```text
src/grasp_interfaces
src/grasp_task_manager
```

当前状态：

```text
grasp_interfaces：
  已定义 GraspCandidate、GraspCandidateArray、GraspTarget、DetectGrasps、ExecuteGrasp。

grasp_task_manager：
  已定义第一版纯状态机，覆盖 IDLE、DETECTING、LOCKED、EXECUTING_PREGRASP、
  APPROACHING、CLOSING、VERIFYING_GRASP、LIFTING、COMPLETED、RECOVERING、
  ABORTED 和 FAULT。
```

下一步落地顺序：

```text
1. 给 grasp_task_manager 接入 ExecuteGrasp action server。
2. 接入 frozen candidate 文件和 offset 扫描参数。
3. 将 scripts/anygrasp_full_pipeline.py 封装成可替换的 plan/execute 后端。
4. 再新增 piper_grasp_mtc，用 MTC 替代脚本式多段轨迹。
```

保留当前脚本作为回归工具和硬件调试入口。正式节点应逐步替换脚本中的逻辑，而不是在单个 Python 脚本里继续堆复杂状态。

## 20. 参考资料

- AnyGrasp SDK usage: https://github.com/graspnet/anygrasp_sdk/blob/main/grasp_detection/USAGE.md
- MoveIt Planning Scene Monitor: https://moveit.picknik.ai/main/doc/concepts/planning_scene_monitor.html
- AgileX Arm ROS: https://github.com/agilexrobotics/agx_arm_ros
- MoveIt Perception Pipeline: https://moveit.picknik.ai/humble/doc/examples/perception_pipeline/perception_pipeline_tutorial.html
- MoveIt Task Constructor: https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor/moveit_task_constructor.html
