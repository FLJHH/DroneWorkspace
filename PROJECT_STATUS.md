# 无人机视觉自主降落项目现状盘点

> 检查日期：2026-08-22（已加入本阶段实测）  
> 检查范围：`vehicle_landing_demo` 当前代码、默认启动配置、模型/config、训练结果，以及 5 次静态正式实验和 0.5 m/s vehicle-only 实验。  
> 判定原则：代码存在不等于功能已验证；默认 `demo.launch` 及现存日志优先于文件名、注释和历史设想。

## 1. 项目目标

最终目标是在 ROS Noetic、Gazebo、PX4 SITL 和 MAVROS 环境中，让无人机通过视觉识别降落平台/车辆，自主完成发现、跟踪、对准、下降、触地和锁桨，并逐步扩展到目标丢失恢复及强化学习控制。

- **当前已实现目标**：默认静态车辆平台场景下，YOLO 确认车辆、AprilTag 估计落点、速度闭环下降、触地判断及自动 disarm 的完整链路已用同参数正式重复 5 次并全部成功；0.5 m/s UGV 直线物理运动已单独验证。
- **当前研究目标**：下一阶段建立固定高度 `TRACK` 移动目标跟踪。移动平台自主降落仍未实现，也未在本阶段尝试。
- **后续计划目标**：稳定移动平台跟踪与降落 → 完善 YOLO/AprilTag 分阶段融合 → 完整丢失恢复 → 建立传统控制 baseline → 接入 PPO 等强化学习方法。

## 2. 当前实际完成状态

| 模块/能力 | 状态 | 实际判断与证据 |
|---|---|---|
| Gazebo 仿真环境 | ✅ 已实现并验证 | `demo.launch` 启动 `empty_world.launch`、道路 world、UGV 和 Iris；现存 Gazebo/ROS 日志完整运行。 |
| PX4 SITL | ✅ 已实现并验证 | 引用 PX4 `single_vehicle_spawn_xtd.launch`；日志显示 FCU 连接、OFFBOARD 和 armed。 |
| MAVROS | ✅ 已实现并验证 | 启动 `mavros/launch/px4.launch`，控制器实际收发 MAVROS topic/service。 |
| 无人机起飞 | ✅ 已实现并验证 | `TAKEOFF` 速度闭环升至 25 m，日志进入 `TRANSIT`。 |
| OFFBOARD 模式 | ✅ 已实现并验证 | 通过 `/iris_0/mavros/set_mode` 请求；日志出现 `PX4 OFFBOARD accepted`。 |
| AprilTag 检测 | ✅ 已实现并验证 | `apriltag_ros_continuous_node` 运行，日志在末段成功 acquired。 |
| AprilTag 相对位置估计 | ✅ 已实现并验证 | 控制器能查询 `iris_0/base_link -> landing_pad` TF，并进一步查询 `map -> landing_pad`。 |
| 静态平台定位 | ✅ 已实现并验证 | 默认平台 `speed=0.0`；日志得到连续 `target=(x,y)`。 |
| 静态平台自主降落 | ✅ 已实现并验证 | 同一 baseline 参数正式运行 5 次，5/5 到达 touchdown、disarm、`DONE`；详见 `experiments/static_baseline.csv`。样本仍小且为同工况。 |
| 静态平台重复性测试 | ✅ 已实现并验证 | 5 次成功率 100%；平均起飞至触地 58.651 s，平均触地 GT 水平误差 0.341 m。 |
| 自动触地判断 | ✅ 已实现并验证 | 代码按 Gazebo 真值高度和水平误差判断，日志触发 touchdown。注意这不是纯机载传感器触地判断。 |
| 自动 disarm | ✅ 已实现并验证 | MAV_CMD_COMPONENT_ARM_DISARM（command 400）强制 disarm；日志记录 `motors locked`。 |
| 移动平台模型/驱动 | ✅ 已实现并验证（0.5 m/s 直线） | vehicle-only 稳定段平均 0.499999933 m/s、标准差 0.000002023 m/s，无明显横漂/翻车；默认 `demo.launch` 仍为 0.0 m/s。 |
| 0.5 m/s 移动平台运动 | ✅ 已实现并验证 | 45 s、864 样本；稳定段超过 30 s，数据见 `experiments/vehicle_motion_0p5.csv`。仅证明车辆运动。 |
| 移动目标跟踪 | ❌ 尚未实现 | 当前状态机仍无独立 TRACK；本阶段没有让无人机跟车或下降。已有真值速度前馈代码不等于已实现跟踪。 |
| 移动平台降落 | ❌ 尚未实现 | 截至本次检查，无当前可复核的、默认配置或多次测试证明移动平台降落成功；用户给定现状也明确尚未实现。 |
| YOLO 车辆检测 | ✅ 已实现并验证 | 静态链路已验证；0.5 m/s 固定相机测试在视场内检出，置信度 0.330～0.734。视场外丢失不算网络异常。 |
| YOLO → AprilTag 切换 | ✅ 已实现并验证 | 高度≤6 m、水平≤3 m且 Tag 有效时切换；现存日志在 2.87 m/3.00 m 触发。 |
| AprilTag 丢失处理 | 🟡 已有代码但未充分验证 | 有 `TAG_DEAD_RECKON`、3 s 超时后悬停；单次日志发生近地丢码并靠短时冻结目标完成，但没有重新搜索/扩圈/复飞等完整恢复。 |
| 强化学习 | ❌ 尚未实现 | 未发现环境接口、reward、policy、训练或推理节点。 |

## 3. 工程目录结构

```text
DroneWorkspace/
├── PROJECT_STATUS.md                         # 本现状盘点
├── ARCHITECTURE.md                           # 顶层架构说明
└── vehicle_landing_demo/                     # ROS/catkin 功能包
    ├── launch/
    │   ├── demo.launch                       # 默认完整仿真入口
    │   └── px4_isolation.launch              # PX4 启动隔离辅助配置
    ├── scripts/
    │   ├── vehicle_precision_landing.py      # 状态机、坐标变换、速度控制、触地/disarm
    │   ├── vehicle_yolo.py                   # YOLO 推理、检测框过滤与发布
    │   ├── moving_vehicle.py                 # Gazebo UGV 物理力驱动原型
    │   ├── sdf_tilt_camera.py                # 生成前下方倾斜 45° 的 Iris 相机 SDF
    │   ├── sdf_scale_tag.py                  # 启动时将几何 Tag 缩放到配置尺寸
    │   └── check_environment.sh              # 依赖环境诊断
    ├── config/
    │   ├── apriltag_settings.yaml            # AprilTag 检测器参数
    │   └── apriltag_tags.yaml                # Tag ID、尺寸和 TF 名称
    ├── models/
    │   ├── tagged_vehicle/model.sdf          # SUV、车顶碰撞面和几何 AprilTag
    │   └── monocular_camera/monocular_camera.sdf # 1280×720、30 Hz Gazebo 相机
    ├── worlds/yolo_road_stable.world          # 默认道路/建筑/行人场景
    ├── training/vehicle_yolo11s_descent/      # 当前 YOLO 权重和训练结果
    ├── .sim_ros_logs/                         # 当前可核查运行日志
    └── run_demo.sh                            # 环境装载和一键 roslaunch
```

## 4. 项目启动流程

一键入口：

```bash
cd /home/fenglijun/DroneWorkspace/vehicle_landing_demo
./run_demo.sh gui:=true visualize_yolo:=true auto_start:=true use_cpu:=true
```

无界面静态基线可用：

```bash
./run_demo.sh gui:=false visualize_yolo:=false
```

`run_demo.sh` 检查 ROS/catkin/PX4 SITL，设置 ROS、Gazebo、PX4/XTDrone 路径，最后执行：

```text
run_demo.sh
↓ roslaunch vehicle_landing_demo demo.launch
Gazebo empty_world + yolo_road_stable.world
↓
spawn ugv_0（默认 speed=0.0）
↓
PX4 SITL + iris_downward_camera（实际改为前下方 45°）
↓
MAVROS（iris_0 namespace）
↓
Gazebo camera → apriltag_ros + vehicle_yolo.py
↓
vehicle_precision_landing.py
↓ PositionTarget 速度型 raw local setpoint
/iris_0/mavros/setpoint_raw/local
↓
MAVROS ENU/FLU → PX4 NED/FRD 转换
↓
PX4 OFFBOARD → UAV
```

默认关键 launch 参数：`gui=true`、`visualize_yolo=true`、`auto_start=true`、`use_cpu=true`；平台驱动 `speed=0.0`；巡航高度 25 m；下降速度 0.8 m/s；`demo_ground_truth=true`。

## 5. ROS 节点关系

### 节点：`/gazebo`

功能：运行世界物理、相机传感器、模型状态和施力服务。

订阅/服务输入：`/gazebo/apply_body_wrench` 服务调用。  
发布：`/gazebo/model_states`、`/iris_0/camera/image_raw`、`/iris_0/camera/camera_info` 等。

### 节点：`/moving_landing_vehicle`

功能：对 `ugv_0::body` 施力以跟踪设定纵向速度和道路中心线；默认速度为 0，因此当前是静态保持。

订阅：`/gazebo/model_states`。  
调用：`/gazebo/apply_body_wrench`。

### 节点：`/iris_0/apriltag`

功能：`apriltag_ros/apriltag_ros_continuous_node`，由相机图像估计 Tag 36h11 ID 0 位姿并发布 TF/检测消息。

订阅：namespace/remap 后的 `/iris_0/camera/image_raw`、`/iris_0/camera/camera_info`。  
发布：`/iris_0/tag_detections`（`apriltag_ros/AprilTagDetectionArray`，标准节点接口）、`/tf`，以及检测图像 topic（实际完整名称待运行时 `rostopic list` 确认）。

### 节点：`/road_yolo`

功能：运行 Ultralytics YOLO，筛选最大有效车辆框和 person 框。

订阅：`/iris_0/camera/image_raw`、`/gazebo/model_states`。  
发布：`/landing/yolo_boxes`（`yolov11_ros_msgs/BoundingBoxes`）、`/yolov11/detection_image`（`sensor_msgs/Image`）。

### 节点：`/vehicle_precision_landing`

功能：主状态机、OFFBOARD/arm、斜向接近、AprilTag 世界坐标滤波、速度控制、触地及 disarm。

订阅：

- `/iris_0/mavros/state`（`mavros_msgs/State`）
- `/iris_0/mavros/local_position/pose`（`geometry_msgs/PoseStamped`）
- `/iris_0/mavros/extended_state`（`mavros_msgs/ExtendedState`）
- `/landing/yolo_boxes`（`yolov11_ros_msgs/BoundingBoxes`）
- `/gazebo/model_states`（`gazebo_msgs/ModelStates`）
- `/tf`、`/tf_static`（通过 tf2 listener）

发布：`/iris_0/mavros/setpoint_raw/local`（`mavros_msgs/PositionTarget`）。  
调用：`/iris_0/mavros/set_mode`、`/iris_0/mavros/cmd/arming`、`/iris_0/mavros/cmd/command`。

### 节点：`/iris_0/mavros`

功能：ROS ENU/FLU 与 PX4 NED/FRD 的桥接、状态回传、模式/解锁服务和 setpoint 转发。

订阅：`/iris_0/mavros/setpoint_raw/local`。  
发布：state、local position、extended state 等。

## 6. ROS Topic 数据流

### 相机图像

Gazebo `libgazebo_ros_camera.so` 生成 1280×720、30 Hz 图像和相机内参。实际图像 topic 是 `/iris_0/camera/image_raw`，相机参数是 `/iris_0/camera/camera_info`；二者同时送入 AprilTag，原图也送入 YOLO。

### AprilTag 检测结果

- 检测消息：`/iris_0/tag_detections`，类型为 `apriltag_ros/AprilTagDetectionArray`。
- 位姿链：检测器以配置的 `iris_0/camera_link` 为相机 frame，并发布到名为 `landing_pad` 的 Tag TF。
- 控制器没有直接订阅 detection array；它通过 tf2 查询 `iris_0/base_link -> landing_pad` 判断 Tag 是否新鲜，再查询 `map -> landing_pad` 得到控制目标的平面世界坐标。

### 无人机当前位置

主要来自 `/iris_0/mavros/local_position/pose`。但平台位置、平台速度、相对高度/触地条件还直接依赖 `/gazebo/model_states` 中的 `ugv_0` 和 `iris_0` 真值，因此当前不是纯视觉/纯飞控状态估计系统。

### 控制指令

当前使用 **raw local velocity setpoint**：

- topic：`/iris_0/mavros/setpoint_raw/local`
- 消息：`mavros_msgs/PositionTarget`
- mask：忽略位置、加速度、yaw 和 yaw rate，仅填写 `velocity.x/y/z`
- frame：根据控制阶段使用 `FRAME_LOCAL_NED` 或 `FRAME_BODY_NED` 常量；传入字段按 ROS 侧 ENU/FLU 语义，由 MAVROS 转成 PX4 NED/FRD。

不是 position setpoint、attitude setpoint，也不是直接电机控制。

## 7. 坐标系

| 坐标系 | 当前语义 |
|---|---|
| Gazebo `world` | 仿真真值世界系；代码按 x/y 水平、z 向上使用，可视为 ENU 风格，但 world 原点与 MAVROS local 原点不必相同。 |
| MAVROS `map` / local | ROS 本地 ENU：x 东/前述世界 x，y 北/世界 y，z 向上。控制器 `pose` 和 `tag_world` 在该平面坐标中运算。 |
| PX4 local | PX4 内部 NED：x 北、y 东、z 向下；MAVROS负责与 ROS ENU 转换。 |
| `iris_0/base_link` | ROS 机体 FLU：x 前、y 左、z 上。 |
| `iris_0/base_link_frd` | 静态 TF 辅助 frame：x 前、y 右、z 下。 |
| `iris_0/camera_link` | AprilTag 检测相机 frame；按光学约定 x 向右、y 向下、z 沿镜头前方。 |
| `landing_pad` | Tag ID 0 的目标 frame，几何中心是降落目标。 |

问题回答：

1. AprilTag 位姿最初由 `apriltag_ros` 从相机图像中在 `iris_0/camera_link` 相关坐标链上估计。
2. 有坐标变换：launch 发布 `base_link -> camera_link` 静态外参；AprilTag 发布相机到 Tag 的 TF；tf2 组合得到 `base_link -> landing_pad` 和 `map -> landing_pad`。
3. 最终水平控制使用 MAVROS `map`/local ENU 中的 `tag_world(x,y)` 与无人机 local pose 之差；近地 clearance 当前主要使用 Gazebo world z 真值。
4. ENU 控制量为 x/y 水平、z 向上；机体系为前/左/上；相机光学系为右/下/前；PX4 内部为北/东/下。
5. ENU/NED 转换发生在 MAVROS；代码发布消息时使用 `PositionTarget` 的 NED frame 枚举，但字段按 ROS ENU/FLU 填写，这是 MAVROS `setpoint_raw/local` 的接口约定。
6. 相机不是严格朝下；它从原始下视安装改为距 nadir 倾斜 45°、朝前下方。
7. 模型安装姿态由 `scripts/sdf_tilt_camera.py` 写入相机 include pose（pitch `0.7853981634`），TF 外参由 `launch/demo.launch` 中 `iris_camera_tf` 定义。

相机具体光轴相对 Gazebo 世界方向会随无人机完整姿态变化；上述静态外参欧拉角解释依赖 ROS Noetic 旧版 `static_transform_publisher` 参数约定，若跨版本移植应实测确认。

## 8. AprilTag 系统

- 包/节点：`apriltag_ros` / `apriltag_ros_continuous_node`。
- family：`tag36h11`。
- ID：`0`。
- 检测配置尺寸：`0.6 m`。
- TF 名称：`landing_pad`。
- Gazebo 模型：`vehicle_landing_demo/models/tagged_vehicle/model.sdf`，Tag 由黑色 box 几何拼成，不依赖贴图。
- 启动缩放：`sdf_scale_tag.py ... 0.375` 将原约 1.6 m 图案缩为约 0.6 m；与检测配置一致。
- 位置：Tag 中心位于车辆 link 原点平面中心附近，z≈2.231 m；白色视觉板 z=2.20 m，物理降落碰撞面尺寸 1.8×1.8×0.06 m。
- 当前平台模型：非 static 的 `ugv_0` SUV 刚体，但默认驱动目标速度为 0，因此验证时是静态平台。
- Tag 与平台关系：Tag 视觉几何与车体在同一个 `body` link 中，刚性固定在车顶平台中心。
- 检测距离：当前成功日志仅确认在离车顶约 2.87 m、水平约 3.00 m 时完成控制交接；Tag 更早是否稳定可见、最大可靠检测距离没有系统统计。
- 检测丢失：确认存在。成功日志在 clearance 约 0.75 m 后由 `TAG_APPROACH` 转为 `TAG_DEAD_RECKON`，随后短时冻结目标完成触地。

## 9. 当前静态自主降落控制逻辑

真实默认状态机：

```text
WAIT_FCU
  ↓ auto_start 且等待 2.5 s
TAKEOFF
  ↓ 高度 > cruise_height - 0.25
TRANSIT
  ↓ 到后侧方观察点，水平误差 < 0.45 m
ACQUIRE
  ↓ YOLO 已确认 + 高度≤6 m + 水平≤3 m + Tag 可用
TAG_APPROACH
  ↔ Tag 丢失/重新获得
TAG_DEAD_RECKON
  ↓ 触地条件成立且 disarm 成功
DONE
```

另有 `AUTO_LAND` 分支和 `request_land()`，但当前主路径没有调用 `request_land()`，不是默认成功链路。

### 水平控制

- `TAKEOFF/TRANSIT`：ENU P 控制，`v = clamp(kp_transit * position_error)`；`kp_transit` 默认 0.7，launch 未覆盖。
- `ACQUIRE` 且 `demo_ground_truth=true`：以 Gazebo 真值车辆位置构造随高度收缩的斜向路径，采用平台速度/路径收缩速度前馈 + `kp_glide * (ex,ey)` 的 P 控制，限速 2.5 m/s。
- 无真值 fallback：YOLO 图像归一化误差，经 deadband 后用 `kp_yolo=0.9` 生成 body-FLU 平面速度。
- `TAG_APPROACH/TAG_DEAD_RECKON`：不是固定增益 PID。以预计剩余下降时间 `max(0.35, clearance/descent_speed)` 为时域，计算 `vx=ex/time_to_touchdown`、`vy=ey/time_to_touchdown`，再限幅。这是随高度变化的比例/有限时间收敛控制，无积分和微分项。

### 高度控制

- 起飞：`vz=clamp(kp_transit*(25-p.z), 1.2)`。
- 斜向下降：仅当路径误差 `< glide_tolerance=1.0 m` 才以 `-0.8 m/s` 下降；低于最终对准高度且误差超阈值时暂停下降；检测到 person 时暂停。
- Tag 阶段：只要 clearance > 0.05 m，就固定以 `-0.8 m/s` 下降；若 Tag 丢失超过 3 s 且仍高于平台 0.45 m，则三轴保持。
- 没有独立可配置的“最低下降速度”；代码中的 `0.05` 仅用于防止除零/末端门限。

### 降落目标点

Tag 阶段最终对准的是 TF `landing_pad` 的 **AprilTag 几何中心在 map 中的滤波 x/y**。默认 `landing_lead_time=0`、`landing_forward_offset=0`，因此没有最终前向偏置。可见期以一阶滤波更新；失去 Tag 后冻结最后位置。

### 触地判断

Tag 阶段 `finish_platform_touchdown()` 同时要求：

- Gazebo 中 Iris 世界高度 `< platform_height + 0.37`（当前即 `<2.60 m`）；
- MAVROS local 平面位置到保存 Tag 位置 `<0.35 m`。

这是依赖 Gazebo 真值的仿真专用判断，不是 PX4 landed state、测距或接触传感器判断。ACQUIRE 真值路径还存在相似的 `<0.45 m` / `<2.60 m` 直接 disarm 分支。

### Disarm

会自动 disarm。主路径通过 `/iris_0/mavros/cmd/command` 发送 MAVLink command 400，`param1=0`、`param2=21196`，即强制锁桨；服务成功后进入 `DONE`。日志已经确认一次成功。

## 10. 所有与降落有关的关键参数

以下是默认 `demo.launch` 的实际值；“代码默认”表示 launch 未显式覆盖。

| 参数 | 当前值 | 单位 | 作用 | 文件位置 |
|---|---:|---|---|---|
| `auto_start` | `true` | bool | FCU 就绪后自动进入 TAKEOFF | `launch/demo.launch:3,84` |
| `cruise_height` | 25.0 | m | 起飞/观察巡航高度 | `launch/demo.launch:85` |
| `staging_x/y` | 9.45 / -0.5 | m | 无真值时固定集结点 fallback | `launch/demo.launch:85` |
| `kp_transit` | 0.7 | s⁻¹ | 起飞/转场 P 增益（代码默认） | `vehicle_precision_landing.py:26` |
| `kp_glide` | 2.0 | s⁻¹ | 斜向真值路径 P 增益 | `demo.launch:90` |
| `kp_yolo` | 0.9 | 比例 | 图像误差到 body 速度增益 | `demo.launch:89` |
| `kp_align` | 0.8 | s⁻¹ | 代码参数，但当前主路径未使用 | `vehicle_precision_landing.py:28` |
| `max_xy_speed` | 2.5 | m/s | ENU 水平指令限幅 | `demo.launch:91` |
| YOLO body 限速 | 1.0 | m/s | `yolo_track_velocity()` 内硬编码限幅 | `vehicle_precision_landing.py:194` |
| 起飞最大竖速 | 1.2 | m/s | TAKEOFF 内硬编码限幅 | `vehicle_precision_landing.py:349` |
| `descent_speed` | 0.80 | m/s | 斜向及 Tag 阶段下降速度 | `demo.launch:105` |
| `glide_tolerance` | 1.0 | m | 允许斜向下降的路径误差 | `demo.launch:101` |
| `final_align_height` | 1.20 | m | 低高度最终对准门槛 | `demo.launch:95` |
| `final_align_tolerance` | 0.20 | m | 低高度暂停下降的误差门槛 | `demo.launch:96` |
| `align_tolerance` | 0.18 | m | 代码默认参数，当前主路径未使用 | `vehicle_precision_landing.py:31` |
| `tag_handoff_height` | 6.0 | m | YOLO→Tag 的高度条件 | `demo.launch:102` |
| `tag_handoff_horizontal` | 3.0 | m | YOLO→Tag 的水平条件 | `demo.launch:103` |
| `vehicle_timeout` | 1.5 | s | 最近车辆检测有效期 | `demo.launch:86` |
| `yolo_deadband` | 0.08 | 归一化图像尺度 | YOLO 图像误差死区（代码默认） | `vehicle_precision_landing.py:46` |
| `tag_filter_alpha` | 0.25 | 比例 | Tag map 坐标一阶滤波权重 | `demo.launch:106` |
| `tag_max_jump` | 0.75 | m | 拒绝单次 Tag 世界坐标跳变 | `demo.launch:107` |
| Tag TF 新鲜度 | 0.6 | s | `tag_transform/update_tag_world` 硬编码 | `vehicle_precision_landing.py:236,253` |
| `dead_reckon_timeout` | 3.0 | s | 丢码冻结目标最大可信时间 | `demo.launch:108` |
| `platform_height` | 2.23 | m | 车顶/Tag 高度基准 | `demo.launch:100` |
| Tag 触地水平阈值 | 0.35 | m | `finish_platform_touchdown` 默认值 | `vehicle_precision_landing.py:310` |
| Tag 触地高度余量 | 0.37 | m | Iris world z 相对平台门限 | `vehicle_precision_landing.py:314` |
| `demo_ground_truth` | `true` | bool | 转场、路径、平台速度、高度使用 Gazebo 真值 | `demo.launch:97` |
| `observation_back/side` | 23.0 / -6.0 | m | 后侧方观察与斜向路径偏移 | `demo.launch:98-99` |
| `landing_lead_time` | 0.0 | s | 平台速度预测前视时间 | `demo.launch:93` |
| `landing_forward_offset` | 0.0 | m | 沿平台运动方向的落点偏置 | `demo.launch:94` |
| `person_pause` | `true` | bool | 人员可见时暂停下降 | `demo.launch:87` |
| UGV `speed` | **0.0** | m/s | 当前默认平台静止 | `demo.launch:26` |
| UGV `start_delay` | 38.0 | s | 平台驱动启动延时；静态配置下无实际运动 | `demo.launch:27` |
| YOLO `conf` | 0.3 | 概率 | 检测阈值 | `demo.launch:73` |
| YOLO `imgsz` | 1280 | pixel | 推理输入尺寸 | `demo.launch:74` |
| `use_cpu` / 全局 `use_gpu` | `true` | bool | 由于安装 wrapper 语义兼容，`true` 实际选择 CPU | `demo.launch:4,66-67` |
| `visualize_yolo` | `true` | bool | 显示 YOLO OpenCV 窗口 | `demo.launch:2,72` |

`max_tag_correction=1.0`、`descent_cone_ratio=1.5`、`land_tag_distance=0.55` 当前被读取但未参与实际主路径计算，应视为遗留/未接线参数，不能当作当前有效控制约束。

## 11. 静态降落当前实际效果

2026-08-21 至 22 日用完全相同参数完成 5 个正式静态轮次（另有一个不计入统计的 smoke test）：

- 5/5 完整完成并自动 disarm，成功率 100%；没有 OFFBOARD 丢失。
- 起飞至触地平均 58.651 s、最大 58.828 s。
- Gazebo GT 触地水平误差平均 0.341 m、最大 0.364 m。该值接近 0.35 m 控制器判定边界；采样/滤波使汇总值可能略越门限。
- Gazebo GT 触地垂直速度绝对值平均 0.804 m/s、最大 0.816 m/s；相对水平速度平均 0.826 m/s、最大 0.849 m/s。
- 5/5 均发生一次近地 Tag 短时丢失；最长丢失平均 0.297 s、最大 0.352 s。
- 触地后 2 s z 时序未出现超过诊断阈值 0.10 m 的再次上升，`possible_bounce=0/5`。触地时刻来自控制器 Gazebo 高度触发，不是接触传感器。
- 原始数据、字段来源和图见 `vehicle_landing_demo/experiments/`；Gazebo 真值未冒充视觉测量。

### 已经确认的问题

1. **触地裕量有限**：5 次 GT 水平误差平均 0.341 m，接近 0.35 m 门限；相对水平速度平均约 0.826 m/s。
2. **近地 AprilTag 丢失可重复出现**：5/5 均有一次 0.26～0.352 s 丢失；目前依赖短时冻结世界坐标完成末段。
3. **依赖 Gazebo 真值**：平台 local 坐标、速度、clearance 和触地均使用 `/gazebo/model_states`；不是纯视觉闭环。
4. **OFFBOARD/arm 服务重复调用和日志刷屏**：在状态回传更新前每个 30 Hz 周期重复请求并打印 accepted/armed；虽然本轮未被拒绝，但工程行为不够干净。
5. **默认降落平台仍是静态的**：0.5 m/s 只在 vehicle-only launch 验证，不能证明移动落车；正常入口默认 `speed=0.0`。

### 怀疑但未确认的问题

- ❓ 接触力意义上的微小弹跳：当前 z/vz/姿态时序排除了超过 0.10 m 的明显弹跳，但没有接触传感器或接触力数据。
- ❓ 是否存在高频控制振荡：已有约 20 Hz 轨迹与姿态记录，未见导致失败的振荡，但记录中没有完整 setpoint 时序。
- ❓ 更大样本、不同初始条件下的成功率：5 次同工况均成功，但不能给出广泛工况的 P95/置信区间结论。
- ❓ AprilTag 最大检测距离和不同视角鲁棒性：没有系统扫描数据。

## 12. 当前静态平台为什么能够工作

成功链路如下：

```text
Gazebo camera
  `/iris_0/camera/image_raw`
  `models/monocular_camera/monocular_camera.sdf`
↓
AprilTag detector
  `/iris_0/apriltag`
  `config/apriltag_settings.yaml` + `apriltag_tags.yaml`
↓ TF
Relative pose / world target
  `iris_0/base_link -> landing_pad`
  `map -> landing_pad`
  `VehicleLanding.tag_transform()` / `update_tag_world()`
↓
Controller
  `VehicleLanding.update()` / `tag_guidance_velocity()`
↓ raw velocity PositionTarget
`/iris_0/mavros/setpoint_raw/local`
↓
MAVROS ENU/FLU → NED/FRD
↓
PX4 OFFBOARD
↓
Gazebo Iris 响应并接触车顶碰撞面
↓
`finish_platform_touchdown()` → MAVLink command 400 → DONE
```

此外，默认路径不是纯 AprilTag：先由 `/road_yolo` 在 `/landing/yolo_boxes` 上确认车辆；`demo_ground_truth=true` 后，`VehicleLanding.vehicle_local_xy()` 用 Gazebo 中车辆和 Iris 真值消除 local/world 原点差，构造稳定斜向路径；到联合门限后才释放 YOLO 并进入纯 Tag 阶段。这条“视觉门控 + 真值路径 + AprilTag 精降 + 真值触地”的混合链路解释了当前静态仿真的成功。

## 13. 当前系统中针对“静态目标”的假设

虽然代码已有部分移动补偿，但末段仍有明显静态假设：

1. **默认降落目标速度为零**：`launch/demo.launch` 的 `vehicle_speed` 默认 0.0。0.5 m/s 已在独立 vehicle-only 场景验证，但没有把该工况接入无人机控制。
2. **Tag 丢失后冻结绝对位置**：`VehicleLanding.update_tag_world()` 保存 `tag_world`；`TAG_DEAD_RECKON` 不按目标速度传播该位置，见 `vehicle_precision_landing.py:243-274,477-508`。平台移动时，3 s 冻结可产生显著落点滞后。
3. **Tag 末段控制仅计算位置误差**：`tag_guidance_velocity()` 使用 `ex/time_to_touchdown`、`ey/time_to_touchdown`，没有显式叠加 `vehicle_velocity`，见 `vehicle_precision_landing.py:276-286`。Tag 可见时世界目标更新可间接追随，丢码时则没有速度匹配。
4. **没有相对速度闭环**：控制器订阅 UAV pose，但不订阅 MAVROS UAV velocity；平台速度只从 Gazebo 真值取得，没有计算可靠的 `Δvx/Δvy`。
5. **没有独立 TRACK 状态**：状态机从 ACQUIRE 直接进入下降型 `TAG_APPROACH`，缺少“先稳定跟随并验证相对速度收敛，再允许下降”的门控。
6. **丢失策略仅是短时续推/超时悬停**：没有预测协方差、按平台运动外推、重新搜索、爬升复飞或安全中止状态。
7. **触地假设依赖固定平台高度**：`platform_height=2.23` 且使用绝对 Gazebo z；只适用于当前平路、固定车顶高度模型。
8. **目标位置并非找到后永久固定**：需避免误判。Tag 可见时 `update_tag_world()` 每轮继续一阶更新；真正的静态假设集中在丢码冻结、无相对速度控制及默认零车速，而不是“找到一次后永不更新”。

`moving_vehicle.py` 的 0.5 m/s 直线运动现已单独验证；但 `models_cb()` 读取 UGV Gazebo 真值速度、ACQUIRE 速度前馈等无人机移动相关逻辑仍未做闭环跟踪验证。这些不等同于移动降落已实现。

## 14. 从静态平台升级到移动平台，哪些模块需要变化

### 可以保持不变

- ROS Noetic / catkin 包结构。
- PX4 SITL、Iris 模型和 MAVROS 桥接。
- Gazebo 相机及 `/iris_0/camera/image_raw` 接口。
- AprilTag family/ID、检测节点和 TF 基础链路（需后续重新标定/验证，但接口可保持）。
- `/iris_0/mavros/setpoint_raw/local` 速度控制接口。
- YOLO 车辆检测作为远距离发现/门控的基本模块。

### 需要修改

- **Gazebo 平台工况**：将速度从 0 改为可控工况，覆盖匀速、换向、加减速，并记录真值用于评估。
- **目标状态估计**：从 Tag 位姿时序估计目标速度；Gazebo 真值只能用于对照，不应作为最终控制输入。
- **末段控制**：加入平台速度前馈/相对速度闭环，尤其是 Tag 丢失期间的位置外推。
- **状态机**：新增明确的稳定跟踪门控，只有位置和相对速度均收敛才下降；增加 ABORT/REACQUIRE 等安全状态。
- **触地判断**：逐步替换固定 world z 真值，融合 PX4 landed state、垂直速度、测距或接触检测。
- **YOLO/Tag 交接**：在移动工况下重新验证高度、水平距离、连续帧和速度匹配条件。
- **安全逻辑**：人员检测、目标丢失、超时和平台驶离边界需要一致的悬停/复飞策略。

### 需要新增

- UAV 速度订阅和时间同步后的相对速度 `Δv`。
- 目标跟踪状态（位置、速度、时间戳、置信度/协方差）。
- 只跟踪不下降的 `TRACK` 验证阶段。
- 丢失目标后的预测、重捕获、超时复飞/中止逻辑。
- rosbag/CSV 自动记录与批量实验评估：成功率、落点误差、相对触地速度、Tag 丢失率。
- 不同速度、方向、初始偏差和视觉退化条件的测试矩阵。

## 15. 当前是否具备强化学习接入条件

| 状态量 | 当前情况 |
|---|---|
| `Δx, Δy` | 已能在 Tag 阶段由 `tag_world - MAVROS local pose` 得到；但代码内部使用，未形成标准 observation topic。 |
| `Δz` | 可由当前 clearance 近似得到，但默认来自 Gazebo 真值高度减固定平台高度，不是可迁移传感器状态。 |
| 目标 `vx, vy` | 代码能从 `/gazebo/model_states` 取得，仅适合作为仿真真值/教师信号。 |
| UAV `vx, vy, vz` | 当前控制器未订阅 MAVROS velocity topic，因此没有实际速度状态。 |
| `Δvx, Δvy, Δvz` | 当前没有完整计算；目标竖直速度也未建模。 |

当前动作输出是 `vx_cmd, vy_cmd, vz_cmd`，封装为 `mavros_msgs/PositionTarget` raw velocity setpoint。这个动作空间适合未来 PPO，但当前尚未具备规范的 observation、时间同步、reward、episode reset、终止条件和安全动作约束，因此 **还不具备直接开展可信 PPO 训练的完整条件**。

若未来接入 PPO，最合适替换的是 `VehicleLanding.tag_guidance_velocity()` 所代表的 **TRACK/TAG_APPROACH 水平和下降速度策略模块**，保留 PX4/MAVROS 底层、视觉状态估计、状态机安全门控、动作限幅和传统控制 fallback。应先建立可靠移动平台传统控制 baseline，再让 PPO 与同一 observation/action/evaluation 接口对比，而不是替换整个飞行栈。

## 16. 当前最需要解决的 3～5 个问题

- **P0：建立移动目标状态与相对速度估计。** 目前末段只有位置误差，丢码后目标位置冻结；这是从静态走向移动平台的核心结构性缺口。
- **P1：增加“只跟踪、不下降”的 TRACK 阶段。** 在下降前验证 UAV 能持续匹配固定速度平台，避免把跟踪误差和下降问题同时引入。
- **P1：完善目标丢失恢复和安全中止。** 当前只有 3 s dead reckon 后悬停；对移动平台需要速度外推、重捕获和超时复飞/中止。
- **P1：改善并扩大静态基线覆盖。** 当前 5/5 同工况成功，但触地误差接近门限、相对水平速度偏大，后续必须持续回归且增加初始条件覆盖。
- **P2：减少 Gazebo 真值依赖并改进触地判断。** 真值可用于评估，但长期不应直接决定路径和锁桨，否则算法迁移和 RL 评估会失真。

## 17. 推荐下一阶段开发路线

### Step 1：固化静态基线

**本阶段已完成首批基线**：保持控制逻辑与参数不变，正式 5 次全部成功，并形成 CSV/JSON/图。后续改动仍应复用该格式回归。

### Step 2：验证平台物理运动本身

**0.5 m/s 匀速直线已完成**：稳定段速度和横向保持通过。换向、加减速尚未测试；不要同时下降。

### Step 3：增加移动目标状态估计

用 AprilTag 位姿时间序列估计目标位置和速度，以 Gazebo `/model_states` 只做真值对照；加入时间戳、滤波和异常跳变评估。

### Step 4：建立只跟踪不下降的传统控制 baseline

新增 TRACK 阶段，固定高度下验证相对位置和相对速度收敛，覆盖不同平台速度/方向及短时丢码。

### Step 5：实现丢失恢复与安全状态

短时按速度预测目标，超过可信窗口则悬停/爬升并重捕获；达到硬超时进入 ABORT，而不是继续盲降。

### Step 6：在速度匹配后允许下降

把下降门控改为位置误差、相对速度、Tag 置信度/新鲜度联合条件；逐步从低速平台开始，并测量触地相对速度和落点分布。

### Step 7：形成移动平台传统控制 baseline

对多速度、多初始位置、多方向和视觉丢失条件做批量测试，达到预先定义的成功率与 P95 误差后，才判定移动平台自主降落实现。

### Step 8：再接入 PPO

固定 observation/action/reward/termination 接口，PPO 仅替换跟踪/下降速度策略，保留状态估计、安全门控、限幅和传统控制 fallback，并与 Step 7 baseline 公平比较。

## 18. 关键证据

| 重要判断 | 文件路径 / 函数 / 参数 / topic |
|---|---|
| 默认平台静止 | `vehicle_landing_demo/launch/demo.launch:23-29`，`moving_landing_vehicle/speed=0.0` |
| 平台运动原型 | `scripts/moving_vehicle.py:35-62`，`PhysicalVehicleDriver.run()`，`/gazebo/apply_body_wrench` |
| PX4/MAVROS 启动 | `launch/demo.launch:31-46`，PX4 `single_vehicle_spawn_xtd.launch`、MAVROS `px4.launch` |
| 相机安装 | `scripts/sdf_tilt_camera.py:15-29`；`launch/demo.launch:58-63` 静态 TF |
| 相机 topic | `models/monocular_camera/monocular_camera.sdf:11-27`；`/iris_0/camera/image_raw` |
| AprilTag 配置 | `config/apriltag_settings.yaml`、`config/apriltag_tags.yaml`；36h11、ID 0、0.6 m、`landing_pad` |
| Tag 模型位置 | `models/tagged_vehicle/model.sdf:18-40`；车顶 z≈2.231 m；`scripts/sdf_scale_tag.py` |
| YOLO 节点 | `scripts/vehicle_yolo.py` / `VehicleYolo.image_cb()`；输入 camera image，输出 `/landing/yolo_boxes` |
| 主状态机 | `scripts/vehicle_precision_landing.py:333-521` / `VehicleLanding.update()` |
| Tag TF 转换 | `tag_transform()`、`update_tag_world()`；`base_link -> landing_pad`、`map -> landing_pad` |
| 速度控制输出 | `publish_enu_velocity()`、`publish_body_velocity()`；`/iris_0/mavros/setpoint_raw/local` / `PositionTarget` |
| 末段控制 | `tag_guidance_velocity()`；位置误差除以预计剩余下降时间 |
| 触地和锁桨 | `finish_platform_touchdown()`；Gazebo z + 0.35 m 水平门限；MAVLink command 400 |
| 真值依赖 | `models_cb()`、`vehicle_local_xy()`、`demo_ground_truth=true`、`/gazebo/model_states` |
| 静态成功日志 | `.sim_ros_logs/17e7ec1a-.../vehicle_precision_landing-13.log:185-195` |
| YOLO 训练结果 | `training/vehicle_yolo11s_descent/results.csv`，最后 epoch P/R=1、mAP50/mAP50-95=0.995；仅代表当前仿真验证集 |
| 一键入口 | `vehicle_landing_demo/run_demo.sh:4-40` → `roslaunch vehicle_landing_demo demo.launch` |
| 静态重复实验 | `experiments/static_baseline.csv`、`experiments/logs/static_01..05.{csv,json}`；5/5 成功 |
| 实验记录器 | `scripts/experiment_recorder.py`；订阅 `/landing/phase`、MAVROS、Tag TF、YOLO、`/gazebo/model_states` |
| 0.5 m/s 车辆验证 | `launch/vehicle_motion_test.launch`、`scripts/vehicle_motion_recorder.py`、`experiments/vehicle_motion_0p5.csv` |
| 车辆速度环 | `scripts/moving_vehicle.py`；物理 wrench PI 控制，测试参数 Kp=5000、Ki=8000、力上限 18000 N |
| 本阶段报告与图 | `vehicle_landing_demo/NEXT_STAGE_REPORT.md`、`experiments/plots/*.png` |

## 19. 待确认事项

- ❓ 静态降落在更多初始位置、噪声和光照工况下的成功率/P95；当前仅有 5 次同工况数据。
- ❓ 接触力层面的轻微弹跳、侧滑或姿态冲击；20 Hz z/vz/姿态记录未发现超过 0.10 m 的明显弹跳，但没有接触传感器。
- ❓ 近地控制是否有高频振荡；抽样日志收敛，但缺少完整 setpoint/pose 时序。
- ❓ AprilTag 从各高度、角度和光照条件下的最大可靠检测范围与丢帧率。
- ❓ `/iris_0/tag_detections` 的实际运行时消息 frame_id、检测图像完整 topic 名；需在运行中用 `rostopic info/echo` 确认。
- ❓ 当前 ROS/MAVROS/PX4 具体版本组合中 `FRAME_BODY_NED` 输入字段的实测轴向是否与代码注释完全一致；现有成功结果支持符号基本可用，但应做单轴脉冲测试。
- ❓ 移动平台换向、加减速、弯道及更高速度；本阶段只验证 0.5 m/s 匀速直线。
- ❓ `AUTO_LAND` 分支是否仍可到达；当前主状态机没有调用 `request_land()`。
- ❓ `max_tag_correction`、`descent_cone_ratio`、`land_tag_distance` 等已读取未使用参数是否计划保留。

## 20. AI HANDOFF SUMMARY

本工程位于 `/home/fenglijun/DroneWorkspace`，核心 ROS 包是 `vehicle_landing_demo`。目标是在 ROS Noetic + Gazebo + PX4 SITL + MAVROS 中实现无人机对视觉平台/车辆的自主发现、跟踪、对准、下降、触地和自动锁桨，之后扩展到移动平台、目标丢失恢复及 PPO。默认入口是 `vehicle_landing_demo/run_demo.sh`，它设置 ROS/PX4/XTDrone/Gazebo 环境后执行 `roslaunch vehicle_landing_demo demo.launch`。

截至 2026-08-22，**静态平台自主降落已用完全相同参数正式重复 5 次并 5/5 成功；移动平台自主降落仍未实现**。正常 `demo.launch` 的 `vehicle_speed` 默认保持 0.0。静态五轮起飞至触地平均 58.651 s，Gazebo GT 触地水平误差平均/最大 0.341/0.364 m，触地垂直速度绝对值平均 0.804 m/s，相对水平速度平均 0.826 m/s；5/5 均发生一次 0.26～0.352 s 的近地 Tag 丢失，OFFBOARD 异常 0 次、disarm 失败 0 次。20 Hz z/vz/姿态记录按触发后上升 0.10 m 的诊断门限未发现明显弹跳。触地与误差指标来自 Gazebo 真值，不是视觉或接触传感器。

0.5 m/s UGV 已在独立的 `launch/vehicle_motion_test.launch` 中完成 45 s vehicle-only 验证，不含 PX4 或无人机降落。原 `moving_vehicle.py` 的 900 N 比例力不足以克服 1200 kg 车体静摩擦，现改为可配置 PI 物理 wrench 速度环；默认零速时积分清零，不改变静态 baseline。最终稳定段平均速度 0.499999933 m/s、标准差 0.000002023 m/s，横向范围约 1.59e-8 m，无翻车或异常旋转。固定俯视相机可见区间内，车辆真值 x 前进 7.854 m，视觉 `map→landing_pad` TF 前进 7.599 m，趋势正确；YOLO 置信度 0.330～0.734。Tag/YOLO 在车辆驶出固定相机视场后丢失，因此不能称全程连续。

系统数据链为：Gazebo 相机发布 `/iris_0/camera/image_raw` 和 `/iris_0/camera/camera_info`；`/road_yolo`（`scripts/vehicle_yolo.py`）发布 `/landing/yolo_boxes`；namespace 下的 `apriltag_ros_continuous_node` 检测 tag36h11 ID 0、配置尺寸 0.6 m，并发布 `landing_pad` TF；主控制器 `/vehicle_precision_landing`（`scripts/vehicle_precision_landing.py`）订阅 MAVROS state/local pose/extended state、YOLO 框和 Gazebo model states，并通过 tf2 查询 `iris_0/base_link -> landing_pad` 与 `map -> landing_pad`。控制输出是 `/iris_0/mavros/setpoint_raw/local` 上的 `mavros_msgs/PositionTarget` **raw velocity setpoint**，只填写 `vx_cmd, vy_cmd, vz_cmd`，由 MAVROS 完成 ROS ENU/FLU 到 PX4 NED/FRD 转换。

实际状态机为 `WAIT_FCU → TAKEOFF → TRANSIT → ACQUIRE → TAG_APPROACH ↔ TAG_DEAD_RECKON → DONE`。默认先升到 25 m，用 YOLO 确认车辆；由于 `demo_ground_truth=true`，转场、斜向路径、平台速度、相对高度和触地判断大量依赖 `/gazebo/model_states` 真值。到离车顶≤6 m、水平≤3 m且 Tag 可用时释放 YOLO。Tag 世界 x/y 经一阶滤波保存；水平速度按 `error / estimated_time_to_touchdown` 计算并限幅，竖直速度默认固定为 -0.8 m/s。Tag 丢失后冻结最后世界坐标进入 `TAG_DEAD_RECKON`，超过 3 s 且仍较高则悬停；目前没有完整重捕获、搜索、复飞或 ABORT。触地由 Gazebo Iris 高度低于平台高度+0.37 m且水平误差<0.35 m判断，然后发送 MAVLink command 400 强制 disarm。

实验诊断由 `scripts/experiment_recorder.py`、`vehicle_motion_recorder.py` 和 `plot_experiments.py` 完成。结果位于 `vehicle_landing_demo/experiments/static_baseline.csv`、`experiments/logs/static_01..05.{csv,json}`、`experiments/vehicle_motion_0p5.csv` 与 `experiments/plots/`；完整阶段报告是 `vehicle_landing_demo/NEXT_STAGE_REPORT.md`。字段明确区分 visual、MAVROS 与 `_gt` Gazebo 真值。`/landing/phase` 是控制器新增的纯诊断状态输出，不改变速度策略。

从静态升级到移动平台的最大缺口仍是：末段没有可靠相对速度闭环，控制器未订阅 UAV 速度，Tag 丢失后目标位置不按平台速度外推，而且状态机缺少“只跟踪不下降”的 TRACK 门控及完整 lost-target recovery。本阶段结论是 **GO，可以开始开发固定高度 TRACK**：先从 Tag 时序估计目标速度并以 Gazebo 真值仅作评估，再验证位置/相对速度收敛和丢失恢复。该 GO 不授权直接下降；移动目标跟踪仍标为未实现，移动平台自主降落明确为 **❌ 尚未实现**。只有 TRACK 与传统移动降落 baseline 经批量测试稳定后，才考虑 PPO。
