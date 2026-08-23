# DroneWorkspace / XTDrone 精准降落架构

## 1. 分析范围

`DroneWorkspace` 本身不是普通源码仓库，而是三个源码树的入口：

```text
DroneWorkspace/
├── XTDrone       -> /home/fenglijun/XTDrone
├── PX4_Firmware  -> /home/fenglijun/PX4_Firmware
└── catkin_src    -> /home/fenglijun/catkin_ws/src
```

三者分别承担：

- `XTDrone`：案例脚本、ROS 控制层、传感器算法、Gazebo 场景和定制模型。
- `PX4_Firmware`：PX4 SITL 飞控、飞行控制器、MAVLink 服务和 Gazebo 飞行器模型。
- `catkin_src`：当前 Catkin 工作空间实际参与 ROS 编译和运行的 package。

需要特别区分：

- `XTDrone` 中包含大量可复制到 Catkin 工作空间的源码。
- `catkin_src` 才是当前 ROS 工作空间中实际安装/编译的 package 集合。
- 同名 package 在 `XTDrone` 和 `catkin_src` 中可能是“源码模板”和“已部署副本”。

---

## 2. 总体架构

```text
                         ROS 控制与感知层
 ┌──────────────────────────────────────────────────────────────┐
 │  Gazebo 相机图像                                             │
 │  /iris_N/camera/image_raw                                    │
 │          │                                                   │
 │          ▼                                                   │
 │  apriltag_ros                                                │
 │          │ 发布 TF：iris_N/camera_link -> tag_N              │
 │          ▼                                                   │
 │  precision_landing.py                                        │
 │          │ 查询 TF：map -> tag_N                             │
 │          │ 订阅：/iris_N/mavros/local_position/pose          │
 │          │ 发布：/xtdrone/iris_N/cmd_vel_enu                 │
 │          ▼                                                   │
 │  multirotor_communication.py                                 │
 │          │ 转换为 mavros_msgs/PositionTarget                 │
 │          ▼                                                   │
 │  /iris_N/mavros/setpoint_raw/local                           │
 └───────────────────────────────┬──────────────────────────────┘
                                 │ ROS
                                 ▼
                              MAVROS
                                 │ MAVLink/UDP
                                 ▼
                             PX4 SITL
                                 │ 电机指令、传感器数据
                                 │ MAVLink/TCP + Gazebo Transport
                                 ▼
                              Gazebo
```

精准降落闭环可概括为：

```text
目标 AprilTag → Gazebo 渲染 → 下视相机图像 → AprilTag 检测和位姿估计
    → TF 中目标的 map 坐标 → P 控制器计算 ENU 速度
    → XTDrone communication → MAVROS → PX4 Offboard 速度设定值
    → PX4 姿态/位置控制器 → Gazebo 电机及刚体动力学
```

---

## 3. 工程目录结构

### 3.1 XTDrone

```text
XTDrone/
├── communication/              # XTDrone 命令到 MAVROS 的适配层
├── control/                    # 键盘、地面站、精准降落控制
├── coordination/               # 多机编队、任务分配、launch 生成
├── sensing/
│   ├── object_detection_and_tracking/
│   │   ├── apriltag/
│   │   └── darknet_ros/
│   └── slam/                   # 激光 SLAM、VIO、VSLAM
├── motion_planning/            # 2D、EGO-Planner、UGV 规划
├── sitl_config/
│   ├── launch/                 # SITL、Gazebo、MAVROS 启动文件
│   ├── models/                 # 飞行器、传感器、目标模型
│   ├── worlds/                 # Gazebo 世界
│   ├── mavros/                 # MAVROS 配置
│   ├── gazebo_plugin/
│   ├── gazebo_ros_pkgs/
│   ├── ugv/
│   ├── usv/
│   └── robotic_arm/
├── contributer_demo/
├── robocup/
├── ros2/
└── zhihangcup/
```

### 3.2 PX4_Firmware

```text
PX4_Firmware/
├── build/px4_sitl_default/     # 已构建的 SITL 程序和运行配置
├── ROMFS/px4fmu_common/        # PX4 启动脚本和默认参数
├── Tools/sitl_gazebo/          # Gazebo 模型、世界和 PX4 插件
├── launch/                     # PX4/MAVROS/SITL launch
├── src/
│   ├── modules/                # mavlink、navigator、控制器、EKF 等
│   ├── drivers/
│   └── lib/
├── msg/                        # uORB 消息
└── boards/
```

### 3.3 catkin_src

```text
catkin_src/
├── apriltag_ros/
├── catvehicle/
├── cmdvel2gazebo/
├── gazebo_ros_pkgs/
│   ├── gazebo_dev/
│   ├── gazebo_msgs/
│   ├── gazebo_plugins/
│   ├── gazebo_ros/
│   ├── gazebo_ros_control/
│   └── gazebo_ros_pkgs/
├── obstaclestopper/
├── sicktoolbox/
├── sicktoolbox_wrapper/
├── stepvel/
├── yolov11_ros/
└── yolov11_ros_msgs/
```

---

## 4. 各 package 的作用

### 4.1 当前 Catkin 工作空间中的 package

| Package | 作用 | 精准降落相关性 |
|---|---|---:|
| `apriltag_ros` | AprilTag 3 ROS 封装；订阅图像和标定，发布检测结果及 TF | 核心 |
| `catvehicle` | Gazebo 地面车辆模型、URDF/Xacro、控制器和 launch | 核心 |
| `cmdvel2gazebo` | 将 ROS 速度命令转换为 Gazebo 模型控制 | 间接 |
| `gazebo_dev` | Gazebo ROS 开发依赖和 CMake 配置 | 基础设施 |
| `gazebo_msgs` | Gazebo 模型、link、状态、力等 ROS 消息和服务 | 基础设施 |
| `gazebo_plugins` | 相机、IMU、激光和控制器等 Gazebo-to-ROS 插件 | 核心基础设施 |
| `gazebo_ros` | 启动 Gazebo、生成/删除模型、连接 ROS 与 Gazebo | 核心 |
| `gazebo_ros_control` | Gazebo 与 `ros_control` 之间的硬件接口 | UGV 相关 |
| `gazebo_ros_pkgs` | Gazebo ROS 元 package | 基础设施 |
| `obstaclestopper` | UGV 障碍检测和停车逻辑 | 非核心 |
| `sicktoolbox` | SICK 激光雷达底层驱动库 | 非核心 |
| `sicktoolbox_wrapper` | SICK 驱动的 ROS 封装 | 非核心 |
| `stepvel` | 为 UGV 产生阶跃速度测试命令 | 非核心 |
| `yolov11_ros` | YOLOv11 ROS 目标检测节点 | 无直接关系 |
| `yolov11_ros_msgs` | YOLOv11 检测结果消息 | 无直接关系 |

### 4.2 XTDrone 中的控制和协调组件

| 组件 | 作用 |
|---|---|
| `communication` | 将 XTDrone 自定义控制 topic 转换成 MAVROS setpoint、模式和解锁服务 |
| `control` | 键盘、地面站、精准降落等上层控制程序 |
| `coordination` | 多机编队、任务分配和 launch 生成 |
| `motion_planning` | 二维、三维无人机/无人车路径规划 |
| `sensing` | AprilTag、YOLO、SLAM、VIO、真值位姿等感知 |
| `sitl_config` | Gazebo 模型、世界、插件、MAVROS 配置和 SITL launch |

精准降落直接使用：

- `control/precision_landing.py`
- `control/flj_precision_landing.py`
- `communication/multirotor_communication.py`
- `sensing/object_detection_and_tracking/apriltag/apriltag_ros`
- `sitl_config/launch/outdoor2_precision_landing.launch`
- `sitl_config/models/iris_downward_camera`
- `sitl_config/models/monocular_camera`
- `sitl_config/ugv/catvehicle`

### 4.3 XTDrone 中的独立 ROS package

#### 感知

| Package | 作用 |
|---|---|
| `apriltag_ros` | AprilTag 检测、位姿计算、TF 发布 |
| `darknet_ros` | Darknet/YOLO ROS 检测节点 |
| `darknet_ros_msgs` | Darknet 检测消息 |
| `A-LOAM` | 基于激光雷达的里程计和建图 |
| `camera_models` | VINS-Fusion 相机模型 |
| `vins_estimator` | 视觉惯性状态估计 |
| `loop_fusion` | VINS 回环检测和优化 |
| `global_fusion` | GPS 与 VIO 全局融合 |
| `Sophus` | SE(3)/SO(3) 李群数学库 |

#### EGO-Planner

| Package | 作用 |
|---|---|
| `bspline_opt` | B 样条轨迹优化 |
| `path_searching` | 路径搜索 |
| `plan_env` | 栅格地图和规划环境 |
| `plan_manage` | 规划状态机和多机规划管理 |
| `traj_utils` | 轨迹消息及可视化工具 |
| `drone_detect` | 多机规划中的其他无人机检测 |
| `rosmsg_tcp_bridge` | ROS 消息 TCP 桥 |
| `cmake_utils` | 通用 CMake 工具 |
| `multi_map_server` | 多地图服务 |
| `odom_visualization` | 里程计可视化 |
| `pose_utils` | 位姿工具 |
| `quadrotor_msgs` | 四旋翼规划消息 |
| `rviz_plugins` | RViz 扩展 |
| `uav_utils` | UAV 通用工具 |
| `waypoint_generator` | 航点生成 |

#### UGV 规划

| Package | 作用 |
|---|---|
| `hybrid_astar_test` | Hybrid A* 路径规划测试 |
| `pure_pursuit` | Pure Pursuit 路径跟踪 |

#### Gazebo、车辆和 USV

| Package | 作用 |
|---|---|
| `gazebo_ros_actor_cmd_plugin` | 控制 Gazebo actor |
| `gazebo_ros_actor_cmd_plugin_msgs` | Actor 控制消息 |
| `velodyne_description` | Velodyne 模型描述 |
| `velodyne_gazebo_plugins` | Velodyne Gazebo 传感器插件 |
| `velodyne_simulator` | Velodyne 仿真元 package |
| `le_arm` | 机械臂描述与控制 |
| `le_arm_moveit_config` | 机械臂 MoveIt 配置 |
| `roboticsgroup_gazebo_plugins` | 机械臂 Gazebo 插件 |
| `usv_gazebo_plugins` | 水面艇 Gazebo 插件 |
| `usv_msgs` | 水面艇消息 |
| `vrx_gazebo` | VRX 仿真场景 |
| `wamv_description` | WAM-V 模型描述 |
| `wamv_gazebo` | WAM-V Gazebo 集成 |
| `wave_gazebo` | 波浪环境 |
| `wave_gazebo_plugins` | 波浪动力学插件 |

#### 示例和地面站

| Package | 作用 |
|---|---|
| `formation` | Contributor demo 编队案例 |
| `fixed_wing_formation_control` | 固定翼编队控制 |
| `control` | 编队通信 demo 控制节点 |
| `xtdrone_qt` | Qt 控制界面 |
| `xtdgroundcontrol` | XTDrone C++ 地面控制站 |

---

## 5. 精准降落涉及的 package 和组件

| 层次 | Package/组件 | 职责 |
|---|---|---|
| 仿真 | `gazebo_ros` | 启动 Gazebo，加载世界和模型 |
| 仿真 | `gazebo_plugins` | 将下视相机图像发布到 ROS |
| 飞行器模型 | `iris_downward_camera` | Iris 动力学模型和下视相机 |
| 目标载体 | `catvehicle` | 携带 AprilTag 的移动平台 |
| 感知 | `apriltag_ros` | 从相机图像估计 Tag 相对位姿并发布 TF |
| 坐标变换 | `tf` / `tf2_ros` | 拼接 `map → body → camera → tag` |
| 控制 | `precision_landing.py` | 根据目标位置生成 ENU 速度 |
| XTDrone 适配 | `multirotor_communication.py` | 将 XTDrone `Twist` 转换为 MAVROS `PositionTarget` |
| 飞控桥 | `mavros` | ROS topic/service 与 MAVLink 互转 |
| 飞控 | `px4` | Offboard 位置/速度控制、状态估计、姿态控制 |
| 动力学 | `mavlink_sitl_gazebo` | PX4 与 Gazebo 的传感器/执行器接口 |

UGV 的 Xacro 引用：

```text
model://apriltag0-2/meshes/apriltag0-2.dae
```

`tags.yaml` 配置 ID 0、1、2，设计意图是：

```text
iris_0 → tag_0
iris_1 → tag_1
iris_2 → tag_2
```

### TF 链

```text
map
└── iris_N/base_link             # MAVROS local-position 插件
    └── iris_N/camera_link       # static_transform_publisher
        └── tag_N                # apriltag_ros
```

精准降落节点通过以下调用取得 Tag 在 `map` 中的位置：

```python
tfBuffer.lookup_transform("map", "tag_" + vehicle_id, rospy.Time(0))
```

---

## 6. 精准降落控制逻辑

原始控制器位于 `control/precision_landing.py`。

输入：

```text
TF: map → tag_N
Topic: iris_N/mavros/local_position/pose
```

输出：

```text
/xtdrone/iris_N/cmd_vel_enu
```

控制律：

```text
vx = Kp × (tag_x - vehicle_x)
vy = Kp × (tag_y - vehicle_y)
vz = -land_vel
```

默认参数：

```text
Kp       = 1.0
land_vel = 0.5 m/s
rate     = 50 Hz
```

这里的“精准降落”实际含义是：

- 水平方向跟踪 AprilTag；
- 垂直方向以固定速度下降；
- 不直接调用 PX4 `LAND` 模式；
- 不向 PX4 发送 MAVLink `LANDING_TARGET`；
- 依赖 PX4 Offboard 速度控制和 Gazebo 碰撞模型完成接触。

本地新增的 `flj_precision_landing.py` 加入水平/垂直误差和 `0.1 m` 停止阈值。但发布零速度通常表示 Offboard 悬停，不等价于正式降落或自动解锁；Tag、相机、机体中心及接触面的高度偏移也会影响垂直阈值。

---

## 7. Launch 文件启动关系

### 7.1 精准降落主仿真 launch

入口为 `outdoor2_precision_landing.launch`：

```text
outdoor2_precision_landing.launch
│
├── gazebo_ros/launch/empty_world.launch
│   ├── gzserver
│   ├── gzclient
│   └── outdoor2.world
│
├── group /ugv_0
│   ├── xacro: catvehicle1-3.xacro
│   └── catvehicle/launch/catvehicle.launch
│
├── group /iris_0
│   ├── px4/single_vehicle_spawn_xtd.launch
│   └── mavros/launch/px4.launch（target system 1）
├── group /iris_1
│   ├── PX4 SITL instance 1 + spawn iris_1
│   └── MAVROS（target system 2）
└── group /iris_2
    ├── PX4 SITL instance 2 + spawn iris_2
    └── MAVROS（target system 3）
```

三架无人机都使用 `iris_downward_camera` SDF。

### 7.2 单机生成 launch

```text
single_vehicle_spawn_xtd.launch
├── 从 SDF 生成 model_description 并修改通信端口
├── 启动 PX4 SITL：px4 ... rcS -i N
└── gazebo_ros/spawn_model：插入 iris_N
```

其中：

- `vehicle="iris"` 决定 PX4 airframe/启动配置。
- `sdf="iris_downward_camera"` 决定 Gazebo 物理和传感器模型。

### 7.3 MAVROS launch

每架飞机调用 `mavros/launch/px4.launch`，并分别加载 `px4_config_iris_N.yaml`：

| 飞机 | MAVROS FCU URL | PX4 system ID |
|---|---|---:|
| `iris_0` | `udp://:24540@localhost:34580` | 1 |
| `iris_1` | `udp://:24541@localhost:34581` | 2 |
| `iris_2` | `udp://:24542@localhost:34582` | 3 |

### 7.4 AprilTag launch

AprilTag 检测不由主仿真 launch 自动 include，需要另行启动：

```text
apriltag_ros/launch/xtdrone_detection.launch
├── 加载 settings.yaml 和 tags.yaml
├── /iris_0/apriltag_ros_continuous_node
├── /iris_1/apriltag_ros_continuous_node
├── /iris_2/apriltag_ros_continuous_node
└── 三个 base_link → camera_link 静态 TF
```

### 7.5 完整进程关系

```text
1. outdoor2_precision_landing.launch
2. xtdrone_detection.launch
3. multi_vehicle_communication.sh
4. multi_precision_landing.sh
```

通常还需通过键盘控制或其他节点完成 `ARM`、进入 `OFFBOARD`、起飞/升高，然后开始精准降落控制。

---

## 8. Topic 和数据流

### 感知

```text
Gazebo camera plugin
    ↓
/iris_N/camera/image_raw + /iris_N/camera/camera_info
    ↓
apriltag_ros_continuous_node
    ├── /iris_N/tag_detections
    ├── 检测结果图像
    └── TF: iris_N/camera_link → tag_N
```

### 位姿

```text
Gazebo simulated sensors → PX4 EKF2 → MAVLink LOCAL_POSITION_NED
    → MAVROS ENU 转换 → /iris_N/mavros/local_position/pose
```

### 控制

```text
precision_landing.py
    ↓ geometry_msgs/Twist
/xtdrone/iris_N/cmd_vel_enu
    ↓ multirotor_communication.py
/iris_N/mavros/setpoint_raw/local
    ↓ MAVROS / MAVLink SET_POSITION_TARGET_LOCAL_NED
PX4 → 位置控制器 → 姿态控制器 → mixer → Gazebo motor model
```

XTDrone 使用 ENU（East/North/Up），PX4 使用 NED（North/East/Down），坐标转换由 MAVROS 完成。

---

## 9. Gazebo、PX4、ROS、MAVROS 之间的关系

### Gazebo

物理世界和传感器模拟器，负责刚体动力学、碰撞、摩擦、电机推力、传感器、下视相机、UGV、AprilTag 及地面接触，不负责高层飞行决策。

### PX4 SITL

运行在计算机上的飞控，负责状态估计、解锁、飞行模式、Offboard 检查、位置/速度/姿态控制、电机混控、失控保护和降落检测。

### ROS

感知、控制和编排层，负责启动组件、传输图像和位姿、AprilTag 检测、精准降落控制、TF 和多机 namespace。

### MAVROS

ROS 与 PX4 的协议桥：

```text
ROS messages/services ⇄ MAVROS ⇄ MAVLink ⇄ PX4
```

它负责遥测、setpoint、解锁、模式、参数，以及 ROS ENU/FLU 和 PX4 NED/FRD 坐标转换。

### PX4 与 Gazebo

```text
PX4 SITL ⇄ gazebo_mavlink_interface ⇄ Gazebo sensors/motor plugins
```

| 接口 | 用途 |
|---|---|
| PX4 ↔ MAVROS | 上层控制、状态、模式和遥测 |
| PX4 ↔ Gazebo MAVLink interface | 仿真传感器和执行器 |
| Gazebo ↔ ROS plugins | 相机、模型状态等 ROS 数据 |

---

## 10. PX4 原生精准降落与本案例的区别

PX4 源码包含原生精准降落能力：

```text
src/modules/landing_target_estimator/
src/modules/navigator/precland.*
src/modules/navigator/rtl_params.c
src/modules/mavlink/...LANDING_TARGET...
```

但当前 XTDrone 案例没有走该路径。

### 当前案例

```text
AprilTag TF → ROS 自定义 P 控制器 → Offboard 速度 setpoint → PX4 普通速度控制
```

### PX4 原生方案

```text
视觉系统 → MAVLink LANDING_TARGET → PX4 landing_target_estimator
    → navigator/precland → PX4 自动精准降落状态机
```

判断依据：

- `precision_landing.py` 不发布 MAVROS landing-target topic；
- 输出是 `/xtdrone/.../cmd_vel_enu`；
- communication 将它转发到 `mavros/setpoint_raw/local`；
- MAVROS 配置中 `landing_target.listen_lt` 为 `false`；
- ROS 控制器主动计算水平误差和下降速度。

---

## 11. 当前工作树状态和潜在问题

### 11.1 多机数量不一致

仿真 launch 创建 3 架 Iris 和 1 辆 UGV，但当前 `multi_vehicle_communication.sh` 设置 `iris_num=1`、`rover_num=1`，`multi_precision_landing.sh` 只实际执行 `flj_precision_landing.py iris 0`。当前配置不是完整的三机精准降落。

### 11.2 主 launch 不启动感知和控制

`outdoor2_precision_landing.launch` 不启动 `apriltag_ros`、communication 或 precision landing 控制器，三者必须独立启动。

### 11.3 零速度不等于落地完成

`flj_precision_landing.py` 达到阈值后只发布零速度，没有切换 `AUTO.LAND`、调用 disarm 或检查 PX4 landed 状态。

### 11.4 TF 依赖

控制器要求以下 TF 全部存在：

```text
map → iris_N/base_link → iris_N/camera_link → tag_N
```

任何一段缺失都会导致 `lookup_transform("map", "tag_N")` 持续失败。

### 11.5 模型存在多份副本

`iris_downward_camera` 同时存在于 XTDrone 和 `PX4_Firmware/Tools/sitl_gazebo/models/`。launch 实际读取 `$(find px4)/Tools/sitl_gazebo/models/...`，所以仅修改 XTDrone 副本不一定影响运行。

### 11.6 本地修改

当前检查发现：

- `XTDrone/control/flj_precision_landing.py` 是未跟踪文件；
- `multi_precision_landing.sh` 有本地修改；
- `multi_vehicle_communication.sh` 有本地修改；
- PX4 源码树存在大量本地修改及未跟踪 launch。

本文对应当前磁盘状态，不一定等同于 XTDrone 上游仓库默认状态。

---

## 12. 推荐阅读顺序

1. `sitl_config/launch/outdoor2_precision_landing.launch`
2. `sitl_config/launch/single_vehicle_spawn_xtd.launch`
3. `models/iris_downward_camera/iris_downward_camera.sdf`
4. `models/monocular_camera/monocular_camera.sdf`
5. `apriltag_ros/launch/xtdrone_detection.launch`
6. `apriltag_ros/config/tags.yaml`
7. `control/precision_landing.py`
8. `communication/multirotor_communication.py`
9. `mavros/launch/px4_config_iris_N.yaml`
10. PX4 Offboard、MAVLink 和多旋翼位置控制模块

一句话总结：

> XTDrone 精准降落案例是一个运行在 ROS 侧的视觉伺服系统：Gazebo 生成下视图像，`apriltag_ros` 将目标转换为 TF，Python 控制器生成 ENU 下降速度，XTDrone communication 经 MAVROS 把速度设定值发送给 PX4 Offboard 控制器，最终由 PX4 驱动 Gazebo 中的无人机落向目标。
