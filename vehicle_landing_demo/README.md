# YOLO + AprilTag 无人机精准降落仿真

这是一个基于 ROS Noetic、Gazebo、PX4 SITL 和 MAVROS 的完整视觉降落演示。无人机先用 YOLO 识别车辆并沿斜向航线接近，近距离切换到 AprilTag；标签进入斜视相机盲区后，控制器使用最后滤波得到的标签世界坐标完成短时估计下降。

> 当前默认场景使用静止车辆，目的是验证视觉坐标变换和末段落点。代码是仿真演示，不应未经真机标定、失效保护和封闭场地测试直接用于真实无人机。

## 演示流程

```text
WAIT_FCU
  -> TAKEOFF             起飞至巡航高度
  -> TRANSIT             飞往车辆后侧观察点
  -> ACQUIRE             YOLO 确认车辆并沿斜线下降
  -> TAG_APPROACH        AprilTag 可见，持续滤波标签的 map 坐标
  -> TAG_DEAD_RECKON     标签进入盲区，锁定最后坐标并继续修正
  -> DONE                接触车顶并锁定电机
```

末段不是保存最后一条速度指令。控制器保存的是标签中心坐标，并根据实时机位和剩余高度反复计算速度：

```text
T_touchdown = clearance / descent_speed
v_x = (tag_x - drone_x) / T_touchdown
v_y = (tag_y - drone_y) / T_touchdown
```

## 最值得阅读的文件

| 顺序 | 文件 | 用途 |
|---:|---|---|
| 1 | `launch/demo.launch` | 总入口：场景、PX4、MAVROS、YOLO、AprilTag、控制器和参数 |
| 2 | `scripts/vehicle_precision_landing.py` | 状态机、坐标转换、下降与丢码续航逻辑 |
| 3 | `scripts/vehicle_yolo.py` | YOLO 推理、检测框筛选与可视化 |
| 4 | `scripts/sdf_tilt_camera.py` | 将相机设置为向前下方 45°，并隔离仿真端口 |
| 5 | `models/tagged_vehicle/model.sdf` | 车辆、车顶和 AprilTag 几何模型 |
| 6 | `worlds/yolo_road_stable.world` | 道路、建筑和人员场景 |
| 7 | `config/apriltag_*.yaml` | 标签家族、尺寸、ID 和 TF 名称 |
| 8 | `scripts/moving_vehicle.py` | 车辆动力学驱动；默认速度为 0 |
| 9 | `scripts/sdf_scale_tag.py` | 启动时缩放车顶标签模型 |
| 10 | `run_demo.sh` | 环境装载和一键启动入口 |

博客建议按“系统架构 → 坐标系 → 状态机 → 可见段控制 → 丢码段控制 → 相机外参踩坑 → 实验结果”展开。尤其要说明 `tf/static_transform_publisher` 的旧版欧拉角参数顺序为 `yaw pitch roll`，以及相机光学坐标系 `x=右、y=下、z=前` 与机体 FLU 坐标系的区别。

## 环境要求

本项目在以下环境中运行：

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- PX4 SITL（项目使用兼容 XTDrone 的 PX4 启动文件）
- MAVROS
- `apriltag_ros`
- XTDrone 及其 `yolov11_ros_msgs`
- Python 3：`ultralytics`、`opencv-python`、`numpy`

先检查环境：

```bash
cd vehicle_landing_demo
./scripts/check_environment.sh
```

如果目录不是默认位置，可在启动前指定：

```bash
export CATKIN_WS=/path/to/catkin_ws
export PX4_DIR=/path/to/PX4_Firmware
export XTDRONE_DIR=/path/to/XTDrone
```

## 一键启动

带 Gazebo 和 YOLO 窗口：

```bash
./run_demo.sh gui:=true visualize_yolo:=true
```

无界面运行：

```bash
./run_demo.sh gui:=false visualize_yolo:=false
```

只加载环境、不自动起飞：

```bash
./run_demo.sh auto_start:=false
```

按 `Ctrl+C` 关闭整套仿真。若上次异常退出，可执行：

```bash
pkill -f 'roslaunch vehicle_landing_demo'
pkill -f gzserver
pkill -f gzclient
pkill -f '/px4 '
```

## 关键 ROS 接口

| 接口 | 含义 |
|---|---|
| `/iris_0/camera/image_raw` | 斜向机载相机原图 |
| `/yolov11/detection_image` | YOLO 标注图像 |
| `/landing/yolo_boxes` | 筛选后的车辆/人员检测框 |
| `/iris_0/tag_detections` | AprilTag 检测结果 |
| `map -> landing_pad` | 标签中心在本地世界坐标系中的估计 |
| `/iris_0/mavros/local_position/pose` | 飞控本地位姿 |
| `/iris_0/mavros/setpoint_raw/local` | 控制器发送给 PX4 的速度设定值 |

常用诊断命令：

```bash
rostopic echo /iris_0/mavros/state
rostopic echo /landing/yolo_boxes
rosrun tf tf_echo map landing_pad
rqt_image_view /yolov11/detection_image
```

## 目录结构

```text
vehicle_landing_demo/
├── config/                  AprilTag 配置
├── launch/                  ROS launch 文件
├── meshes/                  标签网格和纹理
├── models/                  Gazebo 车辆/相机模型
├── scripts/                 感知、控制和辅助脚本
├── training/                YOLO 训练结果与权重
├── worlds/                  Gazebo 世界
├── CMakeLists.txt
├── package.xml
└── run_demo.sh              一键启动
```

`datasets/` 和大部分 `training/` 内容用于训练复现，不是运行仿真的必需文件。运行时必须保留：

```text
training/vehicle_yolo11s_descent/weights/best.pt
```

## 发布到 GitHub

### 1. 创建本地仓库

在 `vehicle_landing_demo` 目录执行：

```bash
git init
git branch -M main
git add .
git commit -m "Initial precision landing simulation"
```

### 2. 为模型权重启用 Git LFS

GitHub 普通单文件上限为 100 MB，但二进制模型仍建议使用 Git LFS：

```bash
sudo apt install git-lfs
git lfs install
git lfs track "*.pt"
git add .gitattributes
git add training/vehicle_yolo11s_descent/weights/best.pt
git commit -m "Track model weights with Git LFS"
```

`.gitignore` 默认忽略数据集、训练中间产物、`last.pt`、日志和 Python 缓存，只保留演示所需的 `best.pt`。如需完整公开训练数据，建议压缩后上传到 GitHub Release、Hugging Face Dataset 或网盘，并在 README 中给出下载地址和校验值。

### 3. 在 GitHub 创建空仓库并推送

在 GitHub 网页新建一个空仓库，例如 `uav-apriltag-precision-landing`，不要让网页自动生成 README。然后执行：

```bash
git remote add origin https://github.com/YOUR_NAME/uav-apriltag-precision-landing.git
git push -u origin main
```

使用 GitHub CLI 也可以一步创建并推送：

```bash
gh repo create uav-apriltag-precision-landing \
  --public --source=. --remote=origin --push
```

## 复现者需要知道的限制

- `demo.launch` 依赖 PX4/XTDrone 提供的 `single_vehicle_spawn_xtd.launch` 和模型插件，单独克隆本仓库并不足以运行。
- 不同 PX4、Gazebo、MAVROS 版本之间的端口和插件路径可能不同。
- AprilTag 尺寸、相机内参和外参必须与模型一致，否则控制器会稳定飞向错误坐标。
- 默认场景使用 Gazebo 真值计算离地高度和前期观察路线；末段标签目标来自 AprilTag TF。
- 若要做到真正“一条命令从空系统安装并运行”，建议后续提供 Docker 镜像；带 Gazebo GUI 和 GPU 的 Docker 配置需要额外处理 X11、显卡驱动和设备权限。

## License

MIT，详见 `LICENSE`。
