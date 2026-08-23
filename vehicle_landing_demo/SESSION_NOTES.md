# 仿真暂停记录（2026-08-15）

## 已保存的当前配置

- 最终 YOLO 权重：`training/vehicle_yolo11s_descent/weights/best.pt`
- 无人机出生点：`(-13, -6, 0)`，位于后侧方观察点正下方
- 起飞高度：25 m
- 车辆速度：0.50 m/s
- 车辆启动延迟：38 s（无人机到达观察高度后启动）
- YOLO/AprilTag 计划交接高度：距车顶 6 m
- YOLO 仍保留 AprilTag 硬负样本、单完整车框和近距离输出隔离

## 本轮结果

- 新出生点实现了基本垂直起飞，不再先从 `(5, 0)` 长距离平移。
- 0.50 m/s 车辆下，YOLO 斜向进近跟随正常。
- AprilTag 在距车顶约 5.99 m 成功获取并进入 `VISUAL_DESCENT`。
- 当前未完成项：AprilTag 控制在横向误差收敛过程中使标签距离增大，随后标签移出视野；状态机回到 `ACQUIRE`，尚未完成最终落车。

## 下次继续

1. 核对 `iris_0/base_link -> landing_pad` 的坐标轴和速度指令符号。
2. 将 AprilTag 交接条件改为“高度 + 图像/横向误差”联合门限，避免在横向距离约 6.5 m 时过早交接。
3. 验证 AprilTag 闭环应使用 BODY_FLU 还是 ENU，并限制修正速度以防标签出画。
4. 重新运行完整仿真，目标是 0.50 m/s 移动车辆上稳定触地。

最后一轮 ROS 日志：`/home/fenglijun/.ros/log/c46807ba-9808-11f1-bd43-6bc78a4a6fa2/`

## 续跑记录（2026-08-17）

- 已将 AprilTag 交接改为高度与水平距离联合门限；当前水平门限为 3.0 m。
- 已将 AprilTag 闭环最大修正速度限制为 0.6 m/s。
- 1.0 m 联合门限测试中，斜向真值闭环在 0.50 m/s 移动车辆上成功触地并锁桨：
  仿真时刻 104.688 s，触地前水平偏差约 0.5 m。日志：
  `.sim_ros_logs/1f656870-99f2-11f1-a1f6-0f84131f03c0/rosout.log`。
- 3.0 m 门限测试在车顶上方 2.59 m、水平 2.99 m 成功进入 `VISUAL_DESCENT`；
  初始标签 y 误差由 3.22 m 收敛到 1.49 m，验证 BODY_FLU 修正符号基本正确。
- 随后发现车辆在 4 m 路段端点反向，而控制器仍使用固定 `+0.50 m/s` 前馈，造成标签再次出画。
  已修正为从 `/gazebo/model_states` 实时读取 `ugv_0` 的 x/y 速度作为前馈。
- 最新代码已通过 `python3 -m py_compile`；实时速度前馈修改后尚未再跑完整落车验证。

下次应直接重跑无界面仿真，重点确认车辆反向后 AprilTag 误差持续收敛并最终触地：

```bash
ROS_LOG_DIR=/home/fenglijun/DroneWorkspace/.sim_ros_logs \
  ./vehicle_landing_demo/run_demo.sh gui:=false visualize_yolo:=false
```
