# 两轮差速自主移动机器人

基于 **ROS 2 + ESP32-S3** 的两轮差速自主移动机器人。下位机在 ESP32-S3 上跑 FreeRTOS + micro-ROS，上位机跑 ROS 2 Humble，自底向上覆盖 **定位（EKF）/ 感知（视觉）/ 视觉-雷达融合 / 避障 / 导航（SLAM + Nav2）** 五层，独立完成从 MCU 外设驱动到上层算法的全栈开发。

## 技术栈

- **下位机**：ESP32-S3 · FreeRTOS · C/C++ · micro-ROS
- **上位机**：ROS 2 Humble · Python · YOLO11n · EKF(robot_localization) · slam_toolbox · Nav2
- **通信**：micro-ROS over WiFi（UDP），ROS 2 话题 `/odom` `/imu` `/scan` `/cmd_vel`
- **建图/导航**：slam_toolbox（异步）· Nav2（全局 A* + 局部 DWA）

## 硬件

| 部件 | 说明 |
|------|------|
| MCU | ESP32-S3 |
| 电机驱动 | AT8236 PWM 调速 |
| 编码器 | PCNT 四倍频测速 |
| IMU | LSM6DS3（零偏校准 + 互补滤波） |
| 雷达 | Camsense 单线激光雷达（UART 解析） |
| 摄像头 | ESP32-CAM（800×600 图传） |
| 电源 | 电池 ADC 采样（分压 + 平均 + 曲线校准） |

## 仓库结构

本仓库由两个工作区组成：

```
leap_demo/
  leap_ros_mcu_driver/   # ESP32-S3 固件：FreeRTOS 任务 + 外设驱动 + micro-ROS
  leap_cam/              # ESP32-CAM 相机固件
mycar_ws/
  src/                   # ROS 2 功能包（mycar_*）
  tools/                 # 调试脚本与开发文档（验收报告 / 融合修复记录等）
```

> 注：ROS 功能包原名 `xuegecar_*` / `xuegeros_demo`，已统一重命名为 `mycar_*`，参数文件 `xuegebot.yaml` → `mycarbot.yaml`，工作区 `xuegeros_ws` → `mycar_ws`。

### 主要 ROS 功能包

| 包 | 职责 |
|----|------|
| `mycar_bringup` | 总启动（socat + micro-ROS Agent + 雷达 + URDF + EKF） |
| `mycar_description` | URDF 机器人模型 |
| `mycar_camera` | 相机 / 视频流 |
| `mycar_vision` | 感知 + 视觉-雷达融合 + 避障（`vision_detector` / `vision_radar_fusion` / `avoidance_decider`） |
| `mycar_navigation2` | 导航（Nav2 + slam_toolbox 参数 / launch） |
| `mycar_cartographer` | Cartographer 建图 |
| `mycar_qt_gui` / `mycar_servo_qt_gui` | Qt 上位机界面 |
| `camsense_lidar` | 雷达驱动 |

## 构建

### 下位机（ESP32-S3 固件）

```bash
cd leap_demo/leap_ros_mcu_driver
idf.py set-target esp32s3
idf.py build flash monitor
```

### 上位机（ROS 2）

```bash
cd mycar_ws
colcon build --symlink-install
source install/setup.bash
```

## 功能要点

- **定位**：`robot_localization` 的 EKF 以 50 Hz 融合轮速里程 + 零偏补偿后的 IMU（预留激光里程相对增量纠偏），输出平滑位姿 `/odometry/filtered` 与一致 TF。
- **感知**：YOLO11n 检测 person/bottle，叠加 HSV 颜色分割与轮廓边缘检测；针对训练集未覆盖的未知物体，以边缘检测快速判定为通用障碍物。
- **融合**：视觉-雷达融合节点以雷达优先、单目回退的策略输出 3D 障碍物。
- **避障**：`avoidance_decider` 订阅融合后的 3D 障碍，输出 `/avoidance_cmd` 避障指令。
- **导航**：slam_toolbox 建图 + Nav2（A* + DWA）完成任务级导航。

## 典型启动

```bash
ros2 launch mycar_bringup bringup_all.launch.py   # 建图/定位链路
ros2 launch mycar_navigation2 navigation2.launch.py   # 导航
```
