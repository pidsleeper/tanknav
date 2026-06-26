# tanknav

基于 ROS 2 Humble 的导航系统，驱动 154mm×267mm×276mm 履带底盘（串口通信控制），搭载 Mid360 激光雷达 + Intel RealSense D435i 深度相机，基于 FAST-LIVO2（视觉+激光+IMU 紧耦合 SLAM）实现已知地图导航。

## 硬件要求

- **Mid360 LiDAR**: 网口连接，出厂标定 LiDAR-IMU 外参
- **Intel RealSense D435i**: USB 3.0 连接，彩色图用于 FAST-LIVO2 VIO（视觉惯性里程计），不用于深度避障
- 履带底盘：154mm×267mm×276mm，差速转向模型，串口通信

## 系统架构

```
底盘驱动 ──→ FAST-LIVO2 ──→ localizer ──→ Nav2
(tank_base)   (VIO+LIO)      (ICP定位)     (路径规划)
    │              │
    │              └── cloud_frame_transform → body 系点云
    │              └── fastlivo_nav_bridge → Nav2 标准 odom
    │
    └──→ 仅诊断反馈，不注入 SLAM
```

### 组件

| 包 | 说明 |
|----|------|
| `tank_base` | 底盘串口驱动，cmd_vel → 串口协议，反馈解析 |
| `tank_bringup` | 启动编排与共享配置，含 bridge/cloud_frame 桥接节点 |
| `tank_nav2` | Nav2 参数文件 |
| `FAST-LIVO2` | 视觉+激光+IMU 紧耦合 SLAM (主方案) |
| `localizer` | ICP 定位器，已知地图重定位 |
| `pgo` | 位姿图优化，建图回环检测 |
| `livox_ros_driver2` | Mid360 LiDAR 驱动 |

## 三种运行模式

```bash
# 建图
./scripts/run_mapping_fastlivo.sh

# 纯定位
./scripts/run_localization_fastlivo.sh

# 导航
./scripts/run_nav_fastlivo.sh /path/to/map.yaml
```

## 快速开始

```bash
# 编译 FAST-LIVO2 工作空间
./scripts/build_fastlivo.sh

# 编译 tanknav 工作空间
./scripts/build_workspace.sh

# 配置串口
# 编辑 src/tank_base/config/serial.yaml → port

# 配置 Mid360 网络
# 编辑 livox_ros_driver2/config/MID360_config.json → host_net_info

# source 双工作空间
source /opt/ros/humble/setup.bash
source FAST-LIVO2/install/setup.bash
source install/setup.bash

# 键盘测试底盘
ros2 launch tank_bringup base.launch.py

# 新终端
python3 scripts/teleop_key.py --linear 0.1
```

详情见 [CLAUDE.md](CLAUDE.md)。
