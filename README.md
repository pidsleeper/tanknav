# tanknav

基于 ROS 2 Humble 的导航系统，驱动 154mm×267mm×276mm 履带底盘（串口通信控制），搭载 Mid360 激光雷达，实现已知地图导航。

## 系统架构

```
底盘驱动 ──→ FAST-LIO2 ──→ localizer ──→ Nav2
(tank_base)   (里程计)      (ICP定位)     (路径规划)
    │
    └──→ 仅诊断反馈，不注入 SLAM
```

### 组件

| 包 | 说明 |
|----|------|
| `tank_base` | 底盘串口驱动，cmd_vel → 串口协议，反馈解析 |
| `tank_bringup` | 启动编排与共享配置 |
| `tank_nav2` | Nav2 参数文件 |
| `fastlio2` | LiDAR-IMU 紧耦合里程计 (iKd-Tree) |
| `localizer` | ICP 定位器，已知地图重定位 |
| `pgo` | 位姿图优化，建图回环检测 |
| `livox_ros_driver2` | Mid360 LiDAR 驱动 |

## 三种运行模式

```bash
# 建图
./scripts/run_mapping.sh

# 纯定位
./scripts/run_localization.sh

# 导航
./scripts/run_nav.sh /path/to/map.yaml
```

## 快速开始

```bash
# 编译
./scripts/build_workspace.sh

# 配置串口
# 编辑 src/tank_base/config/serial.yaml → port

# 配置 Mid360 网络
# 编辑 livox_ros_driver2/config/MID360_config.json → host_net_info

# 键盘测试底盘
source ./scripts/source_workspace.sh
ros2 launch tank_bringup base.launch.py

# 新终端
python3 scripts/teleop_key.py --linear 0.1
```

详情见 [CLAUDE.md](CLAUDE.md)。
