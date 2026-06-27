# tanknav — ROS 2 Navigation for Mid360 + 履带底盘

## 项目概述

基于 ROS 2 Humble 的导航系统，驱动 154mm×267mm×276mm 履带底盘（串口通信控制），搭载 Mid360 激光雷达，实现已知地图导航。

## 工作空间结构

```
tanknav/
├── src/
│   ├── tank_base/               # 底盘串口驱动 (Python)
│   ├── tank_bringup/            # 启动编排 + 共享配置
│   └── tank_nav2/               # Nav2 参数文件
├── FASTLIO2_ROS2/
│   ├── fastlio2/                # FAST-LIO2 LiDAR-惯性里程计 (C++)
│   ├── localizer/               # ICP 定位器，已知地图重定位 (C++)
│   ├── pgo/                     # 位姿图优化，建图回环检测 (C++)
│   ├── hba/                     # 混合光束法平差 (未启用)
│   └── interface/               # 自定义消息定义
├── livox_ros_driver2/           # Mid360 官方驱动
├── scripts/                     # 构建/运行快捷脚本
│   ├── build_workspace.sh
│   ├── source_workspace.sh
│   ├── run_mapping.sh
│   ├── run_localization.sh
│   ├── run_nav.sh
│   ├── teleop_key.py            # 键盘遥控调试
│   └── pcd_to_navmap.py         # PCD 点云 → 2D 栅格地图转换
├── README.md
└── CLAUDE.md
```

## 三种运行模式

### 1. 建图模式 — `./scripts/run_mapping.sh`
启动链: 底盘 → Mid360 → FAST-LIO2 → PGO (回环) → RViz
- PGO 发布 `map→odom` TF，积累全局地图
- 不启动 localizer 和 Nav2

### 2. 纯定位模式 — `./scripts/run_localization.sh`
启动链: 底盘 → Mid360 → FAST-LIO2 → localizer → RViz
- localizer 基于 ICP 发布 `map→odom` TF
- 用于验证已知地图上的重定位

### 3. 导航模式 — `./scripts/run_nav.sh <map.yaml>`
启动链: 底盘 → Mid360 → FAST-LIO2 → localizer → pointcloud_to_laserscan → map_server → Nav2
- Nav2 延迟启动 (默认 8s，等待 FAST-LIO2 收敛)

## TF 树 (导航模式)

```
map --[localizer/ICP]--> odom --[FAST-LIO2]--> base_link --[static]--> mid360_link
```

## 核心通信契约

| Topic | 发布者 | 说明 |
|-------|--------|------|
| `/livox/lidar` | livox_ros_driver2 | Mid360 点云 |
| `/livox/imu` | livox_ros_driver2 | Mid360 内建 IMU (FAST-LIO2 前端紧耦合) |
| `/fastlio2/lio_odom` | FAST-LIO2 | 局部里程计 |
| `/fastlio2/body_cloud` | FAST-LIO2 | body 系点云 → `/scan` 投影 → Nav2 costmap |
| `/fastlio2/world_cloud` | FAST-LIO2 | 全局系点云 |
| `/cmd_vel` | Nav2 | 速度指令 (下发至底盘) |
| `/scan` | pointcloud_to_laserscan | 2D 激光扫描 (Nav2 障碍物层输入) |
| `/base/feedback_odom` | tank_base | 底盘反馈速度 (仅供诊断，不注入 SLAM) |
| `/base/raw_imu` | tank_base | 底盘 IMU (可选，不注入 SLAM) |
| `/battery_state` | tank_base | 电池电压 |
| `/base/motor_enabled` | tank_base | 电机使能状态 |

## FAST-LIO2 配置要点

文件: `src/tank_bringup/config/fastlio_mid360.yaml`

- `world_frame: odom` — 作为 Nav2 期望的局部里程计坐标系
- `body_frame: base_link` — 若 Mid360 安装位置偏离机器人中心较大，后续应改为 mid360_link 并加 frame adapter
- `publish_system_time: true` — **关键参数**，输出层用系统时间替换 LiDAR 硬件时间，解决 LiDAR 时钟漂移导致的 TF 查询失败
- `imu_topic: /livox/imu` / `lidar_topic: /livox/lidar` — 必须用原始话题，不改时间戳（紧耦合同步不受影响）
- `r_il` / `t_il` — LiDAR-IMU 外参 (Mid360 出厂已标定)
- 点云滤波: `min_range: 0.5`, `max_range: 30.0`, `scan_resolution: 0.15`
- 地图: `map_resolution: 0.3`, `cube_len: 300`

### LiDAR 时钟漂移问题

Mid360 硬件晶振与 NX 系统晶振频率偏差约 0.001%，长时间运行后 LiDAR 时间戳与系统时间差可达几十秒。Nav2 costmap 用 scan 时间戳查询 TF 时，TF buffer 只保留 10 秒，导致 Extrapolation Error，障碍物无法投影。

**解决方案**：修改 `lio_node.cpp`，加 `publish_system_time` 参数。LiDAR/IMU 原样输入紧耦合同步，但 TF、body_cloud、odom、path 发布时间戳用 `this->now()` 切为系统时间。下游全线统一系统时钟域。

## 软件架构

### Layer A: 机器人模型 (未实现 `tank_description`)
- 待创建: URDF/Xacro, base_footprint/base_link/mid360_link 定义, 传感器外参统一管理
- 目前用 `robot.launch.py` 的 static_transform_publisher 替代

### Layer B: 底盘驱动 — `tank_base`
- `chassis_driver_node.py`: 串口通信，cmd_vel → 自定义串口协议，解析反馈帧
- 串口协议: 帧头 0x7B, BCC 校验, 帧尾 0x7D
- TX 11 字节 (vx/vy/wz ×1000, 单位 mm/s + mrad/s)
- RX 24 字节 (速度反馈 + 加速度 + 角速度 + 电池电压)
- `cmd_rate_hz: 20.0` — 控制指令发送频率
- `feedback_rate_hz: 50.0` — 串口轮询频率
- `cmd_timeout: 0.5s` — 超时未收到 cmd_vel 自动发零速
- **关键约束**: 底盘反馈数据仅供诊断，不注入 FAST-LIO2

### Layer C: 传感器 — Mid360
- `livox_ros_driver2` 通过网口连接 Mid360
- 需配置 `config/MID360_config.json` 中的 host_net_info (Jetson IP)

### Layer D: SLAM/里程计 — FAST-LIO2 (C++)
- 紧耦合 Mid360 点云 + 内建 IMU
- iKd-Tree 维护局部地图
- 发布 `odom→base_link`

### Layer E: 导航感知

动态避障采用 **双源 VoxelLayer** 直接消费 3D 点云 + 2D 激光清除：

```
/fastlio2/body_cloud (PointCloud2) ──→ VoxelLayer [标记障碍物]
/scan (LaserScan)                  ──→ VoxelLayer [清除射线，消除动态影子]
                                       ↓
                              /local_costmap/costmap (2D 栅格)
                                       ↓
                              DWB Local Planner → /cmd_vel
```

- `pointcloud_to_laserscan`: `/fastlio2/body_cloud` → `/scan`（2D 投影，供清除射线用）
- VoxelLayer 直接订阅 `/fastlio2/body_cloud`（PointCloud2），绕过 LaserScan 的 BEST_EFFORT QoS 兼容问题
- `/scan` 仅用于清除（`marking: false`），360° 规则射线能穿透旧障碍物位置清除残留标记

### Layer F: 路径规划 — `tank_nav2`
- 基于 Nav2: planner (NavFn), controller, BT Navigator
- 参数文件: `config/nav2_params.yaml` (基线，需实物调参)

## 已完成的工作

- 所有软件包已接通: `tank_base`, `tank_bringup`, `tank_nav2`, `livox_ros_driver2`, `FASTLIO2_ROS2` (fastlio2, localizer, pgo, interface)
- 编译命令和三种运行模式的启动脚本已就绪并验证语法
- 工作空间可被 colcon 正常识别
- 清理项:
  - Python `__pycache__` 目录
  - `log/`
  - Livox ROS1 launch 文件 (`livox_ros_driver2/launch_ROS1`)
  - 冗余的 `package_ROS1.xml` / `package_ROS2.xml`
  - 备份源文件 `FASTLIO2_ROS2/hba/src/hba_node copy.cpp`
  - 嵌套的第三方 `.git/` 目录 (FASTLIO2_ROS2, livox_ros_driver2)
- 保留项:
  - `.vscode/`
  - `FASTLIO2_ROS2/hba` (完整保留，后续地图精化可能用到)

### PC 端验证完成

- 底盘串口通信：已通过 `/dev/ttyACM0` 收发正常，`ros2 topic pub /cmd_vel` 手动测试底盘响应正常
- 键盘遥控：`scripts/teleop_key.py` 实现 W/A/S/D/Q/E 控制，Space 急停，+/- 调速
- Mid360 雷达：网络连通 (PC 网口 `enp5s0: 192.168.1.50`)，雷达实际 IP 为 `192.168.1.152`
- Mid360 驱动：`livox_ros_driver2` 已验证收发 `/livox/imu` 和 `/livox/lidar` 数据正常
- 编译脚本修复：`set -euo pipefail` → `set -eo pipefail` (ROS humble `setup.bash` 存在未绑定变量)
- 构建通过：PC (x86_64) 上全部 9 个包编译成功
- RViz 配置：创建 `src/tank_bringup/rviz/mapping.rviz`，Fixed Frame 设为 `odom`
- 代码已推送至 GitHub: `https://github.com/pidsleeper/tanknav.git`

### livox_ros_driver2 适配补丁

- `CMakeLists.txt`: `DISTRO_ROS` → `$ENV{ROS_DISTRO}` (兼容 humble)
- `src/comm/pub_handler.cpp`: 移除 `kLivoxLidarTypeMid360s` 引用 (当前 Livox-SDK2 未包含此枚举)
- `.gitignore`: 移除 `package.xml` (防止 git 忽略导致 NX 编译失败)

## 未完成 (实物验证前必填)

### 外参配置
- `src/tank_bringup/launch/robot.launch.py`:
  - `mid360_x/y/z/yaw/pitch/roll` — 按实物安装位置标定

### 导航参数
- `src/tank_nav2/config/nav2_params.yaml`:
  - 机器人半径、速度/加速度限制、避障参数需真机调参
  - 按 NX 算力调整 costmap 更新频率

### NX 部署 (已完成)

NX: `aewsw@jetson` (aarch64, Ubuntu 22.04, ROS2 Humble)

| 项目 | 状态 |
|------|------|
| ROS2 Humble | 已装 |
| Livox-SDK2 | 已装 (源码编译) |
| PCL | 已装 (apt) |
| GTSAM | 已装 (apt) |
| Sophus | 已装 (v1.22.10，源码编译) |
| Mid360 网口 | 已配 (Netplan 静态 IP `enx00e04c68012a: 192.168.1.50/24`) |
| Mid360 通信 | 已验证 — `/livox/lidar` 和 `/livox/imu` 数据正常 |
| 底盘串口 | `/dev/ttyACM0` 已确认，需 `sudo chmod 666` 或加入 dialout 组 |
| Nav2 / pointcloud_to_laserscan | 已装 |
| 工作空间编译 | 已完成 (colcon build 全部通过) |

### 整机联调
- ✅ NX 上跑通建图模式（底盘 + Mid360 + FAST-LIO2 + PGO）
- ✅ 导航模式验证通过（定位 + 路径规划 + 动态避障）
- ✅ 动态避障：VoxelLayer 双源方案（body_cloud 标记 + scan 清除）

## 推荐执行顺序

### Step 1: 编译
```bash
./scripts/build_workspace.sh
# 等价于:
# source /opt/ros/humble/setup.bash
# colcon build --symlink-install \
#   --base-paths src FASTLIO2_ROS2 livox_ros_driver2
```

### Step 2: 配置网络
编辑 `livox_ros_driver2/config/MID360_config.json`:
- 将 `host_net_info` 中的 IP 改为 Jetson 网口实际 IP
- 确认 `lidar_configs[0].ip` 与 Mid360 设备 IP 一致

### Step 3: 配置串口
编辑 `src/tank_base/config/serial.yaml`:
- 确认 `port` 为真实设备路径 (`/dev/ttyUSB0` 或 `/dev/ttyACM0`)

### Step 4: 运行对应模式
```bash
source ./scripts/source_workspace.sh

# 建图
./scripts/run_mapping.sh

# 纯定位测试
./scripts/run_localization.sh

# 导航 (需已有地图)
./scripts/run_nav.sh /绝对路径/地图.yaml
```

## 下一步（PC 端联调顺序）

### 1. 配通 Mid360 网络
```bash
# 查看本机网口 IP
ip a
# 编辑 livox_ros_driver2/config/MID360_config.json
# host_net_info → 本机网口 IP（必须和雷达在同一网段，默认 192.168.1.xxx）
# lidar_configs[0].ip → 雷达 IP（默认 192.168.1.12）
```

### 2. 建图
```bash
./scripts/run_mapping.sh
```
RViz 中能看到实时点云和 FAST-LIO2 在建图即表示通路正常。

### 3. 保存地图
建图完成后用 PGO 的保存地图服务，或用 map_saver_cli：
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### 4. 导航
```bash
./scripts/run_nav.sh ~/maps/my_map.yaml
```

## 迁移到 Jetson NX

### 整体原则

代码本身不需要改动，在 NX 上重新编译即可（aarch64 不能复用 x86_64 的 install/）。

### NX 环境准备

```bash
# ROS2 Humble + Nav2
sudo apt install ros-humble-desktop
sudo apt install ros-humble-nav2-*
sudo apt install ros-humble-pointcloud-to-laserscan

# Livox-SDK2（源码编译）
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && sudo make install

# 编译依赖
sudo apt install libgtsam-dev libpcl-dev

# Sophus（NX CMake 3.22 不支持最新版，需 v1.22.10）
git clone https://github.com/strasdat/Sophus.git
cd Sophus && git checkout v1.22.10
mkdir build && cd build
cmake .. && make -j4 && sudo make install
```

### 注意：Conda 环境冲突

NX 如果装了 Conda (miniforge3)，`python3` 可能指向 Conda 而非系统 `/usr/bin/python3`，会导致 ROS 编译失败 (`ModuleNotFoundError: No module named 'catkin_pkg'`)。

处理方式：
```bash
# 退出 conda base 环境后编译
conda deactivate
./scripts/build_workspace.sh

# 或删除 build/install 后指定系统 Python 编译
rm -rf build/ install/
colcon build --symlink-install \
  --base-paths src FASTLIO2_ROS2 livox_ros_driver2
```

### 代码部署

```bash
# git clone（推荐）
git clone https://github.com/pidsleeper/tanknav.git ~/tanknav

# 或 scp
scp -r ~/tanknav nx@<nx-ip>:~/

# 编译
cd ~/tanknav
./scripts/build_workspace.sh
```

### NX 实物配置

| 配置项 | 文件 | 说明 |
|--------|------|------|
| NX 网口 IP | `livox_ros_driver2/config/MID360_config.json` `host_net_info` | `192.168.1.50` 等 |
| 雷达 IP | 同上 `lidar_configs[0].ip` | NX 环境雷达 IP `192.168.1.152` |
| 底盘串口 | `src/tank_base/config/serial.yaml` `port` | `/dev/ttyACM0` |
| Mid360 外参 | `src/tank_bringup/launch/robot.launch.py` | x/y/z/yaw/pitch/roll |

### NX 部署注意事项

- FAST-LIO2 在 NX 上约占用 30-40% CPU
- 建议关闭 RViz 节省资源（建图时可以在 PC 端远程看 RViz）
- 建图时注意点云地图大小，`cube_len: 300` 约占用数百 MB 内存
- `tank_nav2/config/nav2_params.yaml` 需要按 NX 算力情况调整 costmap 更新频率
- NX 网口接 Mid360：`enx00e04c68012a` (USB 网卡)，WiFi 口 `wlP1p1s0` 接互联网

## 远程 RViz 查看

在 PC 端通过 RViz 查看 NX 上正在运行的机器人状态。

### 方法一：ROS 2 默认 DDS 跨机器通信（推荐）

**前提**: NX 和 PC 在同一个局域网。

```bash
# NX 上启动机器人（任意模式）
cd ~/tanknav
source ./scripts/source_workspace.sh
# 确保不启动本机 RViz
ros2 launch tank_bringup bringup_localization.launch.py

# PC 上启动 RViz
source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2
```

在 RViz 界面中添加显示：
1. `Add` → `By topic` → `/fastlio2/world_cloud` (PointCloud2)
2. `Add` → `TF`
3. `Global Options` → `Fixed Frame` 设为 `map`

**如果看不到 topic**，检查网络：
```bash
# 两边分别确认 ROS_DOMAIN_ID 一致（不设置则都是默认值，会通）
echo $ROS_DOMAIN_ID

# 或者指定相同的 ID
export ROS_DOMAIN_ID=42
# 两边都要设

# 测试是否能互相发现
# PC 端执行:
ros2 topic list
# 应该能看到 NX 上发布的 topic
```

也可以只拉取特定 topic，减少网络带宽：
```bash
# PC 端只拉取 /fastlio2/world_cloud
ros2 run rviz2 rviz2 --args -o /fastlio2/world_cloud
```

### 方法二：SSH 隧道（跨网段时使用）

当 NX 和 PC 不在同一个局域网时（如 NX 通过 4G 上网）：

```bash
# NX 端安装 ros2 bag 记录并传输，或使用 zenoh/mqtt 桥接
# 最简方案：NX 上录包，PC 上回放
# NX:
ros2 bag record -o robot_bag /fastlio2/world_cloud /fastlio2/lio_odom /scan /tf /tf_static
# PC:
ros2 bag play robot_bag
rviz2
```

## 注意事项

- 底盘为**履带**驱动，差速转向模型
- FAST-LIO2 将 `body_frame` 设为 `base_link`，若 Mid360 安装偏移大，后续应改为 `odom→mid360_link` + `mid360_link→base_link` 静态 TF
- localizer 与 PGO **不可同时运行** (都发布 `map→odom` 会冲突)
- `tank_base` 的底盘 IMU/轮速里程计**仅用于诊断**，不注入 FAST-LIO2
- HBA 包在工作空间内但未接入默认启动流
- 依赖: gtsam, PCL, Sophus, Livox-SDK2, Nav2, pointcloud_to_laserscan

### 导航参数调优要点

局部避障已通过 VoxelLayer 双源方案解决。当前 `nav2_params.yaml` 关键参数：

| 参数 | 值 | 说明 |
|------|-----|------|
| `publish_system_time` | true | FAST-LIO2 输出切系统时间 |
| `local_costmap.plugins` | [voxel_layer, inflation_layer] | VoxelLayer 替代 obstacle_layer |
| `observation_sources` | body_cloud + scan | PointCloud2 标记 + LaserScan 清除 |
| `body_cloud.obstacle_max_range` | 6.0 | 6m 内标记障碍物 |
| `scan.raytrace_max_range` | 12.0 | 12m 射线覆盖清除 |
| `scan.marking` | false | /scan 只清除不标记 |
| `robot_radius` | 0.2 | 匹配 154×267mm 底盘 |
| `inflation_radius` | 0.45 | 障碍物膨胀区 |
| `max_vel_x` / `max_vel_theta` | 0.3 / 0.8 | 安全速度 |

### 保存地图流程

建图完成后：
```bash
# 1. 保存 PCD 点云地图（给 localizer 定位用）
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '/home/aewsw/maps/', save_patches: false}"

# 2. 转换 2D 栅格地图（给 Nav2 导航用）
python3 ~/tanknav/scripts/pcd_to_navmap.py ~/maps/map.pcd my_map
```
