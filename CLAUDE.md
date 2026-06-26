# tanknav — ROS 2 Navigation for Mid360 + 履带底盘

## 项目概述

基于 ROS 2 Humble 的导航系统，驱动 154mm×267mm×276mm 履带底盘（串口通信控制），搭载 Mid360 激光雷达 + Intel RealSense D435i 深度相机，基于 FAST-LIVO2（视觉+激光+IMU 紧耦合 SLAM）实现已知地图导航。

## 工作空间结构

```
tanknav/
├── src/
│   ├── tank_base/               # 底盘串口驱动 (Python)
│   ├── tank_bringup/            # 启动编排 + 共享配置
│   │   ├── tank_bringup/
│   │   │   ├── cloud_frame_transform.py  # 点云 frame 转换 (camera_init → base_link)
│   │   │   └── fastlivo_nav_bridge.py    # FAST-LIVO2 → Nav2 桥接节点
│   │   └── config/
│   │       ├── fastlivo_mid360.yaml          # 建图/定位用 FAST-LIVO2 配置
│   │       ├── fastlivo_mid360_nav.yaml     # 导航用 FAST-LIVO2 配置
│   │       ├── localizer_mid360_fastlivo.yaml  # localizer 配置
│   │       └── pgo_mid360_fastlivo.yaml        # PGO 配置
│   └── tank_nav2/               # Nav2 参数文件
├── FAST-LIVO2/                  # FAST-LIVO2 视觉-激光-IMU 紧耦合 SLAM (C++)
├── FASTLIO2_ROS2/
│   ├── fastlio2/                # (历史保留) FAST-LIO2 LiDAR-惯性里程计
│   ├── localizer/               # ICP 定位器，已知地图重定位 (C++)
│   ├── pgo/                     # 位姿图优化，建图回环检测 (C++)
│   ├── hba/                     # 混合光束法平差 (未启用)
│   └── interface/               # 自定义消息定义
├── livox_ros_driver2/           # Mid360 官方驱动
├── scripts/                     # 构建/运行快捷脚本
│   ├── build_fastlivo.sh        # FAST-LIVO2 工作空间编译
│   ├── run_mapping_fastlivo.sh  # 建图模式
│   ├── run_localization_fastlivo.sh # 纯定位模式
│   ├── run_nav_fastlivo.sh      # 导航模式
│   ├── build_workspace.sh       # (历史保留) FAST-LIO2 编译
│   ├── source_workspace.sh
│   ├── run_mapping.sh           # (历史保留) FAST-LIO2 模式
│   ├── run_localization.sh      # (历史保留)
│   ├── run_nav.sh               # (历史保留)
│   └── teleop_key.py            # 键盘遥控调试
├── README.md
└── CLAUDE.md
```

> **注意**: 项目现用 FAST-LIVO2 作为主 SLAM 方案。FAST-LIO2 相关包/脚本保留作历史参考和 fallback。

## 三种运行模式

### 1. 建图模式 — `./scripts/run_mapping_fastlivo.sh`
启动链: 底盘 → Mid360 → D435i → FAST-LIVO2 → bridge → PGO (回环) → RViz
- FAST-LIVO2 视觉+激光+IMU 紧耦合建图
- cloud_frame_transform 将点云从 camera_init 系转到 base_link 系
- fastlivo_nav_bridge 转发 odom→base_link TF
- PGO 发布 map→odom TF，积累全局地图
- 不启动 localizer 和 Nav2

### 2. 纯定位模式 — `./scripts/run_localization_fastlivo.sh`
启动链: 底盘 → Mid360 → D435i → FAST-LIVO2 → bridge → localizer → RViz
- localizer 基于 ICP 发布 map→odom TF
- 用于验证已知地图上的重定位

### 3. 导航模式 — `./scripts/run_nav_fastlivo.sh <map.yaml>`
启动链: 底盘 → Mid360 → D435i → FAST-LIVO2 → cloud_frame → bridge → localizer → pointcloud_to_laserscan → Nav2
- cloud_frame_transform 提供 body 系点云供 localizer ICP 使用
- Nav2 延迟启动 (默认 8s，等待 FAST-LIVO2 收敛)

## TF 树 (导航模式)

```
map --[localizer(ICP)/pgo]--> odom(=camera_init) --[bridge]--> base_link(=aft_mapped) --[static]--> mid360_link / camera_link
```

**Frame 映射:**
| FAST-LIVO2 原生 | tanknav 等效 | 说明 |
|----------------|-------------|------|
| `camera_init` | `odom` | 里程计原点 (第一帧相机位姿) |
| `aft_mapped` | `base_link` | 当前车体位姿 |
| `camera_init→aft_mapped` | `odom→base_link` | 位姿变换完全等价，仅名称不同 |

**TF 发布者分布:**
- `localizer` 或 `PGO` → `map→odom` (通过 ICP/图优化)
- `fastlivo_nav_bridge` → `odom→base_link` (转发 `/aft_mapped_to_init`)
- `robot_state_publisher / static TF` → `base_link→mid360_link`, `base_link→camera_link`

> 三模式下 TF 结构完全相同，区别仅在于 map→odom 由 localizer (定位/导航) 还是 PGO (建图) 发布。

## 核心通信契约

| Topic | 发布者 | 说明 |
|-------|--------|------|
| `/livox/lidar` | livox_ros_driver2 | Mid360 点云 |
| `/livox/imu` | livox_ros_driver2 | Mid360 内建 IMU (FAST-LIVO2 前端紧耦合) |
| `/aft_mapped_to_init` | FAST-LIVO2 | 里程计 (camera_init→aft_mapped) |
| `/cloud_registered` | FAST-LIVO2 | 世界系注册点云 (frame=camera_init) |
| `/cloud_body` | cloud_frame_transform | body 系点云 (frame=base_link, 供 localizer/pgo) |
| `/odom` | fastlivo_nav_bridge | Nav2 标准里程计 (odom→base_link) |
| `/cmd_vel` | Nav2 | 速度指令 (下发至底盘) |
| `/scan` | pointcloud_to_laserscan | 2D 激光扫描 (Nav2 障碍物层输入) |
| `/base/feedback_odom` | tank_base | 底盘反馈速度 (仅供诊断，不注入 SLAM) |
| `/base/raw_imu` | tank_base | 底盘 IMU (可选，不注入 SLAM) |
| `/battery_state` | tank_base | 电池电压 |
| `/base/motor_enabled` | tank_base | 电机使能状态 |

## FAST-LIVO2 配置要点

文件: `src/tank_bringup/config/fastlivo_mid360.yaml` (建图/定位), `fastlivo_mid360_nav.yaml` (导航)

- 激光-IMU 外参: 使用 Mid360 出厂标定值
- 相机-IMU 外参: D435i 标定结果写入 YAML (`extrinsic_T`, `extrinsic_R`)
- 点云滤波: `min_range: 0.5`, `max_range: 30.0`, `scan_resolution: 0.15`
- 地图: `map_resolution: 0.3`, `cube_len: 300`
- FAST-LIVO2 原生使用 `camera_init` / `aft_mapped` frame，不直接使用 Nav2 标准 frame
- Frame 映射由 `fastlivo_nav_bridge` 和 `cloud_frame_transform` 两个 Python 桥接节点完成，无需修改 FAST-LIVO2 源码

## 桥接节点

### cloud_frame_transform
- **输入**: `/cloud_registered` (camera_init 系点云), `/aft_mapped_to_init` (位姿)
- **输出**: `/cloud_body` (base_link 系点云)
- **职责**: 将 FAST-LIVO2 世界系点云逆变换到 body 系，供 localizer/pgo 使用
- **原理**: 利用 camera_init→aft_mapped 变换矩阵的逆，将点云从世界系转到 body 系

### fastlivo_nav_bridge
- **输入**: `/aft_mapped_to_init` (frame_id=camera_init, child_frame_id=aft_mapped)
- **输出**: `/odom` (frame_id=odom, child_frame_id=base_link) + odom→base_link TF
- **职责**: camera_init ↔ odom, aft_mapped ↔ base_link 的重命名映射

## 软件架构

### Layer A: 机器人模型 (未实现 `tank_description`)
- 待创建: URDF/Xacro, base_footprint/base_link/mid360_link/camera_link 定义, 传感器外参统一管理
- 目前用 `robot.launch.py` 的 static_transform_publisher 替代

### Layer B: 底盘驱动 — `tank_base`
- `chassis_driver_node.py`: 串口通信，cmd_vel → 自定义串口协议，解析反馈帧
- 串口协议: 帧头 0x7B, BCC 校验, 帧尾 0x7D
- TX 11 字节 (vx/vy/wz ×1000, 单位 mm/s + mrad/s)
- RX 24 字节 (速度反馈 + 加速度 + 角速度 + 电池电压)
- `cmd_rate_hz: 20.0` — 控制指令发送频率
- `feedback_rate_hz: 50.0` — 串口轮询频率
- `cmd_timeout: 0.5s` — 超时未收到 cmd_vel 自动发零速
- **关键约束**: 底盘反馈数据仅供诊断，不注入 FAST-LIVO2

### Layer C: 传感器 — Mid360 + D435i
- `livox_ros_driver2` 通过网口连接 Mid360
- 需配置 `config/MID360_config.json` 中的 host_net_info (Jetson IP)
- D435i 通过 USB 3.0 连接，FAST-LIVO2 消费彩色图 (VIO) 和 IMU

### Layer D: SLAM/里程计 — FAST-LIVO2 (C++)
- 紧耦合 Mid360 点云 + 内建 IMU + D435i 视觉
- iKd-Tree 维护局部地图
- 发布 `/aft_mapped_to_init` (camera_init→aft_mapped)，由 bridge 转为 odom→base_link

### Layer E: 桥接层
- `fastlivo_nav_bridge`: FAST-LIVO2 native frame → Nav2 standard frame
- `cloud_frame_transform`: world cloud → body cloud (供 localizer/pgo)

### Layer F: 导航感知
- `pointcloud_to_laserscan`: `/cloud_body` → `/scan`
- Nav2 costmap 2D 障碍物层消费 `/scan`

### Layer G: 路径规划 — `tank_nav2`
- 基于 Nav2: planner (NavFn), controller, BT Navigator
- 参数文件: `config/nav2_params.yaml` (基线，需实物调参)

## 已完成的工作

- 所有软件包已接通: `tank_base`, `tank_bringup`, `tank_nav2`, `livox_ros_driver2`, `FASTLIO2_ROS2` (fastlio2 历史保留, localizer, pgo, interface)
- FAST-LIVO2 集成为主 SLAM 方案: `cloud_frame_transform` 和 `fastlivo_nav_bridge` 两个桥接节点实现 frame 映射
- 编译命令和三种运行模式的启动脚本已就绪并验证语法 (fastlivo 变体 + 历史 fastlio 变体)
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
  - `FASTLIO2_ROS2/fastlio2` (历史保留，作为 fallback)

### PC 端验证完成

- 底盘串口通信：已通过 `/dev/ttyACM0` 收发正常，`ros2 topic pub /cmd_vel` 手动测试底盘响应正常
- 键盘遥控：`scripts/teleop_key.py` 实现 W/A/S/D/Q/E 控制，Space 急停，+/- 调速
- Mid360 雷达：网络连通 (PC 网口 `enp5s0: 192.168.1.50`)，雷达实际 IP 为 `192.168.1.152`
- Mid360 驱动：`livox_ros_driver2` 已验证收发 `/livox/imu` 和 `/livox/lidar` 数据正常
- D435i 相机：已验证彩色图和 IMU 数据正常
- 编译脚本修复：`set -euo pipefail` → `set -eo pipefail` (ROS humble `setup.bash` 存在未绑定变量)
- 构建通过：PC (x86_64) 上全部包编译成功
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
  - D435i 相机外参 — 按实物安装位置标定

### 导航参数
- `src/tank_nav2/config/nav2_params.yaml`:
  - 机器人半径、速度/加速度限制、避障参数需真机调参
  - 按 NX 算力调整 costmap 更新频率

### FAST-LIVO2 相机标定
- D435i 相机内参/外参需在实物上标定，写入 `fastlivo_mid360.yaml`

### NX 部署 (进行中)

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
| D435i 相机 | 已验证 |
| 底盘串口 | `/dev/ttyACM0` 已确认 |
| Nav2 / pointcloud_to_laserscan | 已装 |
| 工作空间编译 | 已完成 (colcon build 全部通过) |

### 整机联调
- ✅ NX 上跑通建图模式（底盘 + Mid360 + D435i + FAST-LIVO2 + PGO）
- 待做: 纯定位模式、导航模式验证

## 推荐执行顺序

### Step 1: 编译
```bash
# FAST-LIVO2 工作空间
./scripts/build_fastlivo.sh

# tanknav 工作空间
./scripts/build_workspace.sh
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
# source 双工作空间
source /opt/ros/humble/setup.bash
source FAST-LIVO2/install/setup.bash
source install/setup.bash

# 建图
./scripts/run_mapping_fastlivo.sh

# 纯定位测试
./scripts/run_localization_fastlivo.sh

# 导航 (需已有地图)
./scripts/run_nav_fastlivo.sh /绝对路径/地图.yaml
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
./scripts/run_mapping_fastlivo.sh
```
RViz 中能看到实时点云和 FAST-LIVO2 在建图即表示通路正常。

### 3. 保存地图
建图完成后用 PGO 的保存地图服务，或用 map_saver_cli：
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### 4. 导航
```bash
./scripts/run_nav_fastlivo.sh ~/maps/my_map.yaml
```

## 迁移到 Jetson NX

### 整体原则

代码本身不需要改动，在 NX 上重新编译即可（aarch64 不能复用 x86_64 的 install/）。FAST-LIVO2 需单独编译。

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
./scripts/build_fastlivo.sh

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

# 编译 tanknav 工作空间
cd ~/tanknav
./scripts/build_workspace.sh

# 编译 FAST-LIVO2 工作空间
./scripts/build_fastlivo.sh
```

### NX 实物配置

| 配置项 | 文件 | 说明 |
|--------|------|------|
| NX 网口 IP | `livox_ros_driver2/config/MID360_config.json` `host_net_info` | `192.168.1.50` 等 |
| 雷达 IP | 同上 `lidar_configs[0].ip` | NX 环境雷达 IP `192.168.1.152` |
| 底盘串口 | `src/tank_base/config/serial.yaml` `port` | `/dev/ttyACM0` |
| Mid360 外参 | `src/tank_bringup/launch/robot.launch.py` | x/y/z/yaw/pitch/roll |
| D435i 外参 | `src/tank_bringup/config/fastlivo_mid360.yaml` | `extrinsic_T`, `extrinsic_R` |

### NX 部署注意事项

- FAST-LIVO2 在 NX 上约占用 40-60% CPU (视觉+激光+IMU 紧耦合)
- 建议关闭 RViz 节省资源（建图时可以在 PC 端远程看 RViz）
- 建图时注意点云地图大小，`cube_len: 300` 约占用数百 MB 内存
- `tank_nav2/config/nav2_params.yaml` 需要按 NX 算力情况调整 costmap 更新频率
- NX 网口接 Mid360：`enx00e04c68012a` (USB 网卡)，WiFi 口 `wlP1p1s0` 接互联网
- D435i 通过 USB 3.0 连接 NX，确保带宽充足

## 远程 RViz 查看

在 PC 端通过 RViz 查看 NX 上正在运行的机器人状态。

### 方法一：ROS 2 默认 DDS 跨机器通信（推荐）

**前提**: NX 和 PC 在同一个局域网。

```bash
# NX 上启动机器人（任意模式）
cd ~/tanknav
source FAST-LIVO2/install/setup.bash
source install/setup.bash
# 确保不启动本机 RViz
ros2 launch tank_bringup bringup_localization_fastlivo.launch.py

# PC 上启动 RViz
source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2
```

在 RViz 界面中添加显示：
1. `Add` → `By topic` → `/cloud_registered` (PointCloud2)
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
# PC 端只拉取 /cloud_registered
ros2 run rviz2 rviz2 --args -o /cloud_registered
```

### 方法二：SSH 隧道（跨网段时使用）

当 NX 和 PC 不在同一个局域网时（如 NX 通过 4G 上网）：

```bash
# NX 端安装 ros2 bag 记录并传输，或使用 zenoh/mqtt 桥接
# 最简方案：NX 上录包，PC 上回放
# NX:
ros2 bag record -o robot_bag /cloud_registered /odom /scan /tf /tf_static
# PC:
ros2 bag play robot_bag
rviz2
```

## 注意事项

- 底盘为**履带**驱动，差速转向模型
- FAST-LIVO2 原生使用 `camera_init` / `aft_mapped` frame，由 bridge 映射为 Nav2 标准 frame
- D435i 仅用于 FAST-LIVO2 VIO（彩色图），不用于深度避障
- localizer 与 PGO **不可同时运行** (都发布 `map→odom` 会冲突)
- `tank_base` 的底盘 IMU/轮速里程计**仅用于诊断**，不注入 FAST-LIVO2
- HBA 包在工作空间内但未接入默认启动流
- Nav2 参数 `nav2_params.yaml` 为初始模板，需实物标定
- 依赖: gtsam, PCL, Sophus, Livox-SDK2, OpenCV, realsense2, Nav2, pointcloud_to_laserscan
- FAST-LIVO2 为独立工作空间 (tanknav_ws 同级目录)，需先编译再 source

---

## 历史方案: FAST-LIO2

> 以下为项目初版架构信息（纯 LiDAR-IMU 方案），现已切换至 FAST-LIVO2。保留作 fallback 参考。

### TF 树 (FAST-LIO2, 已废弃)

```
map --[localizer/ICP]--> odom --[FAST-LIO2]--> base_link --[static]--> mid360_link
```

### FAST-LIO2 通信契约 (已废弃)

| Topic | 发布者 | 说明 |
|-------|--------|------|
| `/fastlio2/lio_odom` | FAST-LIO2 | 局部里程计 |
| `/fastlio2/body_cloud` | FAST-LIO2 | body 系点云 → `/scan` 投影 → Nav2 costmap |
| `/fastlio2/world_cloud` | FAST-LIO2 | 全局系点云 |

### 运行模式 (已废弃)

```bash
./scripts/run_mapping.sh          # 底盘 → Mid360 → FAST-LIO2 → PGO → RViz
./scripts/run_localization.sh      # 底盘 → Mid360 → FAST-LIO2 → localizer → RViz
./scripts/run_nav.sh /path/map.yaml # + pointcloud_to_laserscan + Nav2
```

### FAST-LIO2 配置 (已废弃)

文件: `src/tank_bringup/config/fastlio_mid360.yaml`
- `world_frame: odom` — 直接使用 Nav2 标准 frame
- `body_frame: base_link` — 直接使用 Nav2 标准 frame
