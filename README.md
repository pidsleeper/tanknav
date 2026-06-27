# tanknav

基于 ROS 2 Humble 的导航系统，驱动 154mm×267mm×276mm 履带底盘（串口通信控制），搭载 Mid360 激光雷达，实现已知地图导航。

## 系统架构

```
Mid360 → FAST-LIO2 → localizer → Nav2
              │
         TF odom→base_link
              │
     body_cloud → VoxelLayer (动态避障)
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

## 全流程指南

### 前提

NX 上 conda 必须退出，串口必须有权限：

```bash
conda deactivate
sudo chmod 666 /dev/ttyACM0
```

### 一、编译

```bash
cd ~/tanknav
./scripts/build_workspace.sh
```

### 二、建图

**NX 启动建图**（不启动 RViz 节省资源）：

```bash
source ~/tanknav/scripts/source_workspace.sh
./scripts/run_mapping.sh
```

**PC 遥控底盘**走遍场地：

```bash
python3 ~/tanknav/scripts/teleop_key.py --linear 0.1
```

**PC 远程 RViz 查看**（可选）：

```bash
source /opt/ros/humble/setup.bash
rviz2 -d ~/tanknav/src/tank_bringup/rviz/mapping.rviz
```

### 三、保存地图

走完场地后，NX 上保存两种地图：

```bash
# 1. 保存 PCD 点云地图（给 localizer ICP 定位用）
source ~/tanknav/install/setup.bash
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/home/aewsw/maps/', save_patches: false}"

# 2. 转换 2D 栅格地图（给 Nav2 路径规划用）
python3 ~/tanknav/scripts/pcd_to_navmap.py ~/maps/map.pcd my_map
```

确认生成的文件：

```bash
ls -lh ~/maps/
# 应该有: map.pcd  my_map.yaml  my_map.pgm
```

### 四、启动导航

NX 上一键启动：

```bash
conda deactivate
cd ~/tanknav
source ./scripts/source_workspace.sh
sudo chmod 666 /dev/ttyACM0
./scripts/run_nav.sh ~/maps/my_map.yaml
```

启动链：底盘 → Mid360 → FAST-LIO2 → localizer → Nav2 → 自动加载定位。

### 五、查看与操作

**NX 或 PC 上启动 RViz**：

```bash
rviz2 -d ~/tanknav/src/tank_bringup/rviz/navigation.rviz
```

RViz 中已配置好的显示：
- **Global Map** — 全局栅格地图（灰白）
- **Local Costmap** — 局部代价地图（障碍物红紫，膨胀区蓝）
- **Body Cloud** — 实时 3D 点云
- **TF** — 坐标系
- **Nav Plan** — 规划路径（绿色）

操作步骤：
1. 工具栏点 **2D Pose Estimate** → 在地图上点机器人的大约位置，拖出朝向（触发 ICP 重定位）
2. 工具栏点 **2D Nav Goal** → 点目标位置，拖出朝向
3. 绿色路径线出现后，底盘自动行驶

### 六、键盘遥控（调测用）

```bash
source ~/tanknav/scripts/source_workspace.sh
python3 ~/tanknav/scripts/teleop_key.py --linear 0.1
```

| 键 | 动作 |
|----|------|
| W/S | 前进/后退 |
| A/D | 左转/右转 |
| Q/E | 前进+左转/前进+右转 |
| Space | 急停 |
| +/- | 加减速 |
| Ctrl+C | 退出 |

## 动态避障原理

local costmap 采用 **VoxelLayer 双源**方案：

```
/fastlio2/body_cloud (PointCloud2, 3D) ──→ VoxelLayer [标记障碍物]
/scan (LaserScan, 2D 360°)            ──→ VoxelLayer [清除射线消影]
                                              │
                                     /local_costmap/costmap
                                              │
                                     DWB Local Planner → /cmd_vel
```

- `body_cloud` 负责标出动态障碍物（人、物体），3D 点云覆盖范围广
- `/scan` 只负责清除（`marking: false`），360° 规则射线能穿过旧障碍物位置清除残留
- FAST-LIO2 配置 `publish_system_time: true`，输出层切系统时间，TF 不再漂移

## NX 部署状态

| 项目 | 状态 |
|------|------|
| 建图 | ✅ |
| 纯定位 | ✅ |
| 导航 + 动态避障 | ✅ |

详情见 [CLAUDE.md](CLAUDE.md)。
