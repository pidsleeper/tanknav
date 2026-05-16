# Build And Start Order

## 1. Current workspace layout

The ROS 2 packages that should participate in build are:

- `tank_base`
- `tank_bringup`
- `tank_nav2`
- `livox_ros_driver2`
- `fastlio2`
- `localizer`
- `pgo`
- `interface`
- `hba`

These are currently split across:

- `src/`
- `FASTLIO2_ROS2/`
- `livox_ros_driver2/`

## 2. Completed before hardware connection

Already prepared in this workspace:

- `tank_base` serial chassis driver package
- MID360 launch integration through `livox_ros_driver2`
- FAST-LIO2 / localizer / PGO bringup chaining
- Nav2 package and baseline parameter file
- top-level mapping / localization / navigation bringup launch files
- build and run helper scripts under `scripts/`

## 3. Not completed yet

These items still require real robot values or live validation:

- MID360 host IP in `livox_ros_driver2/config/MID360_config.json`
- MID360 lidar IP if your sensor is not `192.168.1.12`
- chassis serial device path in `src/tank_base/config/serial.yaml`
- exact MID360 mounting extrinsics in `robot.launch.py`
- actual robot radius / inflation / controller limits tuning in `tank_nav2/config/nav2_params.yaml`
- fixed-map navigation map file path
- end-to-end runtime validation on Jetson + chassis + MID360

## 4. One-time build sequence

### Step 1: Install system dependencies

Make sure these are already installed on Ubuntu 22.04 + ROS 2 Humble:

- ROS 2 Humble desktop
- `colcon`
- `gtsam`
- `pcl`
- `yaml-cpp`
- `Sophus`
- `Livox-SDK2`
- Nav2 related packages
- `pointcloud_to_laserscan`

If you already built the existing FAST-LIO2 stack before, you likely have most of them.

### Step 2: Build the workspace

From `/home/sgj/tanknav`:

```bash
./scripts/build_workspace.sh
```

Equivalent manual command:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths /home/sgj/tanknav/src /home/sgj/tanknav/FASTLIO2_ROS2 /home/sgj/tanknav/livox_ros_driver2
```

### Step 3: Source the workspace

```bash
source /opt/ros/humble/setup.bash
source /home/sgj/tanknav/install/setup.bash
```

Or:

```bash
source ./scripts/source_workspace.sh
```

## 5. Configuration sequence before connecting hardware

### Step 1: Set MID360 host IP

Edit:

- `livox_ros_driver2/config/MID360_config.json`

Replace every `192.168.1.5` in `host_net_info` with the Jetson ethernet IP that is connected to MID360.

### Step 2: Confirm MID360 lidar IP

Still in:

- `livox_ros_driver2/config/MID360_config.json`

Check:

- `lidar_configs[0].ip`

Default is `192.168.1.12`. Change it only if your MID360 uses a different IP.

### Step 3: Set chassis serial port

Edit:

- `src/tank_base/config/serial.yaml`

Check:

- `port`

Common values are `/dev/ttyUSB0` or `/dev/ttyACM0`.

### Step 4: Confirm frame and extrinsic assumptions

Current assumptions:

- FAST-LIO2 publishes `odom -> base_link`
- MID360 frame name is `mid360_link`
- static transform is published by `tank_bringup/launch/robot.launch.py`

If the sensor is offset from the robot center, update:

- `mid360_x`
- `mid360_y`
- `mid360_z`
- `mid360_yaw`
- `mid360_pitch`
- `mid360_roll`

## 6. Startup order

### Mode A: Mapping

Use when building a new environment map.

```bash
./scripts/run_mapping.sh
```

Runtime chain:

- chassis serial driver
- MID360 driver
- FAST-LIO2
- PGO
- RViz

### Mode B: Localization only

Use when testing relocalization against a known map, without Nav2.

```bash
./scripts/run_localization.sh
```

Runtime chain:

- chassis serial driver
- MID360 driver
- FAST-LIO2
- localizer

### Mode C: Localization + Nav2

Use when a map yaml already exists.

```bash
./scripts/run_nav.sh /absolute/path/to/your_map.yaml
```

Runtime chain:

- chassis serial driver
- MID360 driver
- FAST-LIO2
- localizer
- pointcloud_to_laserscan
- map_server
- Nav2

## 7. Recommended verification order before first real run

### Verification 1: Package discovery

```bash
colcon list --base-paths /home/sgj/tanknav/src /home/sgj/tanknav/FASTLIO2_ROS2 /home/sgj/tanknav/livox_ros_driver2
```

### Verification 2: Launch syntax

Already checked in this workspace:

- `tank_base` launch and node syntax
- `tank_nav2` launch syntax
- `tank_bringup` launch syntax

### Verification 3: MID360 network

Before launching, validate that:

- Jetson ethernet is up
- Jetson IP matches `MID360_config.json`
- MID360 can be reached on the expected subnet

### Verification 4: Serial permission

Before launching, validate that:

- the chassis serial device exists
- your user can read/write that device

## 8. Practical notes

- `tank_base` currently publishes chassis feedback odom only for diagnostics; it is not fused into FAST-LIO2.
- Nav2 currently consumes `/scan` projected from `/fastlio2/body_cloud`.
- `tank_nav2/config/nav2_params.yaml` is a baseline file, not final tuned motion-control parameters.
- `hba` remains in the workspace but is not in the default runtime path.
