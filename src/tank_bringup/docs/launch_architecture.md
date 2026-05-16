# Launch Architecture for MID360 + FAST-LIO2 + Nav2

## 1. Goal

Build a layered ROS 2 launch structure for:

- indoor / outdoor mapping
- fixed-map localization
- point-to-point navigation
- dynamic obstacle avoidance

With the agreed technical route:

- MID360 point cloud + built-in IMU only
- FAST-LIO2 as the odometry / local SLAM core
- no chassis wheel odometry fused into SLAM
- Nav2 consumes the resulting local odometry and fixed map localization output

## 2. Existing code constraints

From the current workspace:

- `fastlio2` subscribes to:
  - `/livox/lidar`
  - `/livox/imu`
- `fastlio2` publishes:
  - `/fastlio2/body_cloud`
  - `/fastlio2/world_cloud`
  - `/fastlio2/lio_path`
  - `/fastlio2/lio_odom`
  - TF: `world_frame -> body_frame`
- `localizer` subscribes to:
  - `/fastlio2/body_cloud`
  - `/fastlio2/lio_odom`
- `localizer` publishes:
  - TF: `map -> odom` when `lio_odom.header.frame_id == odom`
- `pgo` subscribes to:
  - `/fastlio2/body_cloud`
  - `/fastlio2/lio_odom`
- `pgo` publishes:
  - TF: `map -> odom` when configured with `local_frame: odom`
  - `/pgo/loop_markers`
  - map saving service

## 3. Recommended TF contract

### Mapping mode

```text
odom --(FAST-LIO2)--> base_link
base_link --(static)--> mid360_link
```

### Localization / navigation mode

```text
map --(localizer)--> odom --(FAST-LIO2)--> base_link --(static)--> mid360_link
```

Notes:

- For Nav2, `odom` should be the local continuous frame from FAST-LIO2.
- `map` should be generated only by `localizer` in localization mode.
- `map` should be generated only by `pgo` in loop-closure mapping mode.
- Do not let `localizer` and `pgo` run together in the same mode.

## 4. Functional layers

### Layer A: robot model and static frames

Responsibilities:

- publish `base_link`, `base_footprint`, `mid360_link` relations
- keep sensor mounting extrinsics in one place

Suggested package:

- `tank_description`

### Layer B: chassis communication

Responsibilities:

- subscribe `cmd_vel`
- convert to serial frame:
  - `vx [mm/s]`
  - `vy [mm/s]`
  - `wz * 1000 [rad/s]`
- parse feedback frame:
  - velocity
  - battery
  - stop flag
  - optional chassis IMU

Suggested package:

- `tank_base`

Recommended topic contract:

- subscribe: `/cmd_vel`
- publish: `/base/status`
- publish: `/battery_state`
- publish: `/base/raw_imu` optionally

Important:

- keep chassis IMU and wheel-speed feedback available for diagnostics
- do not feed wheel odometry into FAST-LIO2 in the current route

### Layer C: sensor layer

Responsibilities:

- launch MID360 driver
- guarantee output topics match FAST-LIO2:
  - `/livox/lidar`
  - `/livox/imu`

Suggested package:

- `tank_sensors` or keep the vendor driver external

### Layer D: SLAM / odometry layer

Responsibilities:

- FAST-LIO2 publishes local odometry
- PGO is used only in map-building mode
- localizer is used only in fixed-map localization mode

### Layer E: navigation perception layer

Responsibilities:

- convert 3D cloud to 2D `LaserScan` if using 2D costmaps
- or keep a 3D obstacle layer later if needed

Suggested first step:

- `pointcloud_to_laserscan`
- input: `/fastlio2/world_cloud` or raw projected cloud path chosen during integration
- output: `/scan`

### Layer F: Nav2 layer

Responsibilities:

- map server
- planner / controller / bt navigator / recoveries
- dynamic obstacle avoidance from local costmap observation sources

Suggested package:

- `tank_nav2`

## 5. Runtime modes

### Mode 1: mapping

Bring up:

- robot static frames
- chassis driver
- MID360 driver
- FAST-LIO2
- PGO
- RViz

Do not bring up:

- localizer
- Nav2

### Mode 2: localization only

Bring up:

- robot static frames
- chassis driver
- MID360 driver
- FAST-LIO2
- localizer
- RViz

Do not bring up:

- PGO
- Nav2

### Mode 3: localization + navigation

Bring up:

- robot static frames
- chassis driver
- MID360 driver
- FAST-LIO2
- localizer
- pointcloud_to_laserscan
- map_server
- Nav2
- RViz

Do not bring up:

- PGO

## 6. Recommended package tree

```text
src/
  tank_base/
    launch/
      base_driver.launch.py
    config/
      serial.yaml
    tank_base/
      chassis_driver_node.py

  tank_description/
    launch/
      robot_state_publisher.launch.py
    urdf/
      tank.urdf.xacro
    meshes/

  tank_bringup/
    launch/
      robot.launch.py
      base.launch.py
      lidar.launch.py
      slam.launch.py
      mapping.launch.py
      localization.launch.py
      navigation.launch.py
      bringup_mapping.launch.py
      bringup_nav.launch.py
    config/
      fastlio_mid360.yaml
      pgo_mid360.yaml
      localizer_mid360.yaml
      pointcloud_to_scan.yaml
    rviz/
    maps/
    docs/

  tank_nav2/
    launch/
      nav2_core.launch.py
    config/
      nav2_params.yaml
      keepout_mask.yaml
      speed_filter.yaml

  fastlio2/
  localizer/
  pgo/
  interface/
  hba/
```

## 7. Integration notes

### Topic alignment

Use these topic names consistently:

- `/livox/lidar`
- `/livox/imu`
- `/fastlio2/lio_odom`
- `/fastlio2/body_cloud`
- `/scan`
- `/cmd_vel`

### Frame alignment

Recommended names:

- `map`
- `odom`
- `base_footprint`
- `base_link`
- `mid360_link`

### One practical caveat

The current `fastlio2` node publishes `world_frame -> body_frame` directly from the
internal FAST-LIO2 state. For best TF semantics, that tracked frame should really be
the MID360 tracking frame, not an arbitrary robot frame. The current skeleton uses
`body_frame: base_link` for easier Nav2 hookup, but if the sensor mount offset is not
negligible, the better follow-up is:

1. let FAST-LIO2 publish `odom -> mid360_link`
2. publish a static `mid360_link -> base_link`
3. add a lightweight pose/frame adapter if Nav2 must consume `odom -> base_link`

That follow-up is a code refinement step, not a launch-structure problem.
