# Recommended Workspace Layout

This repository currently contains:

- `FASTLIO2_ROS2/`: adapted FAST-LIO2, PGO, localizer, HBA
- `3478910320串口通信控制与反馈_2024.06.18.pdf`: chassis serial protocol tutorial

For a cleaner ROS 2 workspace, the recommended long-term structure is:

```text
src/
  tank_base/                 # chassis serial driver, cmd_vel bridge, status feedback
  tank_description/          # URDF/Xacro, meshes, static frame definitions
  tank_bringup/              # system launch orchestration, rviz, shared configs
  tank_nav2/                 # Nav2 parameter sets, behavior tree customizations
  fastlio2/                  # optionally migrated from FASTLIO2_ROS2/fastlio2
  localizer/                 # optionally migrated from FASTLIO2_ROS2/localizer
  pgo/                       # optionally migrated from FASTLIO2_ROS2/pgo
  interface/                 # optionally migrated from FASTLIO2_ROS2/interface
  hba/                       # optionally migrated from FASTLIO2_ROS2/hba
```

Short term, `tank_bringup/` can stay in `src/` and include the existing
packages directly from `FASTLIO2_ROS2/`.
