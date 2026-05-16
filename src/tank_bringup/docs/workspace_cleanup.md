# Workspace Cleanup Summary

## Keep

These are required for the current ROS 2 stack:

- `FASTLIO2_ROS2/fastlio2`
- `FASTLIO2_ROS2/localizer`
- `FASTLIO2_ROS2/pgo`
- `FASTLIO2_ROS2/interface`
- `FASTLIO2_ROS2/hba`
- `livox_ros_driver2`
- `src/tank_base`
- `src/tank_bringup`
- `src/tank_nav2`

## Safe to remove

These are not required for the current ROS 2 runtime:

- Python `__pycache__` directories
- `log/`
- Livox ROS1 launch files in `livox_ros_driver2/launch_ROS1`
- redundant `livox_ros_driver2/package_ROS1.xml`
- redundant `livox_ros_driver2/package_ROS2.xml`
- backup source file `FASTLIO2_ROS2/hba/src/hba_node copy.cpp`

## Not removed on purpose

These were left untouched:

- `.vscode/`
- nested `.git/` directories in vendor code
- `FASTLIO2_ROS2/hba` package

Reason:

- they may still be useful for your local workflow or future map refinement
