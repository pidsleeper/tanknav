#!/bin/bash
set -eo pipefail
echo "=== 编译 fast_livo_ws ==="
cd ~/fast_livo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
echo "=== 编译 tanknav_ws ==="
cd ~/tanknav_ws
source ~/fast_livo_ws/install/setup.bash
colcon build --symlink-install --base-paths src/tanknav/src src/tanknav/FASTLIO2_ROS2 src/tanknav/livox_ros_driver2
echo "=== 编译完成 ==="
