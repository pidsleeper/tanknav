#!/bin/bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/tanknav_ws/install/setup.bash
source ~/fast_livo_ws/install/setup.bash
MAP_PATH=${1:-""}
if [ -z "$MAP_PATH" ]; then
    echo "用法: $0 <地图路径.yaml>"
    exit 1
fi
ros2 launch tank_bringup bringup_nav_fastlivo.launch.py map:=$MAP_PATH
