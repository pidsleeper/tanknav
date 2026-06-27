#!/bin/bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source ~/tanknav_ws/install/setup.bash
source ~/fast_livo_ws/install/setup.bash
ros2 launch tank_bringup bringup_localization_fastlivo.launch.py
