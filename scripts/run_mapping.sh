#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"

ros2 launch tank_bringup bringup_mapping.launch.py \
  base_config:="${WORKSPACE_ROOT}/src/tank_base/config/serial.yaml" \
  livox_config:="${WORKSPACE_ROOT}/livox_ros_driver2/config/MID360_config.json" \
  fastlio_config:="${WORKSPACE_ROOT}/src/tank_bringup/config/fastlio_mid360.yaml" \
  pgo_config:="${WORKSPACE_ROOT}/src/tank_bringup/config/pgo_mid360.yaml" \
  use_rviz:=true
