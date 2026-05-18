#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"

ros2 launch tank_bringup bringup_localization.launch.py \
  base_config:="${WORKSPACE_ROOT}/src/tank_base/config/serial.yaml" \
  livox_config:="${WORKSPACE_ROOT}/livox_ros_driver2/config/MID360_config.json" \
  fastlio_config:="${WORKSPACE_ROOT}/src/tank_bringup/config/fastlio_mid360.yaml" \
  localizer_config:="${WORKSPACE_ROOT}/src/tank_bringup/config/localizer_mid360.yaml"
