#!/usr/bin/env bash
set -eo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 /absolute/path/to/map.yaml"
  exit 1
fi

MAP_PATH="$1"
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"

ros2 launch tank_bringup bringup_nav.launch.py \
  map:="${MAP_PATH}" \
  base_config:="${WORKSPACE_ROOT}/src/tank_base/config/serial.yaml" \
  livox_config:="${WORKSPACE_ROOT}/livox_ros_driver2/config/MID360_config.json" \
  fastlio_config:="${WORKSPACE_ROOT}/src/tank_bringup/config/fastlio_mid360.yaml" \
  localizer_config:="${WORKSPACE_ROOT}/src/tank_bringup/config/localizer_mid360.yaml" \
  pcd_map:="${HOME}/maps/map.pcd" \
  nav2_params:="${WORKSPACE_ROOT}/src/tank_nav2/config/nav2_params.yaml" \
  nav2_start_delay:=8.0
