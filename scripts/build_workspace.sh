#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --base-paths \
    "${WORKSPACE_ROOT}/src" \
    "${WORKSPACE_ROOT}/FASTLIO2_ROS2" \
    "${WORKSPACE_ROOT}/livox_ros_driver2"
