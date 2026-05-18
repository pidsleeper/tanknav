#!/usr/bin/env bash
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash

_ws_setup="${WORKSPACE_ROOT}/install/setup.bash"
if [ -f "$_ws_setup" ]; then
  source "$_ws_setup"
else
  echo "  [tanknav] Workspace not built yet — run ./scripts/build_workspace.sh first" >&2
fi

# Auto-configure DDS for NX dual-network (WiFi + LiDAR)
# Ensures ROS traffic goes through the WiFi interface for cross-machine comms
if ip -4 addr show 2>/dev/null | grep -q "10\.143\.100\."; then
  _dds_config="/tmp/tanknav_fastdds.xml"
  if [ ! -f "$_dds_config" ]; then
    _wifi_ip="$(ip -4 addr show | grep -oP '10\.143\.100\.\d+' | head -1)"
    cat > "$_dds_config" << EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_transport</transport_id>
            <type>UDPv4</type>
            <interfaceWhiteList>
                <address>${_wifi_ip}</address>
            </interfaceWhiteList>
        </transport_descriptor>
    </transport_descriptors>
</profiles>
EOF
  fi
  export FASTRTPS_DEFAULT_PROFILES_FILE="$_dds_config"
  echo "  [tanknav] DDS bound to WiFi ($_wifi_ip) for remote RViz access"
fi
