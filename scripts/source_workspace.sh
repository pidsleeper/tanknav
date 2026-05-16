#!/usr/bin/env bash
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash

_ws_setup="${WORKSPACE_ROOT}/install/setup.bash"
if [ -f "$_ws_setup" ]; then
  source "$_ws_setup"
else
  echo "  [tanknav] Workspace not built yet — run ./scripts/build_workspace.sh first" >&2
fi
