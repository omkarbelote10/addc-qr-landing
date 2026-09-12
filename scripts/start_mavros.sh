#!/bin/bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
if [ -f "$REPO_DIR/install/setup.bash" ]; then
    source "$REPO_DIR/install/setup.bash"
fi

echo "======================================"
echo "             Starting MAVROS"
echo "======================================"

ros2 launch mavros apm.launch fcu_url:="udp://127.0.0.1:14550@"
