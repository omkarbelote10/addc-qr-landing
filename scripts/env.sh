#!/bin/bash

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# # ROS 2 Jazzy
# source /opt/ros/jazzy/setup.bash

# # Workspace
# if [ -f "$REPO_DIR/install/setup.bash" ]; then
#     source "$REPO_DIR/install/setup.bash"
# fi

set +u
source /opt/ros/jazzy/setup.bash

if [ -f "$REPO_DIR/install/setup.bash" ]; then
    source "$REPO_DIR/install/setup.bash"
fi
set -u

# Gazebo resources
export GZ_SIM_RESOURCE_PATH="$REPO_DIR/simulation/models:$REPO_DIR/simulation/worlds:${GZ_SIM_RESOURCE_PATH:-}"

# ArduPilot Gazebo plugins
export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_DIR/simulation/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

echo "Environment loaded:"
echo "REPO_DIR = $REPO_DIR"
echo "GZ_SIM_RESOURCE_PATH = $GZ_SIM_RESOURCE_PATH"
echo "GZ_SIM_SYSTEM_PLUGIN_PATH = $GZ_SIM_SYSTEM_PLUGIN_PATH"
