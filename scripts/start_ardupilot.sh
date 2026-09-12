#!/bin/bash
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARDUPILOT_DIR="$HOME/ardupilot"

source /opt/ros/jazzy/setup.bash

export GZ_SIM_RESOURCE_PATH="$REPO_DIR/simulation/models:$REPO_DIR/simulation/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_DIR/simulation/ardupilot_gazebo/build"

cd "$ARDUPILOT_DIR"

echo "======================================"
echo "        ADDC ArduPilot SITL"
echo "======================================"

./Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f gazebo-iris \
    --console \
    --map
