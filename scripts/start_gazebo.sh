#!/bin/bash
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash

export GZ_SIM_RESOURCE_PATH="$REPO_DIR/simulation/models:$REPO_DIR/simulation/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_DIR/simulation/ardupilot_gazebo/build"

echo "======================================"
echo "        ADDC Gazebo Simulation"
echo "======================================"
echo "Repo: $REPO_DIR"
echo "World: iris_runway.sdf"
echo ""

gz sim -v 4 "$REPO_DIR/simulation/worlds/iris_runway.sdf"
