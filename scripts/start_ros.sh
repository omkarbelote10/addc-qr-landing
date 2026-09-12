#!/bin/bash

set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "======================================"
echo " Starting ROS 2 nodes"
echo "======================================"

source /opt/ros/jazzy/setup.bash

# Source workspace
source "$REPO_DIR/install/setup.bash"

# Put your actual launch command here
# Example:
# ros2 launch drone_control_pkg landing.launch.py

echo "ROS 2 workspace sourced."
