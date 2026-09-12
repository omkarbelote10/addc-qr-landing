#!/bin/bash
# setup_rpi.bash — bootstrap a BRAND NEW Raspberry Pi for drone_control_pkg
# (companion computer, real flight controller — no Gazebo, no ArduPilot SITL)
#
# ASSUMPTIONS (fix these two lines if wrong for your image):
#   - OS: Ubuntu Server 24.04 (arm64) — matches the ROS 2 Jazzy binary repo.
#     If this is Raspberry Pi OS (Debian) instead, this script will fail at
#     require_ubuntu_2404 and you'd need to build ROS 2 from source instead.
#   - Camera: USB/V4L2-compatible camera (ros-jazzy-v4l2-camera). If you're
#     using the Raspberry Pi Camera Module via libcamera, swap that install
#     for the appropriate libcamera ROS driver instead.
#
# Layout this script expects (copy from the monorepo's rpi5/ folder):
#   ~/pi_ws/
#     └── src/
#         └── drone_control_pkg/   <- copied from rpi5/src/drone_control_pkg
#     └── setup_rpi.bash           <- this file
#
# Usage: chmod +x setup_rpi.bash && ./setup_rpi.bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

section() {
    printf '\n======================================\n'
    printf '%s\n' "$1"
    printf '======================================\n'
}

require_ubuntu_2404() {
    . /etc/os-release
    if [ "${VERSION_ID:-}" != "24.04" ]; then
        echo "This setup script is intended for Ubuntu 24.04 (arm64). Detected: ${PRETTY_NAME:-unknown}" >&2
        echo "If this is Raspberry Pi OS, ROS 2 Jazzy has no apt binaries for it — you'd need to build from source instead." >&2
        exit 1
    fi
}

ensure_ros_source() {
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        set +u
        source /opt/ros/jazzy/setup.bash
        set -u
        return 0
    fi

    section "Installing ROS 2 Jazzy (ros-base, no desktop/GUI — not needed on the Pi)"
    sudo apt-get update
    sudo apt-get install -y curl gnupg lsb-release software-properties-common
    sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    local codename
    codename="$(. /etc/os-release; printf '%s' "$UBUNTU_CODENAME")"
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${codename} main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    sudo apt-get update
    sudo apt-get install -y ros-jazzy-ros-base
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
}

ensure_rosdep() {
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update
}

ensure_geographiclib_datasets() {
    # mavros_msgs / GeoidPGM (used in mission_control_node.py) needs the
    # actual egm96-5.pgm geoid file on disk, not just the pygeodesy package.
    local script_path
    script_path="$(ros2 pkg prefix mavros 2>/dev/null || true)/lib/mavros/install_geographiclib_datasets.sh"
    if [ -f "/usr/share/GeographicLib/geoids/egm96-5.pgm" ]; then
        return 0
    fi
    if [ -f "$script_path" ]; then
        section "Installing GeographicLib geoid datasets (required by GeoidPGM)"
        sudo bash "$script_path"
    else
        echo "WARNING: could not find install_geographiclib_datasets.sh (mavros not installed yet?). Will retry after mavros install." >&2
    fi
}

section "Checking host prerequisites"
require_ubuntu_2404
ensure_ros_source

section "Installing base packages"
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    curl \
    git \
    gnupg \
    lsb-release \
    libopencv-dev \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-opencv \
    python3-pip \
    python3-rosdep \
    ros-jazzy-cv-bridge \
    ros-jazzy-geographic-msgs \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    ros-jazzy-v4l2-camera
python3 -m pip install --user --upgrade --break-system-packages pygeodesy

section "Installing GeographicLib geoid data (needed by mission_control_node's GeoidPGM)"
ensure_geographiclib_datasets

section "Preparing rosdep"
ensure_rosdep

section "Resolving package dependencies"
cd "$REPO_DIR"
if [ ! -d "$REPO_DIR/src/drone_control_pkg" ]; then
    echo "ERROR: $REPO_DIR/src/drone_control_pkg not found." >&2
    echo "Copy the rpi5/src/drone_control_pkg folder from the main repo into $REPO_DIR/src/ first." >&2
    exit 1
fi
rosdep install --from-paths src --ignore-src -r -y

section "Building workspace"
colcon build --symlink-install

section "Setup complete"
cat <<EOF

Next steps (not handled by this script yet):
  1. Source the workspace: source install/setup.bash
     (add "source $REPO_DIR/install/setup.bash" to ~/.bashrc if you want this automatic)
  2. Connect MAVROS to the real flight controller over serial, e.g.:
       ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:57600
     (adjust the serial device and baud rate for your FC wiring)
  3. Start the camera driver so /camera/image_raw is actually published, e.g.:
       ros2 run v4l2_camera v4l2_camera_node --ros-args -r /image_raw:=/camera/image_raw
     (swap this for a libcamera-based node if you're using the Pi Camera Module)
  4. Launch the mission:
       ros2 launch drone_control_pkg landing.launch.py target_lat:=<lat> target_lon:=<lon>

EOF