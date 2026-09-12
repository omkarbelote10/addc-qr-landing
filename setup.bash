#!/bin/bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_DIR="$HOME/ardupilot"
ARDUPILOT_GAZEBO_DIR="$REPO_DIR/simulation/ardupilot_gazebo"
# Pinned to Copter-4.7.1, the latest stable Copter-4.x tag chosen for this bootstrap.
ARDUPILOT_TAG="Copter-4.7.1"
# Pinned to the validated plugin revision used by this project.
ARDUPILOT_GAZEBO_COMMIT="082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5"

section() {
    printf '\n======================================\n'
    printf '%s\n' "$1"
    printf '======================================\n'
}

require_ubuntu_2404() {
    . /etc/os-release
    if [ "${VERSION_ID:-}" != "24.04" ]; then
        echo "This setup script is intended for Ubuntu 24.04. Detected: ${PRETTY_NAME:-unknown}" >&2
        exit 1
    fi
}

ensure_ros_source() {
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
        return 0
    fi

    section "Installing ROS 2 Jazzy"
    sudo apt-get update
    sudo apt-get install -y curl gnupg lsb-release software-properties-common
    sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    local codename
    codename="$(. /etc/os-release; printf '%s' "$UBUNTU_CODENAME")"
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${codename} main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    sudo apt-get update
    sudo apt-get install -y ros-jazzy-desktop
    source /opt/ros/jazzy/setup.bash
}

ensure_gazebo_repo() {
    if [ ! -f /etc/apt/sources.list.d/gazebo-stable.list ]; then
        sudo apt-get update
        sudo apt-get install -y curl lsb-release gnupg
        sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release; printf '%s' "$UBUNTU_CODENAME") main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
    fi
    sudo apt-get update
    sudo apt-get install -y gz-harmonic
}

ensure_rosdep() {
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update
}

ensure_ardupilot() {
    if [ ! -d "$ARDUPILOT_DIR/.git" ]; then
        git clone https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_DIR"
    fi

    git -C "$ARDUPILOT_DIR" fetch --tags --prune
    git -C "$ARDUPILOT_DIR" checkout "$ARDUPILOT_TAG"
    git -C "$ARDUPILOT_DIR" submodule update --init --recursive

    local stamp_file="$ARDUPILOT_DIR/.setup-tag"
    if [ ! -f "$stamp_file" ] || [ "$(cat "$stamp_file")" != "$ARDUPILOT_TAG" ] || [ ! -x "$ARDUPILOT_DIR/build/sitl/bin/arducopter" ]; then
        section "Installing ArduPilot prereqs and building SITL"
        bash "$ARDUPILOT_DIR/Tools/environment_install/install-prereqs-ubuntu.sh" -y
        (cd "$ARDUPILOT_DIR" && ./waf configure --board sitl && ./waf copter)
        printf '%s' "$ARDUPILOT_TAG" > "$stamp_file"
    fi
}

ensure_ardupilot_gazebo() {
    if [ -d "$ARDUPILOT_GAZEBO_DIR/.git" ]; then
        git -C "$ARDUPILOT_GAZEBO_DIR" fetch --all --tags --prune
        git -C "$ARDUPILOT_GAZEBO_DIR" checkout "$ARDUPILOT_GAZEBO_COMMIT"
        git -C "$ARDUPILOT_GAZEBO_DIR" submodule update --init --recursive
    else
        rm -rf "$ARDUPILOT_GAZEBO_DIR"
        git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$ARDUPILOT_GAZEBO_DIR"
        git -C "$ARDUPILOT_GAZEBO_DIR" checkout "$ARDUPILOT_GAZEBO_COMMIT"
        git -C "$ARDUPILOT_GAZEBO_DIR" submodule update --init --recursive
    fi

    local build_stamp="$ARDUPILOT_GAZEBO_DIR/build/.setup-commit"
    if [ ! -f "$build_stamp" ] || [ "$(cat "$build_stamp")" != "$ARDUPILOT_GAZEBO_COMMIT" ]; then
        section "Building ArduPilot Gazebo plugin"
        cmake -S "$ARDUPILOT_GAZEBO_DIR" -B "$ARDUPILOT_GAZEBO_DIR/build"
        cmake --build "$ARDUPILOT_GAZEBO_DIR/build"
        printf '%s' "$ARDUPILOT_GAZEBO_COMMIT" > "$build_stamp"
    fi
}

section "Checking host and ROS prerequisites"
require_ubuntu_2404
ensure_ros_source

section "Installing base packages"
ensure_gazebo_repo
sudo apt-get install -y \
    build-essential \
    cmake \
    curl \
    git \
    gnupg \
    gstreamer1.0-gl \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-bad \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    libgz-sim8-dev \
    libopencv-dev \
    lsb-release \
    mavproxy \
    rapidjson-dev \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-opencv \
    python3-pip \
    python3-rosdep \
    ros-jazzy-cv-bridge \
    ros-jazzy-geographic-msgs \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    tmux
python3 -m pip install --user --upgrade pygeodesy

section "Preparing rosdep"
ensure_rosdep

section "Preparing workspace dependencies"
touch "$REPO_DIR/rpi5/COLCON_IGNORE"
cd "$REPO_DIR"
rosdep install --from-paths src --ignore-src -r -y

section "Preparing ArduPilot and plugin"
ensure_ardupilot
ensure_ardupilot_gazebo

section "Building workspace"
colcon build --symlink-install

printf '\nsetup complete - source install/setup.bash or run ./run_simulation.bash\n'