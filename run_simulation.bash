#!/bin/bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_DIR="$HOME/ardupilot"
SESSION_NAME="addc-qr-landing"
LAT_DEFAULT="-35.3632171"
LON_DEFAULT="149.1652704"
TARGET_LAT="$LAT_DEFAULT"
TARGET_LON="$LON_DEFAULT"

while [ "$#" -gt 0 ]; do
    case "$1" in
        --lat)
            TARGET_LAT="$2"
            shift 2
            ;;
        --lon)
            TARGET_LON="$2"
            shift 2
            ;;
        *)
            echo "Unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

# source /opt/ros/jazzy/setup.bash
# source "$REPO_DIR/scripts/env.sh" >/dev/null

set +u
source /opt/ros/jazzy/setup.bash
source "$REPO_DIR/scripts/env.sh" >/dev/null
set -u

cleanup() {
    tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1 || true
    pkill -f 'sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map' >/dev/null 2>&1 || true
    pkill -f 'gz sim -v4 .*iris_runway.sdf' >/dev/null 2>&1 || true
    pkill -f 'ros2 launch mavros apm.launch fcu_url:="udp://127.0.0.1:14550@"' >/dev/null 2>&1 || true
    pkill -f 'ros2 run ros_gz_bridge parameter_bridge .*camera/image_raw' >/dev/null 2>&1 || true
    pkill -f 'ros2 launch drone_control_pkg landing.launch.py' >/dev/null 2>&1 || true
}

trap cleanup EXIT INT TERM

launch_window() {
    local window_name="$1"
    local command="$2"
    tmux new-window -t "$SESSION_NAME" -n "$window_name" >/dev/null
    tmux send-keys -t "$SESSION_NAME:$window_name" "$command" C-m
    sleep 2
}

start_tmux_stack() {
    tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1 || true
    tmux new-session -d -s "$SESSION_NAME" -n ardupilot
    tmux send-keys -t "$SESSION_NAME:ardupilot" "cd '$ARDUPILOT_DIR' && ./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map" C-m
    sleep 5
    pgrep -f 'sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map' >/dev/null

    launch_window gazebo "gz sim -v4 '$REPO_DIR/simulation/worlds/iris_runway.sdf'"
    pgrep -f 'gz sim -v4 .*iris_runway.sdf' >/dev/null || true

    launch_window mavros "ros2 launch mavros apm.launch fcu_url:=\"udp://127.0.0.1:14550@\""
    sleep 3

    launch_window bridge "ros2 run ros_gz_bridge parameter_bridge /world/iris_runway/model/iris_with_down_camera/link/down_camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image --ros-args -r /world/iris_runway/model/iris_with_down_camera/link/down_camera_link/sensor/camera/image:=/camera/image_raw"
    sleep 2

    launch_window mission "ros2 launch drone_control_pkg landing.launch.py target_lat:=$TARGET_LAT target_lon:=$TARGET_LON"

    tmux select-window -t "$SESSION_NAME:ardupilot"
    tmux list-windows -t "$SESSION_NAME"
    echo "Attach with: tmux attach -t $SESSION_NAME"
    echo "Press Ctrl+C here to stop the whole stack."
    if [ -t 0 ]; then
        read -r -p "" _ </dev/tty || true
    else
        read -r _ || true
    fi
}

start_fallback_stack() {
    if ! command -v gnome-terminal >/dev/null 2>&1; then
        echo "tmux is unavailable and gnome-terminal is not installed." >&2
        exit 1
    fi

    gnome-terminal -- bash -lc "cd '$ARDUPILOT_DIR' && ./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map; exec bash" &
    sleep 4
    gnome-terminal -- bash -lc "gz sim -v4 '$REPO_DIR/simulation/worlds/iris_runway.sdf'; exec bash" &
    sleep 3
    gnome-terminal -- bash -lc "ros2 launch mavros apm.launch fcu_url:=\"udp://127.0.0.1:14550@\"; exec bash" &
    sleep 3
    gnome-terminal -- bash -lc "ros2 run ros_gz_bridge parameter_bridge /world/iris_runway/model/iris_with_down_camera/link/down_camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image --ros-args -r /world/iris_runway/model/iris_with_down_camera/link/down_camera_link/sensor/camera/image:=/camera/image_raw; exec bash" &
    sleep 3
    gnome-terminal -- bash -lc "source '$REPO_DIR/install/setup.bash' && ros2 launch drone_control_pkg landing.launch.py target_lat:=$TARGET_LAT target_lon:=$TARGET_LON; exec bash" &
    echo "Launched fallback terminals. Use Ctrl+C in this terminal to clean up stray processes."
    wait
}

echo "======================================"
echo "          ADDC SIMULATION"
echo "======================================"
echo "Target latitude : $TARGET_LAT"
echo "Target longitude: $TARGET_LON"
echo "1) ArduPilot SITL -> tmux window ardupilot"
echo "2) Gazebo         -> tmux window gazebo"
echo "3) MAVROS         -> tmux window mavros"
echo "4) Bridge         -> tmux window bridge"
echo "5) Mission        -> tmux window mission"

if command -v tmux >/dev/null 2>&1; then
    start_tmux_stack
else
    start_fallback_stack
fi