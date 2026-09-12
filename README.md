# AeronitK ADDC - Autonomous QR Detection & Precision Landing

Autonomous QR-code search, align, scan, and land for the ADDC drone stack.

## Overview

This repository contains a ROS 2 Jazzy package for precision landing using ArduPilot SITL, Gazebo Harmonic, MAVROS, `ros_gz_bridge`, OpenCV, and NumPy.

## Repository Layout

```text
addc-qr-landing/
├── setup.bash
├── run_simulation.bash
├── scripts/
│   ├── env.sh
│   ├── start_ardupilot.sh
│   ├── start_gazebo.sh
│   ├── start_mavproxy.sh
│   ├── start_mavros.sh
│   └── start_ros.sh
├── simulation/
│   ├── worlds/iris_runway.sdf
│   ├── models/
│   └── ardupilot_gazebo/
└── src/drone_control_pkg/
    ├── package.xml
    ├── launch/landing.launch.py
    └── drone_control_pkg/
        ├── mission_control_node.py
        └── vision_tracker_node.py
```

## Setup

### Requirements

The project targets Ubuntu 24.04 with:

* ROS 2 Jazzy
* Gazebo Harmonic
* ArduPilot SITL
* MAVROS
* `ros_gz_bridge`
* OpenCV
* NumPy

### One-command setup

From the repository root, run:

```bash
chmod +x setup.bash run_simulation.bash
./setup.bash
```

This installs the host dependencies, sources or installs ROS 2 Jazzy if needed, configures `rosdep`, installs workspace dependencies, clones the pinned ArduPilot and ArduPilot Gazebo plugin revisions, builds the plugin, and runs `colcon build --symlink-install`.

## Running the Project

The normal workflow is a single command:

```bash
./run_simulation.bash
```

Override the mission target if needed:

```bash
./run_simulation.bash --lat -35.3632171 --lon 149.1652704
```

`run_simulation.bash` launches ArduPilot SITL, Gazebo, MAVROS, the Gazebo-to-ROS camera bridge, and the mission launch in separate `tmux` windows when available. If `tmux` is missing, it falls back to `gnome-terminal`.

## Manual / Debug Mode

If you need to run components individually, source the workspace first:

```bash
source /opt/ros/jazzy/setup.bash
source ~/aeronitk/addc-qr-landing/install/setup.bash
```

Then launch the pieces manually in separate terminals.

### ArduPilot SITL

```bash
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

### Gazebo

```bash
gz sim -v4 ~/aeronitk/addc-qr-landing/simulation/worlds/iris_runway.sdf
```

### MAVROS

```bash
ros2 launch mavros apm.launch fcu_url:="udp://127.0.0.1:14550@"
```

### Gazebo to ROS bridge

```bash
ros2 run ros_gz_bridge parameter_bridge \
/world/iris_runway/model/iris_with_down_camera/link/down_camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
--ros-args \
-r /world/iris_runway/model/iris_with_down_camera/link/down_camera_link/sensor/camera/image:=/camera/image_raw
```

### Mission launch

```bash
ros2 launch drone_control_pkg landing.launch.py target_lat:=-35.3632171 target_lon:=149.1652704
```

## Data Flow

1. Gazebo publishes the downward camera stream as `gz.msgs.Image`.
2. `ros_gz_bridge` remaps the Gazebo image into `/camera/image_raw`.
3. `vision_tracker_node` detects the red target in OpenCV and publishes pixel coordinates on `/target_pixel`.
4. `mission_control_node` consumes the target pixel stream and MAVROS telemetry to drive the state machine.
5. MAVROS forwards commands and telemetry between ROS 2 and ArduPilot.
6. ArduPilot executes the low-level flight control.

## Mission State Machine

The mission controller advances through these states:

| State | Purpose |
|---|---|
| `WAIT_FOR_CONNECTIONM` | Wait for MAVROS/FCU heartbeat |
| `SET_GUIDED_MODE` | Switch to `GUIDED` |
| `ARM` | Arm the vehicle |
| `TAKEOFF` | Climb to the configured takeoff height |
| `NAVIGATE_TO_TARGET` | Fly to the GPS target |
| `SEARCH_TARGET` | Hover and look for the target |
| `ALIGN_TARGET` | Center over the detected target |
| `DESCEND` | Continue alignment while descending |
| `LAND` | Command landing mode |
| `FINISHED` | End the mission |

## Notes

* `rpi5/` is ignored from `colcon build` by `setup.bash`.
* `simulation/ardupilot_gazebo/` is fetched fresh by `setup.bash` and should not be committed.
* `mav.tlog` and `mav.tlog.raw` are ignored because they are generated at runtime.