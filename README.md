# ADDC QR Landing
Autonomous QR-code search, scan and land.

## 1. Project Overview
The objective of this project is to develop an autonomous drone system capable  
of navigating to a target using its approximate GPS coordinates, searching for  
a QR code despite GPS error, localizing the QR code using computer vision,  
scanning and extracting the qr code data and finally performing a precision  
landing on the target.

The provided GPS coordinates are expected to have an error of approximately 4 m.  
Therefore, GPS alone cannot be used and Vision Based landing is required.


# 2. Mission Objective

The overall mission can be represented as:

```text
GPS Target Coordinates
          │
          ▼
   Navigate to Target
          │
          ▼
   Enter Search Area
          │
          ▼
     Search Pattern
          │
          ▼
     QR Detection
       /       \
    Found     Not Found
      │           │
      │      Continue Search
      │           │
      ▼           └───────┐
 QR Localization          │
      │                   │
      ▼                   │
 Target Alignment         |
      |                   |
      ▼                   |
  scanning QR             |
      │                   │
      ▼                   │
 Controlled Descent       │
      │                   │
      ▼                   │
     LAND ◄───────────────┘
```

---


# 3. Repository Structure

```text
addc-qr-landing/
│
├── README.md
├── .gitignore
│
└── src/
    └── drone_control_pkg/
        │
        ├── drone_control_pkg/
        │   ├── __init__.py
        │   ├── mission_control_node.py
        │   └── vision_tracker_node.py
        │
        ├── launch/
        │   ├── landing.launch.py
        │
        │
        ├── resource/
        │   └── drone_control_pkg
        │
        ├── test/
        ├── package.xml
        ├── setup.py
        └── setup.cfg
```


# 4. Setup

## Requirements

Currently the development environment uses:

* Ubuntu 24.04
* ROS 2 Jazzy
* Gazebo Sim
* Python 3
* ArduPilot SITL
* ArduPilot Gazebo Plugin
* MAVROS
* ros_gz_bridge
* OpenCV
* NumPy

### Simulation Models / World (from ardupilot gz plugin):
* iris_with_down_camera.sdf
* iris_runway.sdf

---

## Clone the Repository

```bash
git clone https://github.com/omkarbelote10/addc-qr-landing.git
cd addc-qr-landing
```

---

## Install ROS Dependencies

From the repository root:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## Build

```bash
colcon build --symlink-install
```

After a successful build:

```bash
source install/setup.bash
```

---

## Verify the Package

```bash
ros2 pkg list | grep drone_control_pkg
```

Expected:

```text
drone_control_pkg
```

---

# 9. Running the Project

Before running the project, source ROS 2 and the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/aeronitk/addc-qr-landing/install/setup.bash
```

---

## Simulation Setup & Launch

Start the following components in **separate terminals**.

### 1. ArduPilot SITL

```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f JSON:127.0.0.1:9002 --console -N
```

### 2. Gazebo

```bash
gz sim -r iris_runway.sdf
```

### 3. MAVROS

```bash
ros2 launch mavros apm.launch fcu_url:="udp://127.0.0.1:14550@"
```

### 4. Gazebo ↔ ROS 2 Bridge

```bash
ros2 run ros_gz_bridge parameter_bridge \
/world/iris_runway/model/iris_with_down_camera/link/down_camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
--ros-args \
-r /world/iris_runway/model/iris_with_down_camera/link/down_camera_link/sensor/camera/image:=/camera/image_raw
```

### 5. Launch the Mission

After the simulation components are running:

```bash
source ~/aeronitk/addc-qr-landing/install/setup.bash
ros2 launch drone_control_pkg landing.launch.py

The landing launch file is intended to start the nodes required for the autonomous landing pipeline.

Current nodes include:

* `mission_control_node`
* `vision_tracker_node`
```



## System Architecture & Data Flow

1. **Gazebo** simulates the drone and publishes the downward-facing camera feed as `gz.msgs.Image`.

2. **`ros_gz_bridge`** converts the Gazebo image message (`gz.msgs.Image`) into a ROS 2 image message (`sensor_msgs/msg/Image`). The camera topic is remapped to `/camera/image_raw`.

3. **Vision Tracker** subscribes to `/camera/image_raw`, processes the image using OpenCV, detects the QR code, and publishes its center pixel coordinates `(x, y)` on `/target_pixel`.

4. **Mission Controller** subscribes to `/target_pixel` and MAVROS telemetry/state topics to obtain the target position, drone pose, connection state, armed state, and flight mode. It uses a state machine to manage the mission and sends velocity, arm/disarm, and mode commands through MAVROS.

5. **MAVROS** acts as the ROS 2 ↔ MAVLink interface between the Mission Controller and ArduPilot, providing telemetry to ROS 2 and forwarding commands to the flight controller.

6. **ArduPilot** receives the commands through MAVLink and performs the low-level flight control of the drone.



## Current State

* 🟢 Implemented State machine with appropriate state switching.
* 🟢 Drone aligns itself above the detected target
* 🟢 Current vision planner detects the **red cylinder** and performs vision-based landing

## Not Implemented

* 🔴 GPS error simulation
* 🔴 Search algorithm to compensate for the ~4 m GPS error
* 🔴 Integration of the QR detection pipeline with the existing vision landing controller
* 🔴 Replacing the current red-cylinder detection with the QR-based target pipeline


