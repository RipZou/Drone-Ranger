# Drone Ranger – Unity Data Pipeline

**PX4 + ROS 2 + Unity Integration**

This repository contains the Unity-side data pipeline for the Drone Ranger project. It integrates PX4 SITL, ROS 2, and Unity to simulate sensors, publish perception data, and visualize the drone in a modular and reproducible way.

---

## System Design Philosophy

The system follows a **strict separation of responsibilities**:

- **PX4**: Flight control, state estimation, and safety
- **ROS 2**: Middleware, data translation, planning, and perception
- **Unity**: Environment simulation and sensor emulation
  - No dynamics
  - No control
  - No estimator

---

## System Architecture

```
PX4 SITL
├─ uXRCE-DDS client (built-in)
│
├── Micro XRCE-DDS Agent
│
↓
ROS 2
├─ px4_msgs
├─ odom_bridge
│
└── ROS-TCP-Endpoint (TCP server)
│
↓
Unity (sensor simulation)
│
└── MAVLink → QGroundControl
```

---

## Unity Components

### `DepthToRT_NoScreen.cs`

**Role:** GPU depth extraction (off-screen)

- Attached to a Unity Camera
- Converts Unity depth buffer into linear eye-space depth (meters)
- Writes depth into an `RFloat` RenderTexture
- Uses a CommandBuffer and shader `Hidden/LinearEyeDepth_CMD`
- No screen rendering and no CPU readback

**Output:** `RenderTexture` (RFloat) containing depth in meters

---

### `OakLitePublisher.cs`

**Role:** Simulated onboard RGB-D camera (Unity → ROS)

Mimics an OAK-Lite–style RGB-D interface:
- Publishes RGB image
- Publishes depth image
- Publishes camera intrinsics (CameraInfo)

#### Published ROS Topics

| Topic | Message Type | Format |
|-------|-------------|---------|
| `/oak/rgb/image_raw` | `sensor_msgs/Image` | rgb8 |
| `/oak/depth/image_rect_raw` | `sensor_msgs/Image` | 32FC1 |
| `/oak/rgb/camera_info` | `sensor_msgs/CameraInfo` | - |
| `/oak/depth/camera_info` | `sensor_msgs/CameraInfo` | - |

> **Note:** Unity publishes raw sensor data only. No obstacle detection or decision logic is performed in Unity.

---

### `StaticCamDepthPublisher.cs`

**Role:** Static environment depth sensor

- Represents a fixed depth camera in the Unity scene
- Publishes depth images and camera intrinsics
- Used for environment monitoring or ground-truth perception

#### Published ROS Topics

| Topic | Message Type | Format |
|-------|-------------|---------|
| `/static_cam/depth/image_raw` | `sensor_msgs/Image` | 32FC1 |
| `/static_cam/depth/camera_info` | `sensor_msgs/CameraInfo` | - |

---

### `StaticCamTargetGPSPublisher.cs` 

**Role:** Visual target → GPS shortcut (PyBullet-era logic)

- Samples depth at the image center
- Back-projects pixel → camera → world → ENU → GPS
- Publishes estimated target GPS position

#### Published ROS Topic

| Topic | Message Type |
|-------|-------------|
| `/static_cam/target_gps` | `sensor_msgs/NavSatFix` |

**Status:** 
- Legacy / experimental
- Strongly coupled to PyBullet-era world assumptions
- Not used in the current architecture
- Kept for reference only

---

## ROS 2 Components

### `odom_bridge`

**Role:** PX4 → Unity odometry translation

- Subscribes to PX4 `VehicleOdometry`
- Publishes standard ROS `nav_msgs/Odometry`
- Handles QoS compatibility between PX4 DDS and ROS 2

**Data Flow:**
```
/fmu/out/vehicle_odometry (px4_msgs) → nav_msgs/Odometry
```

---

## Runtime Startup Instructions

### 1. Start PX4 SITL

```bash
make px4_sitl gz_x500
```

Launches PX4 Software-In-The-Loop with the default x500 model.

---

### 2. Start Micro XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

This bridges the built-in PX4 uXRCE client with ROS 2. Only the Agent is required; PX4 already includes the client.

---

### 3. Start ROS-TCP-Endpoint Server

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args \
  -p ROS_IP:=192.168.1.114 \
  -p ROS_TCP_PORT:=10000
```

Starts a TCP server that Unity connects to via sockets.

---

### 4. Start PX4 → Unity Odometry Bridge

```bash
ros2 run px4_to_unity odom_bridge
```

Translates PX4 odometry into standard ROS format for Unity.

---

### 5. Connect QGroundControl

```bash
mavlink start -x -u 14550 -r 40000 -t 192.168.1.125
```

Enables monitoring and manual interaction via QGroundControl.

---

## 📊 ROS 2 Useful Commands

### List all active topics
```bash
ros2 topic list
```

### Inspect a specific topic
```bash
ros2 topic echo /oak/depth/image_rect_raw
```

### Show topic message type
```bash
ros2 topic info /oak/rgb/image_raw
```

### List active ROS nodes
```bash
ros2 node list
```

---

## Design Principles

- Unity does not handle dynamics or control
- PX4 remains the single source of truth for vehicle state
- ROS 2 handles all data translation and higher-level logic
- Sensor simulation is modular and replaceable
- Legacy PyBullet shortcuts are explicitly isolated

---

## Notes

- All Unity scenes, prefabs, and `.meta` files are committed to preserve exact configuration
- `ProjectSettings` and `Packages` are included for reproducibility
- Depth data is metric (meters) and linearly encoded (32FC1)
- Coordinate frame conventions and assumptions are documented in code comments

---

## License

[Add your license here]

## Contributing

[Add contribution guidelines here]
