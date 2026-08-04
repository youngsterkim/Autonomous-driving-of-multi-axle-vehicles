# Autonomous Driving System for Multi-Axle Vehicles

An **ROS**-based autonomous driving system for multi-axle vehicles, integrating a complete autonomy stack — **Perception → Localization → Planning → Control → Chassis Execution** — for a three-axle, six-wheel vehicle (front, middle, and rear axles are all steerable).

---

## System Architecture

```
┌───────────────────────────────────────────────────────────────────────────┐
│                           SENSOR LAYER                                    │
│   Robosense LiDAR  ──►  rs_to_velodyne  ──►  ASENSING GNSS/INS           │
└──────────┬──────────────────────────────┬──────────────────┬───────────────┘
           │ /velodyne_points             │                  │ /IMU
           ▼                              ▼                  ▼
   ┌────────────────┐            ┌──────────────┐     ┌──────────────┐
   │ lidarObstac    │            │  FAST_LIO    │◄────┤  INS Driver  │
   │ Obstacle Detect│            │ LiDAR-Intertial│    └──────────────┘
   │  & Tracking    │            │  Odometry    │
   └───────┬────────┘            └──────┬───────┘
           │ /detection/.../objects     │ /state_to_control
           ▼                            ▼
   ┌───────────────────────────────────────────────────┐
   │            Local_path (Hybrid A*)                  │
   │  local costmap + global path + vehicle state       │
   │                ──► local path                     │
   └───────────────────────────┬───────────────────────┘
                               │ /local_path
                               ▼
   ┌───────────────────────────────────────────────────┐
   │         PurePursuit_control (Pure Pursuit)         │
   │   preview model + pure pursuit + multi-axle        │
   │             Ackermann steering split               │
   └───────────────────────────┬───────────────────────┘
                               │ /decision/steering_angle
                               ▼
   ┌───────────────────────────────────────────────────┐
   │              can_ros (CAN Communication)           │
   │   front/middle/rear steering angles, throttle      │
   │                   ──► chassis                      │
   └───────────────────────────────────────────────────┘
```

**Data flow:** LiDAR point clouds and IMU data are fused by FAST_LIO to output vehicle pose; the obstacle detection module outputs a list of detected objects. Combined with the global path, Hybrid A* plans a local path that satisfies vehicle kinematic constraints. The Pure Pursuit controller then computes the front-axle steering angle, distributes it across the three steering axles via multi-axle Ackermann steering geometry, and finally sends the commands to the chassis over CAN.

---

## Modules

| Module | Function | Core Algorithm / Technology |
|--------|----------|----------------------------|
| **FAST_LIO** | LiDAR-inertial odometry / mapping | Iterated Extended Kalman Filter (IEKF) + ikd-Tree incremental mapping |
| **INS** | ASENSING GNSS/INS driver | Serial protocol parsing; outputs position, attitude, sideslip angle, yaw rate |
| **rs_to_velodyne** | Point cloud format conversion | Robosense point cloud → Velodyne format (XYZIRT / XYZI) |
| **lidarObstac** | Obstacle detection & tracking | Patchwork concentric-zone ground segmentation + Euclidean clustering + bounding-box fitting + Hungarian matching / Kalman filter tracking |
| **Local_path** | Local path planning | Hybrid A* search + Reeds-Shepp analytic expansion |
| **PurePursuit_control** | Path tracking control | Preview model + Pure Pursuit + multi-axle Ackermann steering split |
| **can_ros** | CAN bus communication | Kvaser CANlib driver; sends steering angle / throttle, reads chassis feedback |
| **rviz** | Visualization | Vehicle model, safety region, local path, search tree |

---

## Hardware Requirements

- **Vehicle platform:** three-axle, six-wheel vehicle; front / middle / rear axles all steerable (multi-axle Ackermann steering geometry)
- **LiDAR:** Robosense series (RS-16 / RS-32 / RS-Ruby / RS-Helios, etc.)
- **GNSS/INS:** ASENSING series (serial connection, default `/dev/ttyUSB0`)
- **CAN bus:** Kvaser CAN adapter (requires Kvaser CANlib)

---

## Software Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| Ubuntu | 16.04 / 18.04 | Operating system |
| ROS | Melodic / Noetic | Communication framework |
| PCL | >= 1.8 | Point cloud processing |
| Eigen | >= 3.3.4 | Linear algebra |
| autoware_msgs | - | Message types such as `DetectedObjectArray` |
| rslidar_sdk | - | Robosense LiDAR driver |
| livox_ros_driver | - | FAST_LIO dependency (must be sourced first) |
| Kvaser CANlib | - | CAN communication |

---

## Quick Start

### 1. Build

Copy the ROS packages in this repository into a catkin workspace and build:

```bash
cd ~/catkin_ws/src
# Copy FAST_LIO-main, INS, Local_path, PurePursuit_control,
# can_ros, lidarObstac, rs_to_velodyne from this repo into this directory
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

> FAST_LIO requires installing and sourcing `livox_ros_driver` (even when using Velodyne-format point clouds), and running
> `git submodule update --init` to fetch the ikd-Tree submodule.

### 2. Run

Start the nodes in the following order:

```bash
# ① LiDAR driver + point cloud format conversion
roslaunch rslidar_sdk start.launch
rosrun rs_to_velodyne rs_to_velodyne XYZIRT XYZIRT

# ② GNSS/INS
roslaunch ins demo.launch    # make sure the serial port is /dev/ttyUSB0

# ③ LiDAR-inertial odometry (localization)
roslaunch fast_lio mapping_velodyne.launch

# ④ Obstacle detection
roslaunch lidar_detection_track lidar_detection_track.launch

# ⑤ Local path planning (Hybrid A*)
rosrun hybrid_astar hybrid_astar

# ⑥ Pure Pursuit control
rosrun PurePursuit_control PurePursuit_control_node

# ⑦ CAN chassis communication
rosrun can_ros can_all
```

---

## Core Algorithms

### Hybrid A* Local Path Planning (`Local_path`)

Searches in the continuous state space of **position + heading angle**, generating kinematically feasible nodes via the vehicle motion model:

- **State-space search:** nodes are expanded with a vehicle kinematic model (`DynamicModel`); steering angle and segment length are discretized
- **Cost design:** steering penalty `steering_penalty`, reversing penalty `reversing_penalty`, steering-change penalty `steering_change_penalty`
- **Reeds-Shepp analytic expansion:** near the goal, RS curves that satisfy kinematic constraints are generated directly, significantly accelerating convergence
- **Collision checking:** ray casting + discretized rectangular vehicle footprint; search-tree visualization available (`/searched_tree`)

### Pure Pursuit Control (`PurePursuit_control`)

A classic look-ahead-point path tracking algorithm. The front-axle steering angle is computed as:

```
δ = atan2(2 · L · e_y, L_d²)
```

where `L` is the wheelbase, `L_d` is the look-ahead distance, and `e_y` is the lateral error. The controller additionally combines:

- **Preview model** (`Preview_error`): dynamically computes the look-ahead distance from vehicle speed; derives the ideal yaw rate and ideal sideslip angle
- **Multi-axle Ackermann steering split** (`Akm_get_delta`): derives the middle / rear axle angles from the front-axle angle, satisfying multi-axle steering geometry constraints

### Obstacle Detection (`lidarObstac`)

- **Patchwork ground segmentation:** divides the point cloud into concentric-zone sectors, fits a ground plane per region with adaptive thresholds, robustly separating ground from non-ground points
- **Euclidean clustering + bounding box:** clusters non-ground points and generates obstacle bounding boxes
- **Multi-object tracking:** Hungarian algorithm for data association + Kalman filtering, producing stable object tracks

---

## ROS Topics

| Topic | Message Type | Direction | Description |
|-------|--------------|-----------|-------------|
| `/rslidar_points` | `sensor_msgs/PointCloud2` | Input | Raw Robosense point cloud |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Output | Velodyne-format point cloud |
| `/state_to_control` | `fast_lio::Position_state` | Output | Localization state (position / attitude / sideslip / yaw rate) |
| `/detection/lidar_detector/objects` | `autoware_msgs::DetectedObjectArray` | Output | Detected object list |
| `/local_path` | `nav_msgs/Path` | Output | Planned local path |
| `/local_map` | `nav_msgs/OccupancyGrid` | Output | Local costmap (obstacles projected) |
| `/global_path` | `nav_msgs/Path` | Output | Global reference path (loaded from a txt file) |
| `/searched_tree` | `visualization_msgs/Marker` | Output | Hybrid A* search-tree visualization |
| `/decision/steering_angle` | `can_ros::send_can` | Output | Front / middle / rear axle steering angles, throttle |
| `/car_cube` | `visualization_msgs/Marker` | Output | Vehicle model visualization |

> Custom messages: `fast_lio::Position_state`, `can_ros::send_can` / `can_ros::read_can`.

---

## Repository Structure

```
Autonomous-driving-of-multi-axle-vehicles-main
├── FAST_LIO-main/            # LiDAR-inertial odometry & mapping
├── INS/                      # ASENSING GNSS/INS driver
├── Local_path/               # Hybrid A* local path planning
│   └── src/hybrid_astar/     #   ├── hybrid_astar_searcher (search algorithm)
│                             #   └── rs_path (Reeds-Shepp curves)
├── PurePursuit_control/      # Pure Pursuit control
│   └── src/PurePursuit_control/
│                             #   ├── purepursuit_controler (Pure Pursuit controller)
│                             #   └── Preview_error (preview model)
├── lidarObstac/              # LiDAR obstacle detection & tracking
│   └── src/lidarObstacleDetect-main/
│                             #   ├── ground_detector/patchwork
│                             #   ├── cluster/euclideanCluster
│                             #   └── bounding_box
├── can_ros/                  # CAN bus communication
├── rs_to_velodyne/           # Robosense → Velodyne point cloud conversion
└── rviz/                     # rviz visualization config
```

---

## Notes

- The local planner **hardcodes the global-path file path** (it reads `/home/kim/Documents/.../path/lla_path.txt` or `easy_path.txt` by default). Adjust it to your environment in [hybrid_astar_main.cpp](Local_path/src/hybrid_astar/src/hybrid_astar_main.cpp) before deployment.
- The control node runs at **10 Hz**; steering angle commands are clamped to ±20°.
- Before real-vehicle tests, verify CAN device permissions and that the chassis protocol matches.

---

## License

The open-source licenses differ across the modules in this repository:

- **FAST_LIO** is a third-party open-source project and follows its original license (see [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO))
- **lidarObstac**, **rs_to_velodyne**, etc. are also third-party open-source projects and follow their respective original licenses
- The license for the **remaining code written in this repository** is left for you to choose and declare (the `license` field in the current `package.xml` files is still a placeholder)

> ⚠️ Before publishing, please review and fill in the `maintainer` and `license` fields of every `package.xml`.

---

## Acknowledgements

This system integrates the following excellent open-source projects:

- [FAST_LIO](https://github.com/hku-mars/FAST_LIO) — Fast LiDAR-Inertial Odometry
- [lidarObstacleDetect](https://github.com/ORiN-Group/lidarObstacleDetect) — LiDAR obstacle detection & tracking
- [rs_to_velodyne](https://github.com/HViktorTsoi/rs_to_velodyne) — Point cloud format conversion tool
- [ASENSING INS Driver](https://github.com/ASENSING/ASENSING_INS_Driver) — GNSS/INS driver
