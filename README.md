# PnP RGB-D Visual-Inertial Odometry (ROS 2)

A minimal **RGB-D Visual-Inertial Odometry (VIO)** implementation for **ROS 2**, combining
ORB feature extraction, RGB-D back-projection, Perspective-n-Point (PnP) pose
estimation, and IMU orientation fusion.

This repository is intended for **research, prototyping, and educational use**.

---

## Overview

The system estimates the robot pose by:
1. Extracting ORB features from the RGB image
2. Back-projecting features using depth data
3. Estimating translation using PnP
4. Fusing orientation from the IMU
5. Publishing odometry and TF

Translation is estimated visually, while rotation currently comes directly from the IMU.

---

## Features

- RGB-D motion estimation using OpenCV PnP
- ORB feature extraction
- IMU orientation fusion
- Publishes `nav_msgs/Odometry`
- Broadcasts TF (`odom → base_link`)
- Lightweight and easy to extend

---

## ROS Interfaces

### Subscribed Topics

| Topic | Message Type |
|------|--------------|
| `/camera/image` | `sensor_msgs/Image` |
| `/camera/depth_image` | `sensor_msgs/Image` |
| `/imu` | `sensor_msgs/Imu` |

### Published Topics

| Topic | Message Type |
|------|--------------|
| `/odom` | `nav_msgs/Odometry` |

### TF Frames

odom → base_link


---

## Algorithm Pipeline

RGB Image ──► ORB Features ──► RGB-D Back-Projection ──► solvePnP ──► Translation
└─► IMU Orientation
│
▼
Odometry + TF Output


---

## Dependencies

- ROS 2 (Humble or newer)
- OpenCV
- cv_bridge
- tf2_ros

---

## Build

--bash
colcon build --packages-select pnp_rgbd_vio
source install/setup.bash

---

## Run

ros2 run pnp_rgbd_vio pnp_rgbd_vio_node


## Configuration Notes

##Camera Intrinsics
Camera intrinsics are currently hardcoded in the source:
fx = 525, fy = 525
cx = 319.5, cy = 239.5
Replace these values with those from your camera calibration.

---

## Depth Image Format
Depth image must be 32-bit float
Units must be meters

---

## Limitations

-No loop closure
-No keyframe selection or feature matching
-No IMU preintegration
-Accumulated drift over time
-Sensitive to depth noise

This project is not a full SLAM system.

---
