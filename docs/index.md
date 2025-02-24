---
title: UR5 & TurtleBot Collaboration using ROS2
tags:
- robotics
- ROS2
- UR5
- TurtleBot
---

# UR5 & TurtleBot Collaboration using ROS2

## Project Overview

- **Team Number**: 05
- **Team Members**: Asrith Pandreka, Varun Karthik, Anjali Notani
- **Semester and Year**: Spring 2025
- **University, Class, Professor**: Arizona State University, RAS 598, Professor Daniel Aukes

## Research Question

This project explores the collaborative capabilities between the UR5 robotic arm and TurtleBot using ROS2. The main objective is to design an integrated system where UR5 can pick and place objects, while TurtleBot navigates autonomously to interact with these objects. The research question focuses on optimizing inter-robot communication, improving object detection accuracy, and ensuring robust navigation in dynamic environments.

## System Architecture

### Hardware Requirements

- UR5 robotic arm (equipped with a gripper and camera)
- TurtleBot3 Waffle Pi
- RGB-D Camera (Intel Realsense D435, Azure Kinect, or Orbbec Astra Pro)
- LiDAR (RPLiDAR A2M8 or Hokuyo) for TurtleBot navigation
- Computing Units: Raspberry Pi 4 or Jetson Nano for TurtleBot, and a PC for UR5 control

### Software Requirements

- ROS2 Humble/Foxy
- MoveIt2 (UR5 motion planning)
- Nav2 (TurtleBot navigation)
- Gazebo/Ignition (simulation)
- YOLOv5/Mask R-CNN/OpenCV (object detection)
- DDS (ROS2 inter-robot communication framework)

## Sensor Integration

### How Sensors Are Utilized

- **Object Detection**: UR5’s camera detects objects using YOLOv5 and OpenCV.
- **Navigation**: TurtleBot’s LiDAR and RGB-D camera are used for SLAM and object verification.
- **Communication**: DDS enables real-time data exchange between UR5 and TurtleBot.

## Interaction and Control

- UR5 manipulator picks and places the object.
- TurtleBot navigates to the drop location and verifies object placement.
- TurtleBot either picks up the object or signals task completion.

## Step-by-Step Implementation

### UR5 Robot Setup

1. Install UR5 Drivers & MoveIt2:
   ```bash
   sudo apt install ros-humble-ur5e-description
   sudo apt install ros-humble-moveit