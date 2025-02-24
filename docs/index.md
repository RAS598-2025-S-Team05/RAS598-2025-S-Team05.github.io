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

## UR5 Robot Setup

### Configure URDF for Attaching a Camera and Gripper

To integrate the camera and gripper with the UR5 robotic arm, follow these steps:

1. **Modify the URDF**:
   - Update the UR5's URDF file to include the camera and gripper models.
   - Ensure the kinematic chain is correctly defined for motion planning.

2. **Test the Configuration**:
   - Use RViz to visualize the UR5 with the attached camera and gripper.
   - Verify that the robot can move as expected with the new components.

### Implement `ur5_vision_node` for Object Detection

1. **Install Dependencies**:
   - Install OpenCV and YOLOv5 for object detection:
     ```bash
     pip install opencv-python
     pip install yolov5
     ```

2. **Develop the Node**:
   - Create a ROS2 node (`ur5_vision_node`) that subscribes to the camera feed and performs object detection.
   - Publish the detected object's position to a ROS2 topic (e.g., `/detected_object`).

### Implement `ur5_pick_place_node` for Pick-and-Place Routine

1. **Develop the Node**:
   - Create a ROS2 node (`ur5_pick_place_node`) that subscribes to `/detected_object` and plans the pick-and-place motion using MoveIt2.
   - Execute the motion plan and confirm successful object manipulation.

2. **Test the Routine**:
   - Simulate the pick-and-place routine in Gazebo.
   - Verify that the UR5 can reliably pick and place objects.

---

## TurtleBot Navigation Setup

### Install Nav2 & SLAM Packages

1. **Install Required Packages**:
   ```bash
   sudo apt install ros-humble-nav2-bringup
   sudo apt install ros-humble-turtlebot3-bringup