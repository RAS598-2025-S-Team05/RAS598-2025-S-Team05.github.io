---
title: Index
tags:
- robotics
- ROS2
- UR5
- TurtleBot
---

# Coordinated Autonomy: UR5 & TurtleBot Navigating the Future Together

## About Us

- **Team Number**: 05
- **Team Members**: Asrith Pandreka, Varun Karthik, Anjali Notani
- **Semester and Year**: Spring 2025
- **University**: Arizona State University
- **Class**: RAS598 Experimentation and Deployment of Robotics
- **Advising Faculty**: Dr. Daniel Aukes

## 1. Introduction

### Project Overview

Our team's project revolves around the integration of two distinct robotic systems, the UR5 robotic arm and the TurtleBot, to perform collaborative object manipulation and retrieval tasks. The essence of the project lies in the ability of the UR5 to precisely place an object at a random, but predetermined location and then communicate the object’s coordinates to the TurtleBot. Utilizing its onboard camera and LiDAR systems, the TurtleBot then autonomously navigates to the location to retrieve or interact with the object. This setup exemplifies an advanced use of robotic collaboration, highlighting the potential for autonomous systems to work together to achieve complex tasks.

The core research question of this project is: "How can two autonomous robotic systems be effectively integrated to collaborate on complex physical tasks involving object manipulation and retrieval in dynamic environments?" This study explores the potential for combining stationary and mobile robots to enhance task execution that single robots cannot perform alone. It focuses on the integration of robotic arms, mobile platforms, precise localization, and autonomous navigation within the Robot Operating System (ROS) framework. Additionally, the research assesses the system's adaptability to new and changing environments, essential for real-world applications.

### Project Architecture Diagram

## 3. Hardware and Software Requirements

### Hardware Components

- **UR5 Robotic Arm**
  - Used for object manipulation.
  - Requires external camera for vision-based object localization.

- **TurtleBot 4**
  - Equipped with LiDAR, IMU, and Camera for navigation.
  - Runs on a Jetson Nano (or Raspberry Pi) with ROS2.

- **External Camera**
  - Used for detecting and localizing the object after UR5 placement.

- **Compute System**
  - A laptop/workstation with Ubuntu and ROS2 installed for running the overall system.

- **Wireless Communication**
  - TCP/IP or ROS2 communication between UR5 and TurtleBot.

### Software Components

- **Operating System:** Ubuntu 22.04 (for ROS2 support)
- **Robotics Framework:** ROS2 (Humble)
- **Simulation & Visualization:** RViz, Gazebo, or Webots
- **Computer Vision:** OpenCV for object detection using the external camera
- **SLAM & Navigation:**
  - SLAM Toolbox for mapping the environment
  - Nav2 Stack for path planning

### Custom ROS2 Nodes:
- Node for UR5 Object Placement & Detection
- Node for TurtleBot Navigation to Target
- Communication node using ROS2 topics/services

## 4. Sensor Integration and Interaction

| Sensor | Function in Project |
| ------ | ------------------- |
| Camera (External) | Detects the object and computes its position in the UR5 frame |
| Camera (TurtleBot 4) | Detects obstacles, supports localization |
| LiDAR (TurtleBot 4) | Helps in SLAM and obstacle detection |
| IMU (TurtleBot 4) | Provides orientation data for navigation |
| Encoders (UR5 & TurtleBot) | Measures movement and ensures accurate positioning |

### Interaction

- **UR5 Object Detection & Placement**
  - Use the external camera to detect the object.
  - Determine the placement location.
  - Move the UR5 to place the object at a random position.
  - Record the object’s final position using vision-based estimation.

- **UR5 to TurtleBot Communication**
  - Use ROS2 services or messages to send the object’s coordinates to TurtleBot.

- **TurtleBot Navigation to Object**
  - Utilize LiDAR and camera to localize itself in the environment.
  - Use SLAM Toolbox to map the environment.
  - Implement Nav2 stack for path planning.
  - Use camera-based object detection for final approach.

- **Final Object Verification**
  - Upon reaching the coordinates, the TurtleBot’s camera verifies the object’s presence.


## 5. Control and Autonomy

### UR5 Control:
- The robotic arm will follow predefined motion paths for picking and placing the object using MoveIt! in ROS2.
- An external camera will verify object placement.

### TurtleBot 4 Navigation:
- TurtleBot will use SLAM Toolbox to map the environment.
- The Nav2 Stack will generate and follow a navigation path.
- Camera-based object recognition will help in final approach adjustments.

### Decision Making:
- The system will autonomously determine the best path for TurtleBot.
- UR5 will confirm object placement before sending coordinates.

## 6. Preparation Needs

To successfully execute this project, the following preparations are necessary:

### ROS2 Environment Setup:
- Installation of ROS2 Humble on Ubuntu.
- Configuration of ROS2 workspaces for UR5 and TurtleBot.

### Hardware Testing:
- Calibration of the external camera and LiDAR.
- Testing UR5 movement and TurtleBot navigation.

### Software Development:
- Writing ROS2 nodes for communication.
- Implementing object detection using OpenCV.

### Simulation Setup:
- Running Gazebo simulations before physical implementation.  

## 7. Final Demonstration Plan

The final demonstration will consist of:

### Initial Setup:
- TurtleBot and UR5 are placed in an indoor environment with obstacles.
- The object is placed in UR5's workspace by a human.

### Execution:
- UR5 picks the object and places it at a random location.
- External camera detects the object's final position and sends coordinates to TurtleBot.
- TurtleBot navigates to the object’s location using LiDAR and Nav2.
- TurtleBot verifies object detection using its onboard camera.

### Performance Metrics:
- Success rate of object retrieval.
- Accuracy of position estimation.
- Navigation efficiency (time taken, obstacle avoidance).

## 8. Required Resources

### Hardware:
- UR5 robotic arm with a gripper and RGB-D camera.
- TurtleBot3 with LiDAR and RGB-D camera.
- A PC for UR5 control and a Raspberry Pi/Jetson Nano for TurtleBot.

### Software:
- ROS2 Humble/Foxy.
- MoveIt2, Nav2, Gazebo, and OpenCV.

### Environment:
- A small, area for the testing and demonstration.
- Objects for the UR5 to pick and place.

## 9. Testing & Evaluation Plan

### **1. Unit Testing**
- Test UR5 pick-and-place, TurtleBot navigation, and sensor accuracy.

### **2. Integration Testing**
- Validate system performance in simulation and real-world scenarios.

### **3. Performance Metrics**
- Measure task completion time, object detection accuracy, and navigation success.

### **4. Error Handling**
- Implement recovery strategies for object detection failures and navigation errors.

### **5. Final Demonstration**
- Showcase end-to-end system functionality, obstacle handling, and adaptability.

## 10. Project Impact

The project has the following impacts:

- **Advancement in Multi-Robot Collaboration:** Demonstrates the integration of robotic arms with mobile robots.
- **Applications in Warehousing & Logistics:** The system can be adapted for autonomous material handling.
- **Research & Development:**
  - Investigates real-time sensor fusion techniques.
  - Improves object localization and robot perception strategies.
- **Educational Value:** The project will serve as a learning tool for ROS2, SLAM, and perception systems.

## 11. Advising & Resource Needs

- **Advisor:** Prof. Daniel M Aukes

### Advisor's Role and Contribution
Prof. Daniel M Aukes, an esteemed faculty member with deep expertise in SLAM (Simultaneous Localization and Mapping) and multi-robot coordination, will serve as the primary advisor for this project. Prof. Aukes brings a wealth of experience in robotic systems design and implementation, which will be invaluable for guiding the technical development and integration aspects of our project. His prior work, focusing on the application of innovative robotics solutions in real-world scenarios, will provide critical insights into our design and execution phases. 

Prof. Aukes will also assist in troubleshooting complex system integration issues, ensuring that both the UR5 robotic arm and the TurtleBot function harmoniously. Additionally, he will provide access to laboratory resources and equipment, facilitate networking with other robotics experts, and help in acquiring necessary project funding. His mentorship will be crucial in steering the project towards successful completion and ensuring that the team remains aligned with the latest advancements in robotics technology.


