---
title: Index
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
- **University**: Arizona State University
- **Class**: RAS598 Experimentation and Deployment of Robotics
- **Advising Faculty**: Dr. Daniel Aukes

# Introduction

Our project explores the collaborative capabilities of a UR5 robotic arm and a TurtleBot using ROS2. The central research question is: How can we design an integrated system where the UR5 and TurtleBot work together to perform complex tasks such as object manipulation and navigation in dynamic environments? Specifically, the UR5 will pick and place objects at predefined locations, while the TurtleBot will autonomously navigate to these locations, verify the object's presence, and optionally pick it up. This project aims to optimize inter-robot communication, improve object detection accuracy, and ensure robust navigation in real-world scenarios. By leveraging ROS2's advanced features, such as MoveIt2 for motion planning and Nav2 for navigation, we aim to create a seamless collaboration between the two robots, paving the way for applications in industrial automation, logistics, and beyond.

## Example figure


# Sensor Integration

Sensor data is critical for enabling the UR5 and TurtleBot to perform their tasks effectively. The UR5’s RGB-D camera will be used for object detection, with the ur5_vision_node processing the camera feed using YOLO/OpenCV to detect objects and publish their positions to the /object_detected topic. The gripper sensors on the UR5 will provide feedback to ensure successful grasping and releasing of objects, which will be used in the ur5_pick_place_node to confirm task completion. On the TurtleBot, the LiDAR will be used for SLAM (Simultaneous Localization and Mapping) and navigation, with the turtlebot_navigation_node subscribing to LiDAR data to create a map of the environment and plan paths. The TurtleBot’s RGB-D camera will verify the presence of the object after the UR5 places it, with the turtlebot_vision_node processing the camera feed and publishing verification results to the /object_verified topic. ROS2 topics will facilitate communication between the UR5 and TurtleBot, ensuring seamless integration of sensor data.

During testing, sensors will be used to validate the functionality of each component and ensure the system works as expected. In simulation (Gazebo), simulated sensor data, such as LiDAR and RGB-D camera feeds, will be used to test the UR5 and TurtleBot in a virtual environment. Mock object detection nodes will simulate object placement and verification. In hardware testing, the UR5’s RGB-D camera and gripper sensors will be tested for accuracy and reliability, while the TurtleBot’s LiDAR and RGB-D camera will be tested for navigation and object verification. Sensor data will be logged and analyzed to identify and fix issues. Integration testing will involve testing the communication between the UR5 and TurtleBot using real sensor data, such as the UR5 placing an object and the TurtleBot navigating to and verifying the object.

In the final demonstration, sensors will play a crucial role in showcasing the system’s capabilities. The UR5’s RGB-D camera will detect an object in the environment, and the gripper will pick it up, with sensor feedback confirming successful grasping and placement. The TurtleBot’s LiDAR will create a map of the environment and navigate to the object’s location, while the RGB-D camera will verify the object’s presence and confirm task completion. Real-time sensor data, such as object detection results and navigation status, will be displayed on a web-based UI, allowing the audience to see how the robots use sensor data to perform their tasks. If sensors detect an issue, such as an object not being found or a navigation failure, the system will handle it gracefully, with the TurtleBot reattempting navigation or signaling an error. This approach ensures that sensor data is effectively utilized throughout the project lifecycle, from development to final demonstration.


# Interaction

## Influencing Robot Behavior

We will influence the behavior of the UR5 and TurtleBot through a combination of pre-programmed tasks, real-time sensor feedback, and user interaction. The robots will follow a predefined workflow, such as the UR5 picking and placing objects and the TurtleBot navigating to verify the object. Their behavior will dynamically adjust based on sensor data (e.g., object detection, navigation status) and user inputs. For example, if the TurtleBot fails to verify an object, it will reattempt navigation or signal an error. ROS2 nodes, such as turtlebot_control_node and ur5_pick_place_node, will process sensor data and execute decision-making logic to ensure smooth operation.

## Interfaces for Viewing,Interaction and Data Storage

We will develop a web-based dashboard for real-time monitoring and control, displaying camera feeds, navigation maps, and system logs. Users can start/stop tasks, view sensor data, and access performance metrics. Additionally, we will use ROS2 CLI tools for launching nodes, debugging, and testing. Sensor data and system logs will be stored using ROSbag and visualized with tools like PlotJuggler or custom scripts. These interfaces will enable seamless interaction, real-time monitoring, and post-processing analysis.


# Control and Autonomy

We will connect sensor feedback to the controllers and higher-level decision-making processes through ROS2 nodes and real-time communication. For the UR5, sensor data from the RGB-D camera and gripper will be processed by the ur5_vision_node and ur5_pick_place_node to detect objects and execute pick-and-place tasks. For the TurtleBot, LiDAR and RGB-D camera data will be used by the turtlebot_navigation_node and turtlebot_vision_node for navigation and object verification. Higher-level decision-making will occur in nodes like turtlebot_control_node, which will subscribe to topics like /object_verified to decide the next action. This modular approach ensures that sensor feedback directly influences both low-level control (e.g., motion planning) and high-level decision-making (e.g., task sequencing).

# Preperation Needs

To be successful, we need a strong understanding of ROS2 fundamentals, including node communication, topic publishing/subscribing, and service calls. We also need expertise in robot motion planning (e.g., MoveIt2 for UR5) and navigation (e.g., Nav2 for TurtleBot). Knowledge of object detection algorithms (e.g., YOLO, OpenCV) and sensor integration (e.g., LiDAR, RGB-D cameras) is essential. Additionally, familiarity with simulation tools like Gazebo and debugging techniques for ROS2 systems will be critical.

In class, we need coverage of ROS2 advanced concepts, such as multi-robot communication and DDS, as well as practical sessions on motion planning and navigation. Hands-on tutorials for sensor integration and object detection would also be highly beneficial. These topics will provide the foundational knowledge and skills required to implement and debug our project effectively.  

# Final Demonstration: Showcasing the Project

## How Will You Demonstrate Your Work in Class?

We will demonstrate the collaboration between the UR5 and TurtleBot in a controlled environment. The UR5 will pick up an object from a predefined location and place it at a target location. The TurtleBot will then navigate to the object’s location using SLAM, verify the object’s presence using its RGB-D camera, and optionally pick it up. Real-time performance data, such as object detection results and navigation status, will be displayed on a web-based dashboard. This will showcase the seamless integration of the two robots and their ability to work together autonomously.

---

## What Resources Will You Need?

### Hardware:
- UR5 robotic arm with a gripper and RGB-D camera.
- TurtleBot3 with LiDAR and RGB-D camera.
- A PC for UR5 control and a Raspberry Pi/Jetson Nano for TurtleBot.

### Software:
- ROS2 Humble/Foxy.
- MoveIt2, Nav2, Gazebo, and OpenCV/YOLO.

### Environment:
- A small, obstacle-free area for the demonstration.
- Objects for the UR5 to pick and place.

---

## Classroom Setup Requirements

- **Space**: A clear area of at least 3m x 3m for the robots to operate.
- **Power Supply**: Access to power outlets for the UR5, TurtleBot, and computing devices.
- **Projector/Screen**: To display the web-based dashboard for real-time monitoring.
- **Wi-Fi**: Stable internet connection for ROS2 communication and dashboard access.

---

## Handling Variability in the Environment

The robots are designed to handle variability through **adaptive algorithms** and **error recovery mechanisms**:

- **UR5**: If the object is not detected or the gripper fails to grasp, the UR5 will retry the operation or signal an error.
- **TurtleBot**: If the environment changes (e.g., new obstacles), the TurtleBot will use SLAM to update its map and replan its path. If object verification fails, it will reattempt navigation or signal an error.
- **Real-Time Adjustments**: The system will dynamically adjust based on sensor feedback, ensuring robustness in varying conditions.

---

## Testing & Evaluation Plan

1. **Unit Testing**:
   - Test individual components (e.g., UR5 pick-and-place, TurtleBot navigation) in isolation using Gazebo and real hardware.

2. **Integration Testing**:
   - Test the full workflow in simulation and real-world environments to ensure seamless communication between the UR5 and TurtleBot.

3. **Performance Metrics**:
   - Measure task completion time, object detection accuracy, and navigation success rate.

4. **Error Handling**:
   - Evaluate the system’s ability to recover from errors (e.g., object not found, navigation failure).

5. **Final Demonstration**:
   - Execute the full workflow in class, showcasing the robots’ ability to collaborate and adapt to changes.


# Impact

This project has a significant impact on both personal learning and course development. By integrating the UR5 and TurtleBot using ROS2, we gain hands-on experience in collaborative robotics, real-time communication, and autonomous decision-making. This drives us to learn new material, such as advanced ROS2 concepts (e.g., DDS, multi-agent systems), object detection algorithms (e.g., YOLO, OpenCV), and motion planning (e.g., MoveIt2). The project also encourages exploration of sensor integration (e.g., LiDAR, RGB-D cameras) and simulation tools (e.g., Gazebo), which are essential for robotics development.

For course development, this project serves as a practical example of multi-robot collaboration, providing a framework for future students to build upon. It highlights the importance of modular design, error handling, and real-time adaptability, which are critical skills in robotics. Additionally, the project demonstrates the potential of ROS2 in industrial and research applications, inspiring further exploration of automation and AI-driven robotics. Overall, this work not only enhances our technical expertise but also contributes to a richer, more applied learning experience for the entire course.

 # Impact

    Dr. Aukes will serve as our project advisor, providing mentorship, technical guidance, and access to specialized hardware and resources. We will rely on Dr. Aukes for expertise in ROS2 development, robot motion planning, and system integration. Specifically, we need access to the UR5 robotic arm, TurtleBot3, and sensors (e.g., LiDAR, RGB-D cameras) available in the lab. Additionally, we will seek guidance on best practices for collaborative robotics and error handling strategies.


