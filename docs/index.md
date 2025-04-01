---
title: Index
tags:
- robotics
- ROS2
- UR5
- TurtleBot
---

# Coordinated Autonomy: UR5 & TurtleBot Navigating the Future Together

## **About Us**

- **Team Number**: 05
- **Team Members**: Asrith Pandreka, Varun Karthik, Anjali Notani
- **Semester and Year**: Spring 2025
- **University**: Arizona State University
- **Class**: RAS598 Experimentation and Deployment of Robotics
- **Advising Faculty**: Dr. Daniel Aukes

## **1. Introduction**

### **Project Scope**

Our team's project revolves around the integration of two distinct robotic systems, the UR5 robotic arm and the TurtleBot, to perform collaborative object manipulation and retrieval tasks. The essence of the project lies in the ability of the UR5 to precisely place an object at a random, but predetermined location and then communicate the object’s coordinates to the TurtleBot. Utilizing its onboard camera and LiDAR systems, the TurtleBot then autonomously navigates to the location to retrieve or interact with the object. This setup exemplifies an advanced use of robotic collaboration, highlighting the potential for autonomous systems to work together to achieve complex tasks.

The core research question of this project is: "How can two autonomous robotic systems be effectively integrated to collaborate on complex physical tasks involving object manipulation and retrieval in dynamic environments?" This study explores the potential for combining stationary and mobile robots to enhance task execution that single robots cannot perform alone. It focuses on the integration of robotic arms, mobile platforms, precise localization, and autonomous navigation within the Robot Operating System (ROS) framework. Additionally, the research assesses the system's adaptability to new and changing environments, essential for real-world applications.

### **Project Architecture Diagram**
![Project Architecture](images/Project_archtecture.png)


## **2. Hardware and Software Requirements**


### Hardware Components

**UR5 Robotic Arm**
  The UR5 robotic arm is used for object manipulation and requires an external camera for vision-based object localization. It plays a crucial role in picking and placing objects at designated locations.

**TurtleBot 4**
  The TurtleBot 4 is equipped with LiDAR, IMU, and a camera, enabling efficient navigation and mapping. It operates on a Jetson Nano (or Raspberry Pi) and runs ROS2, which facilitates autonomous movement and interaction with other robotic systems.

**External Camera**
  Used for detecting and localizing the object after UR5 placement.

**Compute System**
  A laptop/workstation with Ubuntu and ROS2 installed for running the overall system.

**Wireless Communication**
  TCP/IP or ROS2 communication between UR5 and TurtleBot.

### **Interface Mockup**

The GUI below illustrates the interface designed to monitor camera input, visualize IMU readings, and control TurtleBot functions. This prototype demonstrates interaction points such as camera control buttons, real-time visualization, and plot display for sensor feedback.

![GUI Mockup](images/mockup.png)


### **Software Components**

**Operating System:** Ubuntu 22.04 (for ROS2 support)
**Robotics Framework:** ROS2 (Humble)
**Simulation & Visualization:** RViz, Gazebo, or Webots
**Computer Vision:** OpenCV for object detection using the external camera
**SLAM & Navigation:**
  - SLAM Toolbox for mapping the environment
  - Nav2 Stack for path planning

### Custom ROS2 Nodes:
- Node for UR5 Object Placement & Detection
- Node for TurtleBot Navigation to Target
- Communication node using ROS2 topics/services

## **3. Sensor Integration and Interaction**

| Sensor | Function in Project |
| ------ | ------------------- |
| Camera (External) | Detects the object and computes its position in the UR5 frame |
| Camera (TurtleBot 4) | Detects obstacles, supports localization |
| LiDAR (TurtleBot 4) | Helps in SLAM and obstacle detection |
| IMU (TurtleBot 4) | Provides orientation data for navigation |
| Encoders (UR5 & TurtleBot) | Measures movement and ensures accurate positioning |

### **Interaction**

**UR5 Object Detection & Placement**
  The external camera is used to detect the object and determine the placement location. The UR5 then moves to place the object at a random position. Once placed, the object's final position is recorded using vision-based estimation.

**UR5 to TurtleBot Communication**
  ROS2 services or messages are used to send the object’s coordinates from the UR5 to the TurtleBot, ensuring effective communication for the next phase of the task.

**TurtleBot Navigation to Object**
  The TurtleBot localizes itself in the environment using LiDAR and its onboard camera. It utilizes the SLAM Toolbox to map the surroundings and implements the Nav2 stack for efficient path planning. As it approaches the object, camera-based object detection is used for precise final positioning.

**Final Object Verification**
  Upon reaching the designated coordinates, the TurtleBot’s camera verifies the presence of the object, ensuring successful completion of the task.


## **3. Control and Autonomy**

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

## **5. Preparation Needs**

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

## **6. Final Demonstration Plan**

The final demonstration will consist of:

### Initial Setup:
The TurtleBot and UR5 are positioned in an indoor environment with obstacles, and a human places the object within the UR5's workspace.

### Execution:
The UR5 picks up the object and places it at a random location. The external camera detects the object's final position and sends the coordinates to the TurtleBot. Using LiDAR and the Nav2 stack, the TurtleBot navigates to the object's location and verifies its presence using its onboard camera.

### Performance Metrics:
The system's performance is evaluated based on the success rate of object retrieval, the accuracy of position estimation, and navigation efficiency, including time taken and obstacle avoidance.

## **7. Required Resources**

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

## **8. Testing & Evaluation Plan**

### **1. Unit Testing**
Test UR5 pick-and-place, TurtleBot navigation, and sensor accuracy.

### **2. Integration Testing**
Validate system performance in simulation and real-world scenarios.

### **3. Performance Metrics**
Measure task completion time, object detection accuracy, and navigation success.

### **4. Error Handling**
Implement recovery strategies for object detection failures and navigation errors.

### **5. Final Demonstration**
Showcase end-to-end system functionality, obstacle handling, and adaptability.

## **9. Project Impact**

This project advances multi-robot collaboration by integrating robotic arms with mobile robots, showcasing applications in warehousing and logistics for autonomous material handling. It contributes to research and development by exploring real-time sensor fusion, enhancing object localization, and improving robot perception strategies. Additionally, it serves as an educational tool for learning ROS2, SLAM, and perception systems.

## **10. Advising & Resource Needs**

**Advisor:** Prof. Daniel M Aukes

### Advisor's Role and Contribution
Prof. Daniel M Aukes, an esteemed faculty member with deep expertise in SLAM (Simultaneous Localization and Mapping) and multi-robot coordination, will serve as the primary advisor for this project. Prof. Aukes brings a wealth of experience in robotic systems design and implementation, which will be invaluable for guiding the technical development and integration aspects of our project. His prior work, focusing on the application of innovative robotics solutions in real-world scenarios, will provide critical insights into our design and execution phases. 

Prof. Aukes will also assist in troubleshooting complex system integration issues, ensuring that both the UR5 robotic arm and the TurtleBot function harmoniously. Additionally, he will provide access to laboratory resources and equipment, facilitate networking with other robotics experts, and help in acquiring necessary project funding. His mentorship will be crucial in steering the project towards successful completion and ensuring that the team remains aligned with the latest advancements in robotics technology.


