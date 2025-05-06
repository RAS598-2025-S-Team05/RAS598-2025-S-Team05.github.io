# 🧭 Goal-Oriented Autonomy – Team 05 (RAS598 Spring 2025)

This project enables a TurtleBot4 robot to autonomously navigate to a goal position received from an external ESP32 IMU module over ROS 2. The system supports 2D SLAM-based mapping, localization, real-time obstacle avoidance, and navigation via the Nav2 stack.

---

## 🧩 Project Overview

### Components Involved:
- **ESP32 IMU Module** connected to **Laptop A**
- **TurtleBot4** running ROS 2 Humble
- **Nav2 Stack** for autonomous navigation
- **SLAM Toolbox** for 2D mapping
- **Custom ROS 2 Python nodes** for goal publishing, obstacle avoidance, and map saving

---

## 📁 Folder Structure

```bash
RAS598-2025-S-Team05.github.io/
├── code/                      # Individual Python files and launch files
├── src.zip                   # Full ROS 2 workspace src folder (download and extract)
├── README.md
├── index.md
├── mkdocs.yml
└── ...
```

---

## 🚧 Getting Started (Quick Method via ZIP)

> ✅ To get started quickly, download the full ROS 2 package:

### 📦 Download and Extract:
1. [Click to Download `src.zip`](./src.zip)
2. Extract it into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws
   unzip /path/to/src.zip -d src
   ```

---

## 🧱 Build and Source

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

To source automatically:
```bash
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```

---

## 🚀 Running the System

### 🗺️ 1. Start Mapping

```bash
ros2 launch my_project auto_map.launch.py
```

### 🧭 2. Start Navigation

```bash
ros2 launch my_project navigation_with_local.launch.py
```

### 🛰️ 3. Run ESP32 Publisher from Laptop A

```bash
ros2 run my_project esp32_07_imu
```

---


---

## 🔌 ESP32 IMU Setup on Laptop A

This step allows you to stream IMU data from an ESP32 to the TurtleBot4 by publishing positional goals over serial via USB.

### 📋 Requirements

- An ESP32 microcontroller flashed with firmware that prints output in the format:
  ```
  X: 3.42  Y: -1.85  Z: 2.56
  ```
- A USB cable connecting ESP32 to Laptop A
- `pyserial` Python package installed:
  ```bash
  pip install pyserial
  ```

---

### ⚙️ Run the Goal Publisher on Laptop A

Make sure the ESP32 is connected to the correct port (e.g., `/dev/ttyUSB0` on Ubuntu or `COM3` on Windows). Update the port in the script if needed.

Then run:

```bash
ros2 run my_project esp32_07_imu
```

This script:
- Connects to ESP32 over serial
- Parses the IMU data
- Publishes the position as `geometry_msgs/Point` to the topic:
  ```
  /esp32_07/goal
  ```

> 🗂️ **You can find this script in the `code/` folder as `esp32_07_imu.py`.**

---

### 🧠 Output Example

When working correctly, the node will print:
```
✅ Published goal: x=3.42, y=-1.85
```

This data is received by the `esp32_goal.py` node running on TurtleBot4, which converts it into a navigation goal and triggers motion.


## 📹 Video Demonstrations

Visit our [Project Website](https://your-username.github.io/RAS598-2025-S-Team05.github.io/) to view:
- Mapping demonstration
- Goal publishing from ESP32
- Navigation visualization in RViz

---

## 📦 Dependencies

- ROS 2 Humble
- SLAM Toolbox
- Navigation2 (Nav2)
- Python 3.8+
- rclpy, std_msgs, geometry_msgs, nav2_msgs, tf2_ros

---

## 📌 License

MIT License
