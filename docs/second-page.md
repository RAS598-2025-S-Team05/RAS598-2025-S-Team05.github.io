# 📡 Live Sensor Dashboard

This page demonstrates our real-time sensor feeds and visualizations from the TurtleBot platform, including camera, IMU, and Lidar.

---

## 🎥 Live Camera Feed

<iframe width="560" height="315" src="https://www.youtube.com/embed/YOUR_VIDEO_ID" title="Camera Feed" frameborder="0" allowfullscreen></iframe>

> Live video from topic `/rpi_07/oakd/rgb/preview/image_raw`, captured via OpenCV and published using a custom ROS2 GUI.

---

## 🌀 IMU Sensor (Acceleration & Gyroscope)

![IMU Plot](../images/imu_plot.png)

- Topic: `/rpi_07/imu`
- Filtered using moving average
- Plotted using Matplotlib in real-time

---

## 🌐 Lidar Scan

![Lidar](../images/lidar_scan.png)

- Topic: `/rpi_07/scan`
- Visualized using polar plot
- Highlights obstacles in red

---

## 🧠 GUI Overview

![GUI Overview](../images/gui_live.png)

> Our PyQt5-based GUI integrates the camera, IMU, and Lidar feeds in a single window with real-time updates.
