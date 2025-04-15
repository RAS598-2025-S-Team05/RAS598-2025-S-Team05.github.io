---
layout: default
title: Sensor Dashboard
---

<style>
.dashboard {
  display: grid;
  grid-template-columns: 1fr 1fr;
  grid-template-rows: 1fr 1fr;
  grid-gap: 20px;
  padding: 20px;
}
.box {
  border: 2px solid #ccc;
  border-radius: 10px;
  padding: 15px;
  background-color: #fdfdfd;
  box-shadow: 2px 2px 10px rgba(0,0,0,0.05);
  min-height: 300px;
  display: flex;
  flex-direction: column;
  justify-content: flex-start;
}
h2 {
  text-align: center;
}
canvas, img, #imu_plot, #slam_canvas {
  width: 100%;
  height: 250px;
  object-fit: contain;
}
</style>

# 🧠 Live Sensor Data Viewer

<div class="dashboard">
  <!-- Left Column -->
  <div class="box">
    <h2>📷 Camera Preview</h2>
    <img id="camera-feed" src="http://<ROBOT_IP>:8080/stream?topic=/camera/image_raw" alt="Camera Feed" />
  </div>

  <div class="box">
    <h2>📡 LIDAR Visualization</h2>
    <canvas id="lidar_canvas" width="500" height="400">LIDAR Display</canvas>
  </div>

  <div class="box">
    <h2>🗺️ SLAM Mapping</h2>
    <canvas id="slam_canvas" width="500" height="400">SLAM Map</canvas>
  </div>

  <div class="box">
    <h2>📈 IMU Angular Velocity</h2>
    <div id="imu_plot" style="height:300px;"></div>
  </div>
</div>

<!-- Libraries -->
<script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
<script src="https://static.robotwebtools.org/roslibjs/current/roslib.min.js"></script>

<!-- Live ROS Dashboard Script -->
<script src="/docs/javascripts/live_dashboard.js"></script>
