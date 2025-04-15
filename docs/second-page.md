<style>
.dashboard {
  display: grid;
  grid-template-columns: 1fr 1fr; /* 2 equal columns */
  grid-template-rows: 1fr 1fr;    /* 2 equal rows */
  gap: 20px;
  padding: 20px;
  height: 100vh; /* Adjust as needed */
}
.box {
  border: 2px solid #ccc;
  border-radius: 10px;
  padding: 15px;
  background-color: #fdfdfd;
  box-shadow: 2px 2px 10px rgba(0,0,0,0.05);
  display: flex;
  flex-direction: column;
  justify-content: flex-start;
}
h2 {
  text-align: center;
  margin-top: 0;
}
canvas, img, #imu_plot, #slam_canvas {
  width: 100%;
  height: 100%;
  object-fit: contain;
  flex-grow: 1;
}
</style>

<div class="dashboard">
  <!-- Left Column -->
  <div class="box">
    <h2>📷 Camera Preview</h2>
    <img id="camera-feed" src="http://<ROBOT_IP>:8080/stream?topic=/camera/image_raw" alt="Camera Feed" />
  </div>
  
  <!-- Right Column -->
  <div class="box">
    <h2>📡 LIDAR Visualization</h2>
    <canvas id="lidar_canvas" width="500" height="400">LIDAR Display</canvas>
  </div>

  <!-- Left Column -->
  <div class="box">
    <h2>🗺️ SLAM Mapping</h2>
    <canvas id="slam_canvas" width="500" height="400">SLAM Map</canvas>
  </div>

  <!-- Right Column -->
  <div class="box">
    <h2>📈 IMU Angular Velocity</h2>
    <div id="imu_plot" style="height:100%;"></div>
  </div>
</div>
