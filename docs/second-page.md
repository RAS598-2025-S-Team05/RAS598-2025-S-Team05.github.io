<style>
.dashboard {
  display: grid;
  grid-template-columns: repeat(2, 1fr); /* 2 columns */
  grid-template-rows: repeat(2, 1fr);    /* 2 rows */
  gap: 20px;
  padding: 20px;
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
  height: 100%;
  min-height: 300px;
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

<div class="dashboard">
  <!-- Row 1, Column 1 -->
  <div class="box">
    <h2>📷 Camera Preview</h2>
    <img id="camera-feed" src="http://<ROBOT_IP>:8080/stream?topic=/camera/image_raw" alt="Camera Feed" />
  </div>

  <!-- Row 1, Column 2 -->
  <div class="box">
    <h2>📡 LIDAR Visualization</h2>
    <canvas id="lidar_canvas" width="500" height="400">LIDAR Display</canvas>
  </div>

  <!-- Row 2, Column 1 -->
  <div class="box">
    <h2>🗺️ SLAM Mapping</h2>
    <canvas id="slam_canvas" width="500" height="400">SLAM Map</canvas>
  </div>

  <!-- Row 2, Column 2 -->
  <div class="box">
    <h2>📈 IMU Angular Velocity</h2>
    <div id="imu_plot" style="height:300px;"></div>
  </div>
</div>
