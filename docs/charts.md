## 🚀 TurtleBot4 Navigation Flow

```mermaid
graph TD;
  A[🔧 Start System Initialization] -->|Power Up & Self Check| B[🤖 TurtleBot4: Activate Sensor Suite]
  B --> C[🧭 IMU, LiDAR, Camera, Odometry: Perform SLAM & Build Map]
  C --> D[📡 ESP32: Transmit Target Coordinates]
  D --> E[🌐 ROS2: Convert Coordinates to Goal Pose]
  E --> F[🗺️ Nav2 Stack: Global & Local Planning]
  F --> G[🤖 Execute Motion Plan]
  G --> H[📊 Collect Real-time Sensor Data]
  H --> I[📍 Check Goal Reachability]
  I --> J{✅ Has TurtleBot4 Reached Target?}
  J -->|Yes| K[📈 Log Metrics & Final Position]
  K --> L[🧠 Optimize Parameters & Update Configs]
  L --> A
  J -->|No| F
