---
title: Charts
---

```mermaid
graph TD;
    A[🔄 Start System Initialization] -->|Power Up & Self Check| B
    B[🤖 TurtleBot4: Activate Sensor Suite<br/><i>IMU, LiDAR, Camera, Odometry</i>] --> C
    C[🗺️ Perform SLAM & Build Map<br/><i>LiDAR-based Environment Mapping</i>] --> D
    D[📡 ESP32: Transmit Target Coordinates<br/><i>Over Wireless Interface</i>] --> E
    E[🧭 ROS2: Convert Coordinates to Goal Pose<br/><i>Apply TF Transformations</i>] --> F
    F[🛣️ Nav2 Stack: Perform Global & Local Planning<br/><i>Pathfinding with Obstacle Avoidance</i>] --> G
    G[🤖 TurtleBot4: Execute Motion Plan<br/><i>Navigate Autonomously to Target</i>] --> H
    H[📊 Collect Real-time Sensor Data<br/><i>IMU for Stability, LiDAR for Collision Avoidance</i>] --> I
    I[📍 Check Goal Reachability<br/><i>Feedback Loop from Navigation Result</i>] --> J

    J{✅ Has TurtleBot4 Reached Target?}
    J -->|Yes| K
    J -->|No: Replan & Adjust Path| F

    K[📈 Log Metrics & Final Position<br/><i>Store for Analysis</i>] --> L
    L[🧠 Optimize Parameters<br/><i>Update SLAM, TF, Nav Configs</i>] --> A

    %% Styling
    classDef startStyle fill:#CAAEFF,stroke:#333,stroke-width:2px;
    classDef processStyle fill:#85D3FF,stroke:#333,stroke-width:1px;
    classDef decisionStyle fill:#ff6666,stroke:#333,stroke-width:1px,font-weight:bold;

    class A startStyle;
    class B,C,D,E,F,G,H,I,K,L processStyle;
    class J decisionStyle;
