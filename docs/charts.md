---
title: Project Implementation Process
---

## **Flowchart: Project Workflow**
```mermaid
graph TD
  A[Start] --> B{UR5 Object Placement};
  B -->|Object Placed| C[Detect Position with Camera];
  C -->|Extract Coordinates| D[Send Data to TurtleBot];
  D -->|Coordinates Received| E[SLAM Navigation Initiated];
  E -->|Path Planning| F[TurtleBot Navigates to Object];
  F -->|Verifies Object Location| G[End];
```

## **Sequence Diagram: Communication Between UR5 and TurtleBot**
```mermaid
sequenceDiagram
autonumber
participant UR5
participant Camera
participant TurtleBot

UR5->>Camera: Capture Object Position
Camera-->>UR5: Return Object Coordinates
UR5->>TurtleBot: Send Object Coordinates
TurtleBot->>SLAM: Update Map and Plan Path
TurtleBot->>Navigation: Move Towards Object Location
Navigation-->>TurtleBot: Arrived at Target
TurtleBot->>UR5: Object Located Successfully
```

## **State Diagram: System States**
```mermaid
stateDiagram-v2
  [*] --> Idle
  Idle --> Object_Pick[UR5 Picks Object]
  Object_Pick --> Object_Placed[UR5 Places Object Randomly]
  Object_Placed --> Capture_Position[Camera Captures Object Position]
  Capture_Position --> Send_Data[Send Position to TurtleBot]
  Send_Data --> Navigation_Active[TurtleBot Navigates to Object]
  Navigation_Active --> Object_Verification[TurtleBot Confirms Object Location]
  Object_Verification --> [*]
```