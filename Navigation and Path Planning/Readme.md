# 🤖 Navigation & Path Planning

This project implements the navigation and path planning system for the basketball robot in ABU Robocon 2025. The system enables the robot to move across the court efficiently, avoid obstacles, and reach optimal shooting positions for scoring.

---

## 🔍 What the Project Does

**Plans Robot Paths:**  
The robot uses a camera and sensors to detect opponents and obstacles, then calculates the best route to move across the court using the RRT* path planning algorithm.

**Adapts to Dynamic Obstacles:**  
The positions of opponent robots are tracked in real time. The robot updates its path every second to avoid collisions while maintaining a safe distance.

**Follows Game Phases:**  
The navigation system is integrated with a Stateflow state machine to handle different phases of the game:

- **IDLE:** Start state  
- **TO_HALF_COURT:** Move to the half-court position  
- **NAVIGATE_TO_ARC:** Move toward the 3-point arc entry  
- **AROUND_ARC:** Follow an arc trajectory for optimal shooting  
- **SHOOT:** Execute the 3-point shot  
- **GAME_COMPLETE:** End state  

**Integrates with Shooting Mechanism:**  
Navigation data is combined with the robot’s shooting control to ensure it reaches the right position before taking a shot, following the 20-second shot clock.

**Real-Time Visualization and Debugging:**  
The system includes a 3D court visualization showing the robot’s path, opponent positions, and live state monitoring to verify and debug navigation.

---

## ▶️ How to Run

To simulate navigation and view the planned paths:

1. Open **`runRoboconWithSimulink.m`** in MATLAB.  
2. Run the script to generate the real-time 3D court visualization, showing robot trajectories, opponents, and state transitions.
3. Required Toolboxes of Matlab (only add if not installed previously)
    - Mapping Toolbox
    - Navigation Toolbox
    - ROS Toolbox
    - Simulink
    - Stateflow