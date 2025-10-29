## 📈 System Overview

The Shooter Bot is designed to autonomously compute the required projectile velocity and launch angle to make consistent 3-pointer shots from varying distances.
It integrates sensor feedback, motor control, and trajectory prediction into a modular control system capable of fast real-time execution.

### Core Process

1. Distance Measurement –
    - The DT35 distance sensor continuously measures the robot-to-hoop distance.

2. Trajectory Calculation –
    - The control system computes the required velocity and angle using projectile motion equations.

3. Constraint Handling –
    - All computed values are bounded by the physical limits of the mechanism and actuators.

4. Control Execution –
    - PID controllers drive the shooting motor (RPM) and servo (angle) to reach the computed parameters.

5. Feedback Loop –
    - Real-time RPM and distance feedback ensure trajectory correction within each shot cycle.

## 🔧 Operational Workflow
The shooter bot uses a distance sensor to measure the distance from the hoop. This input is fed into the Simulink model, which calculates the optimal trajectory and outputs the required RPM and angle for a successful 3-pointer shot.

### Running the Simulation

1. Execute the file `SimulateandPlot.m`.  
   - This will automatically generate the Simulink model.  
   - The required graphs based on trajectory calculations will also be produced.

2. To manually adjust the shooting distance, modify the Distance Constant block in the Simulink model.  
   - This allows you to observe and analyze the various trajectories produced for different distances.

3. Required Toolboxes of Matlab (only add if not installed previously)
    - Simulink
    - Simulink Control Design
    - Control System Toolbox

## 📊 Simulation Results
| Distance | Predicted Trajectory     | Actual Match     | Execution Delay |
| -------- | ------------------------ | ---------------- | --------------- |
| 4 m      | Successful arc into hoop | 93 % correlation | <100 ms         |
| 5 m      | Successful arc into hoop | 91 % correlation | <100 ms         |
