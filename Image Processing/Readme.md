# 🏀 Image Processing

This project enables the basketball robot to see the hoop and ball, understand their positions on the field, and make decisions to score effectively during the ABU Robocon 2025 competition.  

---
## 🔍 What the Project Does

**Detects Hoop and Basketball:**  
The robot uses a camera to identify the position of the basketball and the hoop in real time.  

**Converts Image to Real-World Positions:**  
The robot calculates the actual positions of the hoop and ball on the 15m × 8m field from the camera images.  

**Decides the Best Action:**  
Based on the ball’s distance from the hoop, the robot chooses whether to:  
- Attempt a dunk (close range)  
- Take a 2-point shot (medium range)  
- Take a 3-point shot (long range)  

**Calculates Shooting Details:**  
The robot uses these positions to calculate the exact angle and speed needed to make the shot.  

**Monitors Performance in Real Time:**  
The system keeps track of distance, speed, and shooting angle to improve accuracy and strategy during the competition.  

---
## ▶️ How to Run

To see the results of the image processing and decision-making:  

1. Open **`image_processing_decision.m`** in MATLAB.  
2. Run the file.  
3. The script will produce all the necessary graphs showing hoop and ball positions, shooting zones, and decision outputs based on the vision data.  
4. Required Toolboxes of Matlab (only add if not installed previously)
    - Computer Vision Toolbox
    - Image Acquisition Toolbox
    - Image Processing Toolbox

---
This README summarizes the **image processing pipeline, decision logic, and real-time performance monitoring** of the basketball robot for ABU Robocon 2025.
