# Embedded Autonomous Car

## Project Overview
This project develops a integrated autonomous car system designed to bridge low-level control with high-level perception and navigation.  
It integrates real-time embedded control, visual SLAM, and wireless communication in a unified ROS 2 framework.
- Implement **low-level real-time control** on an **ESP32** using **FreeRTOS**  
- Design a **PID controller** for precise **speed** and **position** regulation  
- Deploy **ORB-SLAM3** on a **laptop** using **monocular camera** map building and localization  
- Allow **Xbox Controller** to manually control the car via **Bluetooth**  
- Collect and process **IMU**, **voltage/current sensor**, and **wheel encoder** feedback on ESP32 and display through UI  

## System Architecture

### 1. ESP32 (Microcontroller)

**Runs:** FreeRTOS  
**Responsibilities:**
- PWM motor control  
- PID-based speed and position control using motor encoder feedback  
- IMU and power monitor sensor data acquisition  
- Command reception via UART (from Jetson Nano) or Bluetooth (from Xbox controller)  
- IMU, power, xbox, encoder data transmission to Jetson Nano  

**Key Features:**
- Multi-tasking using FreeRTOS (PID, BLE, Sensor, UART tasks)  
- Thread-safe communication using mutexes and queues  
- Real-time logging for debugging and tuning

### 2. Jetson Nano (Edge Device)

**Runs:** Linux 
**Responsibilities:**
- Capture camera data and stream wirelessly to laptop
- Receive and display sensor information from ESP32

### 3. Laptop

**Runs:** Linux + ROS 2
**Responsibilities:**
- Run ORB-SLAM3 for monocular mapping and localization   
- Visualize trajectory, map, and system status in **RViz**

## Power and Communication Protocol
The power for the system comes from power bank and LiPo battery. The 7.4V LiPo battery is used to power motor driver and motors, with estimated 6V and 1A power consumption for each motors.
The power bank supply 5V to ESP32 and Jetson Nano. ESP32 consumes ~0.75W and Jetson Nano consumes ~10W, which is less than the maximum power output from power bank (15W).

IMU and power monitor (INA219) uses I2C to communicate with microcontroller. Jetson use UART and Xbox controller use BLE.

## Future Work
- Profiling for RTOS
- Monocular 3D map is too noisy for path planning. Consider building usable map with Semantic Segmentation-Based Occupancy Grid Map
- Combine map with visual odometry for path planning
- Obstacle avoidance via on board camera object detection

### System Overview

<img width="1903" height="924" alt="system" src="https://github.com/user-attachments/assets/2fb3ffa1-1204-41dd-ad6d-9c4444813b89" />

### Power diagram

<img width="879" height="930" alt="power" src="https://github.com/user-attachments/assets/a0e5cdd5-b235-4eb0-a7bb-0fba8083576b" />

### RTOS diagram

<img width="1635" height="931" alt="rtos" src="https://github.com/user-attachments/assets/6f0524cd-b60b-4c94-9e38-2878134377e1" />

### Final product

<img width="1635" height="931" alt="product1" src="https://github.com/user-attachments/assets/42ba84d0-ebc5-4eb4-916f-618c0549ce71" />
<img width="1635" height="931" alt="product2" src="https://github.com/user-attachments/assets/aa3f794a-09ac-4cb1-99ad-13a48148fb29" />

### Demo

https://github.com/user-attachments/assets/c597c4e1-2a24-4021-994b-320b01e9cd99

https://github.com/user-attachments/assets/7df206d4-695d-4590-8f20-995dfc60ed55


