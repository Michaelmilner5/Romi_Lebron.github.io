# Romi Autonomous Robot Project

## Overview
This project documents the design and implementation of an autonomous Romi robot developed to complete a multi-stage obstacle course. The robot integrates multiple sensors and control strategies to navigate, detect obstacles, and execute sequential behaviors without human intervention.

The system combines embedded programming, sensor integration, and control theory to create a reliable and repeatable autonomous platform.

---

## Key Features
- Line following using a reflectance sensor array  
- Closed-loop motor control with encoder feedback  
- Obstacle detection using bump sensors  
- Heading and motion tracking with an IMU (BNO055)  
- Discrete-time state observer for improved estimation  
- Modular task-based software architecture  
- Real-time user interface for tuning and debugging  

---

## System Architecture
The robot software is structured using a cooperative multitasking scheduler. Each major function is implemented as its own task, allowing for modular design and easier debugging.

Core subsystems include:
- Motor control (PI velocity loops)
- Line following (centroid-based control)
- IMU processing
- State observer
- Bump detection and safety response
- User interface and command handling

---

## Repository Structure

### Main & Task Files
- `main.py` – Initializes hardware, shared variables, and scheduler  
- `task_course.py` – High-level state machine for obstacle course execution  
- `task_motor.py` – Motor control with encoder feedback  
- `task_line.py` – Line sensing and steering logic  
- `task_imu.py` – IMU data acquisition and filtering  
- `task_observer.py` – State estimation using discrete-time observer  
- `task_bump.py` – Collision detection and response  
- `task_user.py` – Serial interface for runtime interaction  
- `task_log_est.py` – Logs estimated states for analysis  
- `task_garbage.py` – Memory management task  

### Drivers & Utilities
- `motor_driver.py` – PWM motor driver interface  
- `encoder.py` – Encoder reading and velocity estimation  
- `line_sensor.py` – Reflectance sensor array interface  
- `driver.py` – BNO055 IMU communication driver  
- `task_share.py` – Inter-task communication structures  
- `cotask.py` – Cooperative task scheduler  
- `observer_matrices.py` – Observer model matrices  

### Other Files
- `boot.py` – MicroPython boot configuration  
- Calibration and helper files as needed  

---

## Hardware Overview
The robot is built on the Pololu Romi platform and includes:

- Differential drive motors with encoders  
- Nucleo L476RG microcontroller  
- Shoe of Brian interface board  
- BNO055 IMU  
- QTRX reflectance sensor array  
- Dual front bump sensors  

---

## Getting Started
1. Flash MicroPython onto the Nucleo board  
2. Upload all repository files to the board  
3. Connect hardware according to wiring diagram  
4. Power the robot and open serial interface  
5. Run `main.py` to start the system  

---

## Project Goals
- Demonstrate closed-loop control and sensor fusion  
- Implement modular embedded system architecture  
- Develop reliable autonomous navigation  
- Create a reproducible and well-documented system  

---

## Project Website
For full documentation, design details, and media:

https://michaelmilner5.github.io/Romi_Lebron.github.io/

---

## Authors
- Tyler Lum  
- Michael Milner  

---

## Notes
This repository focuses on implementation and system structure.  
Detailed explanations, diagrams, and analysis are provided on the project website.
