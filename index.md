# Pan-Tilt Laser Tracker Portfolio

Project portfolio by Tyler Lum documenting the design, implementation, and testing of an STM32-based pan-tilt laser tracker.

## Project Overview

The Pan-Tilt Laser Tracker is an embedded mechatronics system that detects and tracks a colored target using a Pixy2 camera, measures distance using a TF-Luna LiDAR sensor, and controls two DC motors to aim a laser module.

## Objectives

- Detect a colored target using computer vision
- Move pan and tilt motors to follow the target
- Measure distance using LiDAR
- Control a laser indicator
- Organize firmware into modular drivers

## Hardware

- STM32F411 microcontroller
- Pixy2 camera
- TF-Luna LiDAR sensor
- Dual DC motors
- Laser module
- Custom PCB

## Software Architecture

The firmware is organized into separate drivers for the camera, LiDAR, motors, and laser. The main loop initializes each subsystem and coordinates sensor readings with actuator outputs.

## Code Architecture

- `main.c` – Main application loop
- `pixy2_camera.c/.h` – Camera communication and target detection
- `LiDAR.c/.h` – Distance measurement
- `motor_driver.c/.h` – Pan and tilt motor control
- `Laser.c/.h` – Laser control

## Results

Include photos, testing notes, and video links here.

## Detailed Documentation

[Doxygen Documentation](./docs/html/index.html)

## Repository

[GitHub Repository](PASTE_REPO_LINK_HERE)
