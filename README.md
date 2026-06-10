# Pan-Tilt Laser Tracker

## Overview

The Pan-Tilt Laser Tracker is an STM32F411-based embedded system that automatically detects and tracks a target using computer vision and distance sensing. The system uses a Pixy2 camera to identify a colored target, a TF-Luna LiDAR sensor to measure distance, and two DC motors to control pan and tilt motion. A laser module is used to indicate the tracked target location.

This project was developed for ME 405 Mechatronics at California Polytechnic State University, San Luis Obispo.

## Features

- Real-time target detection and tracking
- Pan and tilt motor control
- TF-Luna LiDAR distance measurement
- Laser target indication
- SPI communication with Pixy2 camera
- UART communication with LiDAR sensor
- PWM motor control using STM32 timers
- Modular embedded C firmware architecture

## Hardware

- STM32F411 Microcontroller
- Pixy2 Vision Sensor
- TF-Luna LiDAR Sensor
- Dual DC Motors
- Laser Module
- Custom PCB

## Software Architecture

The firmware is organized into several modular drivers:

- **Laser Driver** – Controls laser activation and indication
- **LiDAR Driver** – Reads and validates TF-Luna distance measurements
- **Motor Driver** – Controls pan and tilt motors using PWM
- **Pixy2 Driver** – Processes camera data and identifies tracking targets
- **Main Application** – Coordinates sensor inputs and actuator outputs

## Repository Structure

```text
Core/
├── Inc/
├── Src/

README.md
Doxyfile
.gitignore
```

## Documentation

Full project documentation and Doxygen-generated documentation:

https://tylerlum40portfolio.github.io/Projectile-Tracking-System/#repository

https://tylerlum40portfolio.github.io/Projectile-Tracking-System/docs/index.html

## Author

Colby Cordoba, Tyler Lum, Sam Nuno
Mechanical Engineering – Mechatronics  
California Polytechnic State University, San Luis Obispo
