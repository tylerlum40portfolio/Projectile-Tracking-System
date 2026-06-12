# Pan-Tilt Laser Tracker

## Overview

The Pan-Tilt Laser Tracker is an STM32F411-based embedded mechatronic system designed to automatically detect, track, and indicate the location of a target in real time. The system combines computer vision, distance sensing, embedded control, and custom electronics into a single integrated platform.

A Pixy2 vision camera is used to detect a colored target and determine its position within the camera image. A TF-Luna LiDAR sensor provides distance measurements to validate target range. A custom controller processes this information and commands pan and tilt motors to continuously align the system with the target. A laser module is used to indicate the current tracked location.

The project was developed as part of ME 405 Mechatronics at California Polytechnic State University, San Luis Obispo and demonstrates many of the embedded systems, control, sensing, and software engineering concepts covered throughout the course.

---

## Team Members

- Tyler Lum
- Colby Cordoba
- Sam Nunes

Mechanical Engineering – Mechatronics

California Polytechnic State University, San Luis Obispo

---

## Project Features

### Sensing

- Real-time target detection using a Pixy2 vision sensor
- Distance measurement using TF-Luna LiDAR
- Target validation based on camera and distance data
- Sensor data filtering and validation

### Actuation

- Pan-axis motor control
- Tilt-axis motor control
- Laser target indication
- Closed-loop target tracking

### Embedded Systems

- STM32F411 microcontroller
- SPI communication with Pixy2 camera
- UART communication with TF-Luna LiDAR
- PWM motor control using STM32 timers
- GPIO control for laser operation
- Modular embedded C firmware architecture

---

## Major Hardware

### Electrical Components

- STM32F411 Microcontroller
- Pixy2 Camera
- TF-Luna LiDAR Sensor
- Custom PCB
- Laser Module
- Voltage Regulation Circuitry
- SWD Programming Interface

### Mechanical Components

- Pan/Tilt Tracking Assembly
- Camera Mount
- LiDAR Mount
- Laser Mount
- PCB Mounting Bracket
- Structural Support Components

---

## Software Architecture

The firmware is organized into modular drivers and application layers.

### Main Application

The main application initializes all peripherals, configures the hardware interfaces, and executes the primary tracking loop. It coordinates communication between the sensors, controller, and actuators.

### Controller Module

The controller module serves as the intelligence layer of the system. It processes Pixy2 target information and LiDAR distance measurements, computes tracking error, and generates motor commands to maintain target alignment.

### Motor Driver

The motor driver provides an abstraction layer for pan and tilt actuation. The driver handles PWM generation, motor direction control, speed limiting, and safe motor shutdown.

### LiDAR Driver

The LiDAR driver manages UART communication with the TF-Luna sensor. The driver validates measurement frames, checks data integrity, and provides distance measurements to the controller.

### Pixy2 Driver

The Pixy2 driver handles SPI communication with the camera and processes detected object information. The driver extracts target location information used by the tracking controller.

### Laser Driver

The laser driver manages laser activation and indication logic based on controller state and target lock conditions.

---

## Tracking Algorithm

The tracking system operates using the following sequence:

1. Acquire target location from the Pixy2 camera.
2. Acquire distance measurement from the TF-Luna LiDAR.
3. Validate target detection and range.
4. Compute horizontal and vertical tracking error.
5. Generate pan and tilt motor commands.
6. Update motor outputs.
7. Activate laser when tracking criteria are satisfied.
8. Repeat continuously during operation.

---

## Mathematical Modeling and Sensor Processing

Several calculations are performed within the firmware to convert sensor measurements into motor commands.

### Pixel Error Calculation

The controller calculates the difference between the target position and the image center.

```text
Error = Target Position − Image Center
```

The resulting horizontal and vertical error values are used to determine pan and tilt corrections.

### Motor Command Generation

Tracking error is converted into motor commands using proportional control and output limiting to ensure stable operation.

### Distance Validation

LiDAR measurements are checked against valid operating ranges before tracking commands are issued. This prevents false target responses caused by invalid sensor readings.

---

## Repository Structure

```text
Projectile-Tracking-System/
│
├── Core/
│   ├── Inc/
│   └── Src/
│
├── Drivers/
│
├── docs/
│   └── Doxygen Documentation
│
├── images/
│   └── CAD Models, PCB Images, and Project Photos
│
├── index.html
├── README.md
├── Doxyfile
├── .gitignore
```

---

## Documentation

### Project Portfolio

https://tylerlum40portfolio.github.io/Projectile-Tracking-System/

### Doxygen Documentation

https://tylerlum40portfolio.github.io/Projectile-Tracking-System/docs/index.html

The Doxygen documentation contains:

- File documentation
- Function documentation
- Driver descriptions
- Data structures
- Source code references
- Module relationships

---

## Repository Requirements

This repository contains:

- Complete STM32 source code
- Doxygen-generated documentation
- Doxyfile configuration
- Project portfolio website
- CAD renders
- PCB images
- Wiring diagrams
- Mechanical design documentation
- Software architecture documentation
- Project media and demonstration materials

---

## Future Improvements

Potential future improvements include:

- Position feedback using encoders
- Improved target prediction algorithms
- Additional sensor fusion techniques
- Enhanced filtering for sensor measurements
- Improved tracking accuracy and stability

---

