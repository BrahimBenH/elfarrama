# ElFarrama - Autonomous Robot

An autonomous robot system with computer vision capabilities for digit detection and precise navigation control.

## Features

- **Navigation**: Differential drive with PID control and odometry
- **Computer Vision**: Digit detection (3, 5, 6, 9) using OpenCV and ML models
- **Hardware**: Arduino-based control with encoder feedback

## Project Structure

- `arduino code/` - Main robot control code
- `arduino test codes/` - Testing and development code
- `computer vision/` - Python-based vision processing
- `resources arduino/` - Reference implementations

## Hardware

- Arduino-compatible microcontroller
- 2x DC motors with quadrature encoders
- Motor driver (L298N)
- USB camera for vision processing

## Quick Start

1. Upload Arduino code to your microcontroller
2. Install Python dependencies: `pip install opencv-python numpy torch`
3. Run vision detection scripts from `computer vision/digit detection/`

## Robot Specifications

- Wheel radius: 39.55mm
- Wheelbase: 305mm
- Encoder resolution: 800 ticks/revolution