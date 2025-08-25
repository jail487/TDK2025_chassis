# TDK2025 Chassis

This project is for the TDK2025 chassis, which includes communication, localization, and control for a robotic system. It is developed using STM32CubeIDE and integrates ROS1 for communication.

## Features
- **ROS1 Communication**: Publishes and subscribes to ROS topics for controlling the chassis and receiving mission updates.
- **Localization**: Tracks the chassis position and orientation in real-time.
- **Motor Control**: Implements inverse and forward kinematics for mecanum wheels.

## Folder Structure
- `ROS1/`: Contains ROS communication code.
- `Chassis/`: Handles motor control and localization.
- `location/`: Implements pathfinding and movement logic.
- `Lifter/`: Controls the lifter mechanism.

## How to Use
1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/TDK2025_chassis.git