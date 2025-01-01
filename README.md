# "SPOK" Project

## Overview

This project is inspired by Boston Dynamics' Spot Mini robot and aims to develop a quadrupedal robot capable of navigating inaccessible zones to assist in firefighting operations. Built on ROS2, this robot integrates advanced locomotion and perception to function in hazardous environments.

## Features

- **Quadrupedal Locomotion**: Robust and stable walking mechanisms for rough terrain.
- **Real-Time Video Display**: A camera system to stream real-time video to remote operators.
- **Autonomous and Teleoperated Modes**: Switch between autonomous operation and manual control via a remote interface.

## Objectives

1. Develop a quadrupedal robot platform using 3D-printed parts and accessible components.
2. Leverage ROS2 for modular and efficient robotics software development.
3. Implement a robust locomotion algorithm for navigation.
4. Create a user-friendly interface for teleoperation and data monitoring.

## Hardware Requirements

- **Processor**: Raspberry Pi 4 or NVIDIA Jetson Nano for onboard processing.
- **Motors**: High-torque servo motors for leg movement.
- **Sensors**:
  - Thermal camera for heat detection.
  - Lidar for environmental mapping.
  - Gas sensor for hazardous gas detection.
  - IMU for stability and orientation.
- **Camera**: For real-time video streaming to remote operators.
- **Power Supply**: Lithium polymer (LiPo) battery pack.
- **Body**: 3D-printed frame using heat-resistant materials.

## Software Requirements

- **ROS2 (Robot Operating System 2)**
- **Python** for custom control algorithms.
- **Gazebo** for simulation.
- **Rviz** for visualization.
- **OpenCV** for computer vision tasks.
- **SLAM Toolbox** for mapping and navigation.

## Installation

...

## Usage

- **Simulation**:

- **Teleoperation**:

- **Real-Time Video Stream**:
  Access the live video feed from the robot’s camera through the provided ROS2 node or remote interface.

## Future Enhancements

...
