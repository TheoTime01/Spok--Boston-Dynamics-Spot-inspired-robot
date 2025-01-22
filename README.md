# "SPOK" Project

Authors:
- Théotime PERRICHET
- Arnaud SIBENALER

## Overview

This project, inspired by Boston Dynamics' Spot robot, aims to develop a quadruped robot capable of navigating inaccessible zones to assist in firefighting operations. 

![Spok](media/spok2.jpg)

### Hardware

Our robot is made up of 3D printed parts. Most of the 3D models can be found here: [Thingiverse Project](https://www.thingiverse.com/thing:3638679). Some parts have been recreated to fit to our electronics.

Hardware list:
- Raspberry Pi Zero 2W
- Servo Driver HAT (PCA9685)
- x12 Servo motors (MG996R)
- IMU (MPU6050)
- USB to Micro USB adapter
- USB webcam
- Raspberry Pi Pico
- x2 Ultrasonic sensors (HC-SR04)
- x4 Red LEDs with 220Ω resistors
- Jumper wires
- x2 Micro USB cables
- 7.5V mains power supply
- 3D printed parts
- x8 bearings
- screws and nuts

![Spok](media/schema.png)

The total cost of the robot is around 200€.


# List of features:

1. [x] RO
2. [ ] Incomplete features
    1. [ ] Sub-task 1
    2. [x] Sub-task 2
3. [x] ROS Navigation Stack
    1. [x] Sub-task 1
    2. [x] Sub-task 2

# Représentation des noeuds

```mermaid
graph LR
    T1[Odometry source] -- /odom --> Node((local_planner_student))
    T2[Laser source] -- /scan --> Node((local_planner_student))

    S1[ ] -. /move_to/singleGoal .-> Node
    S2[ ] -. /move_to/pathGoal .-> Node

    Node -- /cmd_vel_mux/input/navi -->D[base controller]
```

# Description de l'algo

```mermaid
sequenceDiagram
    participant Alice
    participant Bob
    Alice->John: Hello John, how are you?
    loop Healthcheck
        John->John: Fight against hypochondria
    end
    Note right of John: Rational thoughts <br/>prevail...
    John-->Alice: Great!
    John->Bob: How about you?
    Bob-->John: Jolly good!
```

# Vidéos de présentation

[Lien vers la vidéo pitch youtube](url)

[Lien vers la vidéo tutoriel youtube](url)

# Liste des dépendances et pré-requis

- a
- b

# Procédure de mise en route

- a
- b
- n

























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

![Schema](media/schema.png)

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







