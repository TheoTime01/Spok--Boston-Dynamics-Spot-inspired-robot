# SPOK Project

## Authors
- **Théotime Perrichet**
- **Arnaud Sibenaler**

## Overview
SPOK is a quadruped robot inspired by Boston Dynamics' Spot, designed to navigate inaccessible areas to assist in firefighting operations.

This project is made possible thanks to [SpotMicroAI](https://spotmicroai.readthedocs.io/en/latest/).

![Spok](media/spok2.jpg)

## Table of Contents

- [Videos](#videos)
- [Hardware](#hardware)
- [Software](#software)
- [Features](#list-of-features)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Running the Project](#running-the-project)
  - [On the Robot](#running-on-the-robot)
  - [On the Laptop](#running-on-the-laptop)
- [ROS2 Nodes](#ros2-nodes)

---

## Video

[Pitch Video](https://youtu.be/bJ2OreaSHS8)

## Hardware

SPOK consists of 3D-printed parts, electronics, and sensors. Most of the 3D models are available [here](https://www.thingiverse.com/thing:3638679), with some modifications for compatibility with our electronics.

### Hardware List

- **Processing Units:**
  - Raspberry Pi Zero 2W
  - Raspberry Pi Pico
- **Motor Control & Sensors:**
  - Servo Driver HAT (PCA9685)
  - 12x MG996R Servo Motors
  - IMU (MPU6050)
  - 2x Ultrasonic Sensors (HC-SR04)
- **Peripherals:**
  - USB Webcam
  - USB to Micro USB Adapter
  - 2x Micro USB Cables
  - 7.5V Power Supply
  - Jumper Wires, Screws, Nuts, Zip Ties, Glue
- **3D-Printed Parts & Miscellaneous:**
  - PLA-printed structural components
  - Anti-slip material (foam, TPU, etc.)
  - 8x Bearings
  - 4x Red LEDs with 220Ω resistors

### Estimated Cost

Approx. **200€**

![Spok Hardware](media/schema.png)

## Software
The robot's inverse kinematics are based on the [CHAMP repository](https://github.com/chvmp/champ). More details are available [here](software/ik.md).

## List of Features

- **Manual Control**
  - [x] Joystick control
  - [x] Simulation in Gazebo
  - [x] Real robot operation
- **Autonomous Navigation**
  - [x] Simulation in Gazebo
  - [ ] Real robot implementation (in progress)
- **Additional Features**
  - [x] Connection state detection
  - [x] Obstacle detection
  - [x] Video feedback
  - [ ] Person detection (too slow to use)
  - [x] Visual markers (LEDs)


## Dependencies

### On the Robot

- **Operating System:** Ubuntu 22.04
- **Framework:** ROS2 Humble (ROS_DOMAIN_ID=0)
- **Languages & Libraries:**
  - Python 3
  - Python libraries: `smbus`, `smbus2`, `OpenCV`, `SpeechRecognition`
  - ROS packages: `usb_cam`
  - Micro-ROS Agent
  - Docker

### On the Laptop

- **Operating System:** Ubuntu 22.04
- **Framework:** ROS2 Humble (ROS_DOMAIN_ID=0)
- **Languages & Libraries:**
  - Python 3
  - Python libraries: `OpenCV`
  - ROS packages: `Champ`, `CvBridge`, `Nav2`, `Joy`

## Installation

### Install Dependencies

Run the following command to install the required dependencies:
```bash
cd ~/your_workspace
rosdep install --from-paths src --ignore-src -r -y
```

### Build the Workspace

#### On the Laptop:

```bash
colcon build --packages-ignore micro_ros_raspberrypicosdk
```
#### On the Raspberry Pi:

```bash
colcon build --packages-select spok_rob micro_ros_raspberrypicosdk
```

## Running the Project

### On the Robot

#### Launch ROS2 on the robot:

```sh
ros2 launch spok_rob spok.launch.py
```
#### Start Micro-ROS Agent:

```sh
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 115200
```

### On the Laptop

#### Manual Mode:

```sh
ros2 launch quadruped_robot spot_bringup.launch.py
```
#### Autonomous Mode:

```sh
cd src/Spok--Boston-Dynamics-Spot-inspired-robot/software/quadruped_robot/params/ 
ros2 launch quadruped_robot spot_bringup_nav.launch.py headless:=False params_file:="nav2_params_empty.yaml" map:="<absolute_path>/<maps>.yaml"
```

## ROS2 Nodes

## Running on the robot

### joint_servo_controller_node

![Node file](software/spok_rob/spok_rob/joint_servo_controller.py)

This node acts as an interface between the position of every joint and the PWM sent to the servo motors.

```mermaid
graph LR
    T1[Joints positions] -- /joint_group_effort_controller/joint_trajectory --> Node((joint_servo_controller_node))
```

Whenever a _JointTrajectory_ message is received from the _/joint_group_effort_controller/joint_trajectory_ topic, the angles are translated into duty cycles, and sent to the Servo Driver HAT which generates a PWM signal for every motor.
This node connects to the Servo Driver HAT through I2C, with the _smbus_ Python librairy.


### mpu6050_node

![Node file](software/spok_rob/spok_rob/mpu6050_node.py)

This node is used to read acceleration values from the MPU6050 IMU, through the same I2C bus as the servo driver.

```mermaid
graph LR
    T1[IMU data] -. / .-> Node((mpu6050_node))

    Node -- /imu/data -->D[Odometry]
```

The node reads the value of the MPU6050 sensor through the I2C bus, also with the _smbus_ Python librairy. The data is sent every 0.1s in the form of a ROS Imu message, which is the type of message used by the Nav2 package for IMU sensors.


### gyro_node

![Node file](software/spok_rob/spok_rob/gyro_node.py)

This node also reads data from the MPU6050 sensor, but it also converts the data into the pitch and roll angles of the robot using a Kalman filter.
We used this node to balance the robot on an uneven floor, but it is not used for the movement.

```mermaid
graph LR
    T1[IMU data] -. / .-> Node((gyro_node))

    Node -- /robot_orientation -->D[Self balancing]
```

The node publishes the pitch an roll angles every 0.005s in the form of an array of 2 ints.

### connection_node

![Node file](software/spok_rob/spok_rob/connection_node.py)

This node is used to check the state of the connection between the robot and the computer running the video feedback and the navigation packages.
It uses the _subprocess_ Python librairy to ping the computer every second.

```mermaid
graph LR
    T1[Ping IP] -. / .-> Node((connection_node))

    Node -- /connection_state -->D[Self balancing]
    Node -- /pico_subscriber -->D[Micro ROS: LED blinking pattern]
```

The node has 2 publisher. The first one, _/connection_state_ send the state of the connection as a boolean after every ping.
The second one publishes an Int32 message (any number) only when the state of the connection changes. The Micro ROS node subscribes to this topic the change the blinking pattern of the LEDs depending on the connection state.

### face_detection_node

![Node file](software/quadruped_robot/quadruped_robot/face_detection_node.py)

The aim of this node is to add information to the video feedback, such as rectangle around the face of detected people. The node searches for faces using a Haar cascade and OpenCV in every 10 frames of the webcam video. We use a Haar Cascade instead of a detection network because it can run on a Raspberry Pi, even if the result is less precise.

```mermaid
graph LR
    T1[Webcam video] -- /image_raw --> Node((face_detection_node))

    Node -- /output_video -->D[Video feedback]
```

The node subscribes to the _/image_raw_ topic from the _usb_cam_ package, and search for faces with a Haar cascade in every 10 frames.
It creates new frames, overlaying rectangles over detected faces, and send thoese frames through the _/output_video_ topic.
In practice, the low quality of our network makes it difficult to run this node, as it does not receive enough video frames.


### Micro ROS node → pico_node

![Node file](hardware/micro_ros_raspberrypi_pico_sdk/pico_micro_ros_spok.cpp)

We created a Micro ROS node, running in a Raspberry Pi Pico, to interface a microcontroller to the ROS environment.
The microcontroller is conected to 2 HC-SR04 ultrasonic sensors, and 4 LEDs, through its GPIO pins.

```mermaid
graph LR
    T1[Connection state] -- /pico_subscriber --> Node((pico_node))

    Node -- /pico_publisher -->D[Obstacle detection]
    Node -- /pico_publisher_bis -->D[Obstacle detection]
```

The _/pico_subscriber_ subscriber changes the _state_ variable whenever a message is received. This variable changes the blinking pattern of the LEDs.

A timer is used to measure every 0.1s the distance with both ultrasonic sensors, and sends the measured values on the _/pico_publisher_ and _/pico_publisher_bis_ publishers.

To run this node, the c++ file must be compiled with `make -j4`. The microcontroller has to be plugged into a computer in "BOOTSEL" mode (by pressing the button when plugging it). The _.uf2_ compiled file can now be uploaded into the microcontroller.

The Micro ROS Agent is the interface between Micro ROS and ROS, and must be running on the Raspberry Pi to enable serial communication.
```sh
  docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 115200
```


## Running on the laptop

### Manual control

Structure of the ROS nodes in charge of controling the robot manually.
The joystick inputs are converted into velocity , and then joint positions.
We can estimate the position of the robot with the contact position of the foot.

```mermaid
graph LR
    n1[Joy Teleop Node] -- /body_pose --> n2[Quadruped Controller Node]
    n1 -- /cmd_vel --> n2
    n3[Contacts Sensor] -- /foot_contacts --> n4[State Estimation Node]
    n2 -- /joint_group_effort_controller/joint_trajectory --> n5[Joint Group Effort Controller]
    n6[Joy Node] -- /joy --> n1
    n4 -- /odom/raw --> n7[Footprint to Odom EKF]
    n4 -- /base_to_footprint_pose --> n8[Base to Footprint EKF]


    classDef nodeStyle fill:#f9f,stroke:#333,stroke-width:2px;
    class n1,n2,n4,n9,n10,n11,n12,n14,n17,n18 nodeStyle;
```

### Autonomous control

Structure of the node for autonomous movements.
We use the odometry and the measurements from the lidar to estimate the position of the robot.

```mermaid
graph LR
    T1[Odometry source] -- /odom --> Node((Ros2_nav))
    T2[Laser source] -- /scan --> Node((Ros2_nav))

    Node --  /cmd_vel -->D[joint_group_effort_controller]
```

## Possible Improvements

- **Run all processes locally on the Raspberry Pi** to remove the dependency on an external computer.
- **Upgrade motors** to better compensate for the robot's weight and improve mobility.
- **Replace ultrasonic sensors with a LiDAR** for more accurate obstacle detection and odometry.
- **Add a robotic arm**, similar to Boston Dynamics' Spot, to enable object manipulation.
