# SpotMicro ROS 2 Gazebo Simulation

## Project Overview
The SpotMicro ROS 2 Gazebo simulation project is a robotics simulation environment for SpotMicro, a quadruped robot. The project leverages ROS 2  and Gazebo, providing a realistic simulation platform to test and develop algorithms for locomotion, navigation, and control.

## Key Features
1. **Robot Model**:
   - A URDF  model of SpotMicro with accurate dimensions, joint configurations, and sensor placements.
   - Integration of CAD models for a realistic appearance.

2. **Simulation Environment**:
   - Gazebo integration for real-time physics simulation.
   - Customizable terrains and obstacles to evaluate robot performance.

3. **Control Interfaces**:
   - Joint position, velocity, and torque control.
   - Support for trajectory generation and inverse kinematics.

4. **Sensor Simulation**:
   - Simulated sensors, including IMU, LiDAR, and cameras.
   - ROS 2 topics for real-time sensor data streaming.

5. **Extensibility**:
   - Modular design to facilitate adding new sensors, actuators, or algorithms.
   - ROS 2 nodes for easy integration with other robotics projects.

---

## System Requirements

- **Operating System**: Ubuntu 22.04 or compatible.
- **Dependencies**:
  - ROS 2 Humble.
  - Gazebo Fortress.
  - Python 3.10 or later.
  - `rosdep` for dependency management.

---

## Installation Instructions
Follow `commands.sh`
### Step 1: Install ROS 2
Follow the official ROS 2 installation guide for your operating system. Ensure that the workspace is sourced correctly:
```bash
source /opt/ros/humble/setup.bash
```

### Step 2: Install Gazebo
Install Gazebo and source.
```bash
source /usr/share/gazebo/setup.sh
```

### Step 3: Clone the Repository
Clone the SpotMicro simulation repository to your workspace:
```bash
git clone https://github.com/Jagadeesh-pradhani/minispot.git -b spotmicro_gazebo_ros2
```

### Step 4: Install Dependencies
Navigate to the workspace and run `rosdep` to install the required dependencies:
```bash
cd ~/your_workspace
rosdep install --from-paths src --ignore-src -r -y
```

### Step 5: Build the Workspace
Build the workspace using `colcon`:
```bash
colcon build
```

### Step 6: Source the Workspace
Source the workspace to enable the ROS 2 packages:
```bash
source ~/your_workspace/install/setup.bash
```

---

## Usage Instructions

### Launch the Simulation
Run the following command to launch the Gazebo simulation:
```bash
ros2 launch quadruped_robot spot_bringup.launch.py
```

### Control the Robot
Use the Joystick or teleoperation node to control SpotMicro:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


---

## Project Structure

```plaintext
quadruped_robot/
├── launch/
│   ├── spot_bringup.launch.py
│   └── spot_controller.launch.py
├── urdf/
│   ├── spot.urdf.xacro
│   
├── worlds/
│   └── env.world
└── config/
```

---


# Explanation of `quadruped_controller.cpp`

This section explains the purpose, design, and implementation of the `quadruped_controller.cpp` file located in the `minispot/champ_base/src/` directory. This file defines the main logic for controlling a quadruped robot in a ROS 2-based simulation or real-world scenario.

---

## Purpose of the File

The `quadruped_controller.cpp` file implements a ROS 2 node responsible for controlling the motion and pose of the quadruped robot. It interfaces with the robot’s actuators and sensors, processes velocity and pose commands, and calculates joint positions using inverse kinematics.

### Key Responsibilities
- Convert high-level commands (velocity, pose) into joint-level commands.
- Publish control commands and robot state.
- Interface with simulation or real-world hardware.

---

## Key Components

### 1. **ROS Node Initialization**
The `QuadrupedController` class initializes the ROS 2 node and sets up publishers, subscribers, and parameters.

#### Code Highlights:
```cpp
QuadrupedController::QuadrupedController()
    : Node("quadruped_controller_node", rclcpp::NodeOptions()
                  .allow_undeclared_parameters(true)
                  .automatically_declare_parameters_from_overrides(true)),
      clock_(*this->get_clock()),
      body_controller_(base_),
      leg_controller_(base_, rosTimeToChampTime(clock_.now())),
      kinematics_(base_)
```
- The node is named `quadruped_controller_node`.
- It uses `NodeOptions` to enable dynamic parameter declaration.
- Initializes controllers for body, legs, and kinematics.

### 2. **Time Conversion Function**
This helper function converts ROS 2 time format:
```cpp
champ::PhaseGenerator::Time rosTimeToChampTime(const rclcpp::Time& time)
{
  return time.nanoseconds() / 1000ul;
}
```

### 3. **Parameter Configuration**
The constructor loads various gait and control parameters:
```cpp
this->get_parameter("gait.max_linear_velocity_x",  gait_config_.max_linear_velocity_x);
this->get_parameter("gait.max_angular_velocity_z", gait_config_.max_angular_velocity_z);
this->get_parameter("publish_joint_control",       publish_joint_control_);
```
- Parameters control gait configuration (e.g., maximum velocities, stance height).
- Supports publishing options for joint states and foot contacts.

### 4. **Subscribers and Publishers**
#### Subscribers:
- **`cmd_vel`**: Subscribes to velocity commands.
- **`cmd_pose`**: Subscribes to body pose commands.
```cpp
cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel/smooth", 10, std::bind(&QuadrupedController::cmdVelCallback_, this, std::placeholders::_1));
cmd_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "body_pose", 1, std::bind(&QuadrupedController::cmdPoseCallback_, this, std::placeholders::_1));
```
#### Publishers:
- Publishes joint commands, joint states, and foot contacts based on configuration.

---

## Control Loop Implementation
The `controlLoop_` function executes periodically to calculate and publish commands:
```cpp
void QuadrupedController::controlLoop_()
{
    float target_joint_positions[12];
    geometry::Transformation target_foot_positions[4];
    bool foot_contacts[4];

    body_controller_.poseCommand(target_foot_positions, req_pose_);
    leg_controller_.velocityCommand(target_foot_positions, req_vel_, rosTimeToChampTime(clock_.now()));
    kinematics_.inverse(target_joint_positions, target_foot_positions);

    publishFootContacts_(foot_contacts);
    publishJoints_(target_joint_positions);
    req_vel_old_ = req_vel_;
}
```
### Key Steps:
1. **Body and Leg Control**:
   - `poseCommand`: Calculates target foot positions based on body pose.
   - `velocityCommand`: Calculates foot positions from velocity commands.

2. **Inverse Kinematics**:
   - Converts foot positions to joint angles.

3. **Publishing**:
   - Calls helper functions to publish foot contacts and joint states.

---

## Callbacks
### Velocity Command Callback
Processes incoming velocity commands:
```cpp
void QuadrupedController::cmdVelCallback_(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    req_vel_.linear.x = msg->linear.x;
    req_vel_.linear.y = msg->linear.y;
    req_vel_.angular.z = msg->angular.z;
}
```
### Pose Command Callback
Processes incoming pose commands and extracts roll, pitch, yaw:
```cpp
void QuadrupedController::cmdPoseCallback_(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    tf2::Quaternion quat(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    req_pose_.orientation.roll = roll;
    req_pose_.orientation.pitch = pitch;
    req_pose_.orientation.yaw = yaw;

    req_pose_.position.x = msg->position.x;
    req_pose_.position.y = msg->position.y;
    req_pose_.position.z = msg->position.z + gait_config_.nominal_height;
}
```
---

## Publishing Functions
### Joint Commands
Publishes joint position commands:
```cpp
void QuadrupedController::publishJoints_(float target_joints[12])
{
    if(publish_joint_control_)
    {
        trajectory_msgs::msg::JointTrajectory joints_cmd_msg;
        joints_cmd_msg.header.stamp = clock_.now();
        joints_cmd_msg.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(12);

        for(size_t i = 0; i < 12; i++)
        {
            point.positions[i] = target_joints[i];
        }

        joints_cmd_msg.points.push_back(point);
        joint_commands_publisher_->publish(joints_cmd_msg);
    }
}
```
### Foot Contacts
Publishes foot contact states:
```cpp
void QuadrupedController::publishFootContacts_(bool foot_contacts[4])
{
    if(publish_foot_contacts_ && !in_gazebo_)
    {
        champ_msgs::msg::ContactsStamped contacts_msg;
        contacts_msg.header.stamp = clock_.now();
        for(size_t i = 0; i < 4; i++)
        {
            contacts_msg.contacts[i] = base_.legs[i]->gait_phase();
        }
        foot_contacts_publisher_->publish(contacts_msg);
    }
}
```
---

## Conclusion Controller
The `quadruped_controller.cpp` file is the core component for controlling the quadruped robot. It efficiently manages the robot’s motion by converting high-level commands into actuator commands while maintaining modularity and extensibility for different environments and configurations.

# Inverse Kinematics: Explanation and Details

## Overview
Inverse Kinematics (IK) is a mathematical method used to calculate the joint angles required for a robot’s end-effector (in this case, the feet of a quadruped) to reach a desired position in 3D space. This document provides an in-depth explanation of the **inverse kinematics** implementation located in `minispot/champ/include/kinematics/kinematics.h`.

The IK implementation in this code is designed for a quadruped robot and calculates joint angles for each of the robot's four legs, ensuring that the feet reach specified positions while maintaining joint constraints and avoiding unreachable configurations.

---

## Key Components of the Code

### 1. **Class: `Kinematics`**
This class provides methods for both forward and inverse kinematics. Our focus is on the `inverse` methods:

- **`void inverse(float (&joint_positions)[12], geometry::Transformation (&foot_positions)[4])`**: Computes the joint angles for all four legs.
- **`static void inverse(float &hip_joint, float &upper_leg_joint, float &lower_leg_joint, champ::QuadrupedLeg &leg, geometry::Transformation &foot_position)`**: Calculates the joint angles for a single leg.

---

### 2. **Detailed Explanation of the Inverse Kinematics Method**

#### **High-Level Workflow**
1. **Foot Position Translation**: Transform the desired foot position into the local frame of the leg's hip joint.
2. **Joint Length Calculations**: Use the geometry of the leg (upper leg, lower leg) to determine feasible joint angles.
3. **Reachability Check**: Ensure the desired position is within the reachable workspace of the leg.
4. **Angle Calculations**: Use trigonometric relationships to compute joint angles for the hip, upper leg, and lower leg.
5. **Adjust for Joint Constraints**: Modify angles to maintain physically plausible configurations.

#### **Code Breakdown**

```cpp
static void inverse(float &hip_joint, float &upper_leg_joint, float &lower_leg_joint,
                    champ::QuadrupedLeg &leg, geometry::Transformation &foot_position)
```

##### **Inputs**:
- `hip_joint`, `upper_leg_joint`, `lower_leg_joint`: Output joint angles.
- `leg`: The structure representing the current leg (including its dimensions and joint chain).
- `foot_position`: The desired foot position in the global coordinate frame.

##### **Steps**:

1. **Initialize Variables**
   ```cpp
   geometry::Transformation temp_foot_pos = foot_position;
   float l0 = 0.0f;
   ```
   - `temp_foot_pos` is a copy of the foot position for manipulation.
   - `l0` accumulates the vertical offsets of joints in the leg.

2. **Calculate Vertical Offset (`l0`)**
   ```cpp
   for(unsigned int i = 1; i < 4; i++) {
       l0 += leg.joint_chain[i]->y();
   }
   ```
   - Sum the y-coordinates of the joints to compute the effective vertical offset.

3. **Calculate Effective Lengths**
   ```cpp
   float l1 = -sqrtf(pow(leg.lower_leg.x(), 2) + pow(leg.lower_leg.z(), 2));
   float l2 = -sqrtf(pow(leg.foot.x(), 2) + pow(leg.foot.z(), 2));
   ```
   - Compute the lengths of the lower leg (`l1`) and foot segment (`l2`).

4. **Calculate Intermediate Angles**
   ```cpp
   float ik_alpha = acosf(leg.lower_leg.x() / l1) - (M_PI / 2);
   float ik_beta = acosf(leg.foot.x() / l2) - (M_PI / 2);
   ```
   - `ik_alpha` and `ik_beta` account for the offset between the local axes and the actual joint axis.

5. **Compute Hip Joint Angle**
   ```cpp
   hip_joint = -(atanf(y / z) - ((M_PI/2) - acosf(-l0 / sqrtf(pow(y, 2) + pow(z, 2)))));
   ```
   - The hip joint angle is computed using trigonometry to align the leg with the desired foot position.

6. **Transform Foot Position**
   ```cpp
   temp_foot_pos.RotateX(-hip_joint);
   temp_foot_pos.Translate(-leg.upper_leg.x(), 0.0f, -leg.upper_leg.z());
   ```
   - Rotate and translate the foot position to align it with the upper leg's frame.

7. **Reachability Check**
   ```cpp
   float target_to_foot = sqrtf(pow(x, 2) + pow(z,2));
   if(target_to_foot >= (abs(l1) + abs(l2)))
       return;
   ```
   - If the target position is out of reach, the function exits early.

8. **Compute Upper and Lower Leg Angles**
   ```cpp
   lower_leg_joint = leg.knee_direction() * acosf((pow(z, 2) + pow(x, 2) - pow(l1 ,2) - pow(l2 ,2)) / (2 * l1 * l2));
   upper_leg_joint = (atanf(x / z) - atanf((l2 * sinf(lower_leg_joint)) / (l1 + (l2 * cosf(lower_leg_joint)))));
   ```
   - Using trigonometry, compute the joint angles for the upper and lower leg.

9. **Adjust for Constraints**
   ```cpp
   lower_leg_joint += ik_beta - ik_alpha;
   upper_leg_joint += ik_alpha;

   if(leg.knee_direction() < 0) {
       if(upper_leg_joint < 0) {
           upper_leg_joint += M_PI;
       }
   } else {
       if(upper_leg_joint > 0) {
           upper_leg_joint += M_PI;
       }
   }
   ```
   - Adjust the joint angles to ensure they remain within valid ranges.

---

### 3. **Reachability and Error Handling**

The function ensures that the desired foot position is physically reachable by the leg. If the target is unreachable (e.g., due to exceeding the leg’s maximum extension), the function exits without modifying joint angles.

---

## Summary IK
This implementation provides a robust method for computing joint angles for a quadruped robot’s legs. The approach ensures that the target position is reachable and respects the mechanical constraints of the robot.

Key features include:
- Use of geometric relationships for accurate angle computation.
- Reachability checks to avoid invalid configurations.
- Adjustments for joint constraints to maintain physical feasibility.

This inverse kinematics algorithm is crucial for tasks like walking, climbing, or navigating uneven terrain in a simulated or real-world environment.


