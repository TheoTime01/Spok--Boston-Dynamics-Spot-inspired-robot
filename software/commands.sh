# Paste 'minispot' in src folder of your workspace

# Create a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src/

# Clone the repo
git clone https://github.com/Jagadeesh-pradhani/minispot.git -b spotmicro_gazebo_ros2
cd ~/ros2_ws/

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y  

# Build
colcon build  
source ~/ros2_ws/install/setup.bash



# Terminal-1
cd ~/ros2_ws/
source ~/ros2_ws/install/setup.bash
ros2 launch quadruped_robot spot_bringup.launch.py

# If using Joystick directly connect and run the robot


# For virtual joystick
# Terminal-2
cd ~/ros2_ws/
source ~/ros2_ws/install/setup.bash
sudo chmod +0666 /dev/uinput
ros2 run quadruped_robot virtual_joy_stick

# Click on Start and control robot



# For keyboard control
# Terminal-2
ros2 run teleop_twist_keyboard teleop_twist_keyboard


# IK files

# Main spot controller
# Location : minispot/champ_base/src/quadruped_controller.cpp

# Inputs/Subscriber : cmd_vel
#              body_pose

# Output : Moves the robot to desired position based on cmd_vel & body_pose

# Kinematics file : 
# Location : minispot/champ/include/champ/kinematics/kinematics.h

# Involves Inverse kinematics functions

