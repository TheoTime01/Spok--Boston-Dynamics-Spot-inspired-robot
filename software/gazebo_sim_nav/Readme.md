# Overview of different folder
- /worlds : set of worlds for simulation into gazebo
- /maps: set of maps corresponding to the different simulated worlds. Allow to do navigation
- /params: set of param files especially for navigation purpose. all different configuration for the navigation are set here

# Prerequistes
- cyclonedds
```
   sudo apt install ros-humble-rmw-cyclonedds-cpp
```
- Nav2
```
    sudo apt install ros-humble-navigation2
    sudo apt install ros-humble-nav2-bringup
```
- Turtlebot simulation
```
    sudo apt install ros-humble-turtlebot3-gazebo
```

# Start configuration
- Configure your ROS to communicate only on localhost
```
export ROS_LOCALHOST_ONLY=1
```  
- Some issues as been identified with default dss use cyclonedds instead
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
- set default end directory 

```
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```
- Tips : Save all these commands into your `~/.bashrc` file

# Start Simlation for mapping

```
ros2 launch gazebo_sim_nav tb3_simulation_without_nav_launch.py headless:=False
```

```
ros2 launch slam_toolbox online_async_launch.py
```

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

# Start Simlation for navigation

- Launch Gazebo, Localization, navigation and tools.

```
cd src/training-turtlebot-simulator-student/simulation/gazebo/gazebo_sim_nav/params/
ros2 launch gazebo_sim_nav tb3_simulation_launch.py headless:=False params_file:="nav2_params_empty.yaml" map:="<your absolute map path>/<your map>.yaml"
```

- On rviz set "2d pose estimate" 