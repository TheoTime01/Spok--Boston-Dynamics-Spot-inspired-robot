# Using navigation

# Install 

# OLD Readme info

- Configure env.

- need tp change DDS communications

```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

```
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

- Launch Gazebo, Localization, navigation and tools.

```
cd src/gazebo_sim_nav/params
ros2 launch gazebo_sim_nav tb3_simulation_launch.py headless:=False params_file:="nav2_params_local_global_obstacle.yaml"
```




```
ros2 launch nav2_bringup tb3_simulation_without_nav_launch.py headless:=False
```
- note `headless:=False` opena graphicla client of gazebo

- Launch the navigation 
```
ros2 launch nav2_bringup navigation_launch.py params:="nav2_params.yaml"
```
```
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False params_file:="nav2_params_local_obstacle.yaml"
```
## Pratical work
- static layers (local and global)
- test navigation an see failure

- inflate layers (global)
- test navigation an see behavior
- rqt_reconfigure may with inflate radius and cost factor what happen ?


- local cost map add unmapped obstacle
- add no mapped obstacle and see behavior
- add inflate layer
- add a voxel layer to detect 3d objects

display voxel 
```
ros2 run nav2_costmap_2d nav2_costmap_2d_markers voxel_grid:=/local_costmap/voxel_grid visualization_marker:=/my_marker
```
- use and understand combination methods if laser and voxel in the same layer

- when trying to move local (voxel)desapear when close to the obstacle why (clear costmap of recovery behavoir BT)


- Map obstacle on the global cost map
- explain why this is important

- see pb when inflate layer is too close obstacle (and especially 3d obstable)
- adjust inflate radius to avoid such situations


- understand the used behavior tree
- update the behavior tree



## Ref Nav: 
- https://navigation.ros.org/
- https://navigation.ros.org/behavior_trees/index.html
- https://roboticsbackend.com/ros2-nav2-tutorial/
- https://youtu.be/5mCebxxPXTY 
- https://www.youtube.com/watch?v=jkoGkAd0GYk
- https://automaticaddison.com/the-ultimate-guide-to-the-ros-2-navigation-stack/


- https://navigation.ros.org/configuration/index.html


## Ref Behavior tree:
- https://py-trees-ros-tutorials.readthedocs.io/en/devel/tutorials.html
- https://markus-x-buchholz.medium.com/behavior-trees-in-c-for-robotic-applications-ros2-775ec0e97856
-  https://youtu.be/KO4S0Lsba6I 
- For navigation: https://navigation.ros.org/behavior_trees/overview/detailed_behavior_tree_walkthrough.html
- https://github.com/Adlink-ROS/BT_ros2




