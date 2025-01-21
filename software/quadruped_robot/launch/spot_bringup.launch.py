import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    description_path = LaunchConfiguration("description_path")
    base_frame = "base_link"

    curr_pkg = launch_ros.substitutions.FindPackageShare(
        package="quadruped_robot"
    ).find("quadruped_robot")




    
    ros_control_config = os.path.join(
        curr_pkg, "/config/ros_control.yaml"
    )

    # Spot
    gait_config = os.path.join(curr_pkg, "config/spot_gait.yaml")
    links_config = os.path.join(curr_pkg, "config/spot_links.yaml")
    joints_config = os.path.join(curr_pkg, "config/spot_joints.yaml")
    default_model_path = os.path.join(curr_pkg, "urdf/spot_without_lidar_imu.urdf.xacro")

    # Champ
    # gait_config = os.path.join(curr_pkg, "config/champ_gait.yaml")
    # links_config = os.path.join(curr_pkg, "config/champ_links.yaml")
    # joints_config = os.path.join(curr_pkg, "config/champ_joints.yaml")
    # default_model_path = os.path.join(curr_pkg, "urdf/champ.urdf.xacro")

    

    default_world_path = os.path.join(curr_pkg, "worlds/1st_floor.world")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="champ", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=ros_control_config,
        description="Ros control config path",
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path, description="Gazebo world name"
    )

    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.6"
    )


    bringup_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("quadruped_robot"),
                "launch",
                "spot_controller.launch.py",
            )
        ),
        launch_arguments={
            "description_path": default_model_path,
            "joints_map_path": joints_config,
            "links_map_path": links_config,
            "gait_config_path": gait_config,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "gazebo": "true",
            "lite": LaunchConfiguration("lite"),
            "rviz": LaunchConfiguration("rviz"),
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "false",
            "close_loop_odom": "true",
        }.items(),
    )

    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("quadruped_robot"),
                "launch",
                "spot_gazebo.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "world": LaunchConfiguration("world"),
            "lite": LaunchConfiguration("lite"),
            "world_init_x": LaunchConfiguration("world_init_x"),
            "world_init_y": LaunchConfiguration("world_init_y"),
            "world_init_heading": LaunchConfiguration("world_init_heading"),
            "gui": LaunchConfiguration("gui"),
            "close_loop_odom": "true",
            "links_map_path": links_config,
        }.items(),
    )

    joy = Node(
        package="joy",
        output="screen",
        executable="joy_node",
        parameters=[{'use_sim_time': use_sim_time}]
    )

    joy_controller = Node(
        package="quadruped_robot",
        output="screen",
        executable="joystick_controller"
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_rviz,
            declare_robot_name,
            declare_lite,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_gui,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_heading,
            bringup_ld,
            gazebo_ld,
            joy,
            joy_controller
        ]
    )
