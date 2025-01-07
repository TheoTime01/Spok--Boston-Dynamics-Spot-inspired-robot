import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    description_path = LaunchConfiguration("description_path")
    base_frame = "base_link"

    curr_pkg = get_package_share_directory("quadruped_robot")

    ros_control_config = os.path.join(
        curr_pkg, "config/ros_control.yaml"
    )

    gait_config = os.path.join(curr_pkg, "config/spot_gait.yaml")
    links_config = os.path.join(curr_pkg, "config/spot_links.yaml")
    joints_config = os.path.join(curr_pkg, "config/spot_joints.yaml")
    default_model_path = os.path.join(curr_pkg, "urdf/spot.urdf.xacro")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="champ", description="Robot name"
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
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "true",
            "publish_foot_contacts": "true",
            "close_loop_odom": "true",
        }.items(),
    )

    joint_servo_controller_node = Node(
        package="quadruped_robot",
        executable="joint_servo_controller_node",
        output="screen",
        name="joint_servo_controller"
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
            bringup_ld,
            joint_servo_controller_node,
            joy,
            joy_controller,
        ]
    )
