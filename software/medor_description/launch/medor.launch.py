import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    xacro_file_name = 'urdf/medor.xacro'
    xacro_f = os.path.join(
        get_package_share_directory('medor_description'),
        xacro_file_name)
    
    robot_desc = xacro.process_file(xacro_f).toxml() # il faut du coup aussi import xacro
    conf_file_rviz = os.path.join(get_package_share_directory('medor_description'),'config','medor.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', conf_file_rviz],
        ), 
        Node(
            package='medor_description',
            executable='medor_tf_broadcaster',
            name='medor_tf_broadcaster',
            output='screen',
        ),
    ]
)