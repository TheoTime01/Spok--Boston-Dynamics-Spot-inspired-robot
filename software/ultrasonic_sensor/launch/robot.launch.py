from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to your files
    package_dir = get_package_share_directory('ultrasonic_sensor')
    xacro_file = os.path.join(package_dir, 'urdf', 'medor.xacro')
    gazebo_file = os.path.join(package_dir, 'urdf', 'fb.gazebo')
    world_file = os.path.join(package_dir, 'worlds', 'empty.world')

    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Node to process the Xacro file
    robot_description = Command(
        [
            'xacro ',
            xacro_file
        ]
    )

    # Gazebo Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gazebo_launch,
        robot_state_publisher
    ])
