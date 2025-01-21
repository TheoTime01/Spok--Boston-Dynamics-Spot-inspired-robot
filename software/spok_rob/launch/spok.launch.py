from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spok_rob',
            executable='joint_servo_controller',
            name='joint_servo_controller_node'
        ),
        Node(
            package='spok_rob',
            executable='mpu6050_node',
            name='mpu6050_node'
        ),
        Node(
            package='spok_rob',
            executable='gyro_node',
            name='gyro_node'
        ),
        Node(
            package='spok_rob',
            executable='word_detect_node',
            name='word_detect_node'
        ),

        Node(
            package='spok_rob',
            executable='connection_node',
            name='connection_node'
        ),

        Node(
            package='spok_rob',
            executable='word_detect_node',
            name='word_detect_node'
        ),

        Node(
            package='usb_cam',
            executable='usb_cam_node',
            name='usb_cam_node',
            output='screen',
            parameters=[
                {
                    'video_device': '/dev/video0',
                    'image_width': 640,
                    'image_height': 480,
                    'framerate': 15,
                    'pixel_format': 'mjpeg'
                }
            ]
        )
        
    ])