import os
from glob import glob
from setuptools import setup

package_name = 'quadruped_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch' ), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config' ), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'   ), glob('urdf/*')),
        (os.path.join('share', package_name, 'stl'   ), glob('stl/*')),
        (os.path.join('share', package_name, 'meshes'   ), glob('meshes/*')),
        (os.path.join('share', package_name, 'worlds'   ), glob('worlds/*')),
        (os.path.join('share', package_name, 'json'   ), glob('json/*')),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.yaml'))),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    extras_require={'test':['pytest'],},
    entry_points={
        'console_scripts':
        [
        'virtual_joy_stick = quadruped_robot.virtual_joy_stick:main',
        'joystick_controller = quadruped_robot.joystick_controller:main',
        'camera_sonar_view_node = quadruped_robot.camera_sonar_view:main',
        'manual_motion_node = quadruped_robot.manual_motion:main',
        'tkinter_node = quadruped_robot.tkinter_node:main',
        'camera_process_node = quadruped_robot.camera_process_node:main',
        'demo_16_node = quadruped_robot.demo_16_node:main',
        'lidar_scan_republisher = quadruped_robot.lidar_scan_republisher:main',
        'face_detectiuon_node = quadruped_robot.face_detection_node:main',
        ]
    },
)
