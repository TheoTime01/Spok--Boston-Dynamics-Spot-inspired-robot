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
        'joystick_controller = quadruped_robot.joystick_controller:main',
        'camera_sonar_view_node = quadruped_robot.camera_sonar_view:main',
        'lidar_scan_republisher = quadruped_robot.lidar_scan_republisher:main',
        'face_detection_node = quadruped_robot.face_detection_node:main',
        'scenario = quadruped_robot.scenario:main',
        ]
    },
)
