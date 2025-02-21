from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'spok_rob'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tototime',
    maintainer_email='perricht@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_servo_controller_node = spok_rob.joint_servo_controller:main',
            'gyro_node = spok_rob.gyro_node:main',
            'camera_node = spok_rob.camera_node:main',
            'word_detect_node = spok_rob.word_detect_node:main',
            'connection_node = spok_rob.connection_node:main',
            'mpu6050_node = spok_rob.mpu6050_node:main',

        ],
    },
)
