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
        'joystick_controller = quadruped_robot.joystick_controller:main'        ]
    },
)
