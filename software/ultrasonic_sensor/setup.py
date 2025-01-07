import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'ultrasonic_sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch' ), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config' ), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'   ), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'   ), glob('worlds/*')),
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
        ],
    },
)
