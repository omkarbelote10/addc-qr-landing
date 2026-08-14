from setuptools import find_packages, setup

from glob import glob
import os

package_name = 'drone_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/landing.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omkar',
    maintainer_email='omkar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'vision_tracker_node = drone_control_pkg.vision_tracker_node:main',
        'mission_control_node = drone_control_pkg.mission_control_node:main',
        ],
    },
)
