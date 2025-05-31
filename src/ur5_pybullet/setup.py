from setuptools import setup
import os
from glob import glob

package_name = 'ur5_pybullet'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='A ROS2 package for learning from demonstration using PyBullet simulation',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_pybullet_bridge = ur5_pybullet.ros2_pybullet_bridge:main',
        ],
    },
) 