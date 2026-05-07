from setuptools import setup
from glob import glob
import os

package_name = 'floor_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Register package for ament_index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include package.xml
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pewing',
    maintainer_email='pewing@todo.todo',
    description='Floor follower robot using camera input to steer based on floor color',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'floor_follower_node = floor_follower.floor_follower_node:main',
        ],
    },
)
