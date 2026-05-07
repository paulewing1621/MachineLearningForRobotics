import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='metal_detection_marker',
            executable='metal_detection',
            name='metal_detection',
            parameters=[
                {'world_frame': 'world'},            
                {'sensor_frame': 'VSV/Kision_sensor'},  
                {'threshold': 0.7},                 
                {'min_weight_to_publish': 5.0}, 
                {'cluster_timout': 0.5},
                {'zone_radius': 5.0}
            ],
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('vrep4_helpers'),
                '/kinect_pc_min.launch.py'
            ])
        )
    ])
