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
            package='shore_follower_observe2',       
            executable='shore_follower_observe2',    
            name='shore_follower_observe2',

            parameters=[
                {'~/base_frame': 'VSV/Kision_sensor'},
                {'~/analysis_frame': 'world'},       
                {'~/out_dir': "/tmp"},
                
                {'~/min_displacement_xy': 0.01},
                {'~/min_z_displacement': 0.005},
                {'~/max_image_per_type': 1500},
                {'~/joystick_button': 4},

                {'~/prediction_up_threshold': 0.02},
                {'~/prediction_down_threshold': -0.02},
                {'~/prediction_level_threshold': 0.01},
            ],

            remappings=[
                ('~/joy', '/joy'),
                ('~/image', '/vrep/kision/image'),
                ('~/twist', '/arm_ik/twist'),
            ],

            output='screen'
        ),
    ])
