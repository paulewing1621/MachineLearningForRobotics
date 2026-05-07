import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='floor_plane_mapping',
            executable='floor_map',
            name='floor_map',
            output='screen',
            parameters=[
                {'~/base_frame': 'bubbleRob'},
                {'~/world_frame': 'world'},
                {'~/n_samples': 300},
                {'~/tolerance': 0.06},       
                {'~/tilt_deg_max': 20.0},    
                {'~/prior_trav': 0.5},       
                {'~/p_hit_trav': 0.8},       
                {'~/p_hit_occ': 0.2},        
                {'~/range_sigma': 2.0},     
                {'~/min_weight': 0.2},       
                {'~/L_clip': 6.0},           
                {'~/thr_trav': 0.6},         
                {'~/thr_occ': 0.4},
                {'~/max_range': 4.5},
                {'~/res': 0.10},
                {'~/z_floor_max': 1000.0},
                {'~/min_points_trav': 3},
                {'~/flip_vertical': False},   
            ],
            remappings=[
                ('~/scans', '/points'),
                ('~/traversability_map',    '/traversability/map'),
                ('~/traversability_image',  '/traversability/image'),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('vrep4_helpers'),
                    'kinect_pc_min.launch.py'
                )
            )
        ),
    ])
