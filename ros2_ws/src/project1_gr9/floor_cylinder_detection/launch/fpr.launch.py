import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='floor_cylinder_detection',
            executable='cylinder_detection',
            name='cylinder_detection',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'~/world_frame': 'world'},
                {'~/base_frame': 'bubbleRob'},
                {'~/max_range': 3.0},
                {'~/voxel_leaf': 0.03},
                {'~/ground_ransac': 300},
                {'~/ground_tol': 0.02},
                {'~/belt_min': 0.10},
                {'~/belt_max': 0.60},
                {'~/r_min': 0.11}, {'~/r_max': 0.14},
                {'~/n_samples': 800},
                {'~/dist_tol': 0.02},
                {'~/min_inliers': 70},
                {'~/min_cov_deg': 190.0},
                {'~/merge_center_tol': 0.08},
                {'~/merge_radius_tol': 0.02},
            ],
            remappings=[('~/scans','/points')],
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
