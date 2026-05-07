from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([

        launch_ros.actions.Node(
            package='floor_follower',
            executable='floor_follower_node',
            name='floor_follower',
            output='screen',
            parameters=[
                {"forward_speed": 0.3},  
                {"turn_speed": 0.06}     
            ],
            remappings=[
                ("image", "/vrep/kision/image"),          
                ("twist", "/vsv_driver/twistCommand"),         
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('vrep_ros_teleop'),
                '/teleop_mux.launch.py'
            ])
        )
    ])
