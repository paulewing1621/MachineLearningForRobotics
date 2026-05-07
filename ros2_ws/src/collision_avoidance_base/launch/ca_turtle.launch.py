import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='joy',
            parameters=[
                {'autorepeat_rate': 10.},
                {'dev': "/dev/input/js0"},
            ],
            output='screen'),

        launch_ros.actions.Node(
            package='topic_tools', executable='mux', name='cmd_mux',
            arguments=['/teleop/twistCommand_raw', '/teleop/twistCommand', '/mux/autoCommand'],
            parameters=[
                {'output_topic': '/teleop/twistCommand_raw'},
                {'input_topics': ['/teleop/twistCommand', '/mux/autoCommand']},
            ],
            output='screen'),

        launch_ros.actions.Node(
            package='vrep_ros_teleop', executable='teleop_node', name='teleop',
            parameters=[
                {'axis_linear_x': 1},
                {'axis_angular': 0},
                {'scale_linear_x': 0.2},
                {'scale_angular': 1.0},
                {'timeout': 10.0} 
            ],
            remappings=[
                ('~/twistCommand', '/teleop/twistCommand'),
            ],
            output='screen'),

        launch_ros.actions.Node(
            package='vrep_ros_teleop', executable='teleop_mux_node', name='teleop_mux',
            parameters=[
                {'joystick_button': 0},
                {'joystick_topic': '/teleop/twistCommand'},
                {'auto_button': 1},
                {'auto_topic': '/mux/autoCommand'}
            ],
            remappings=[
                ('select', '/cmd_mux/select'),
            ],
            output='screen'),

        launch_ros.actions.Node(
            package='collision_avoidance_base', 
            executable='collision_avoidance_base', 
            name='collision_avoidance',
            parameters=[
                {'safety_diameter': 1.0}, 
                {'ignore_diameter': 2.0},
                {'max_velocity': 1.0},
                {'only_forward': True},
            ],
            remappings=[
                ('~/scans', '/scan'),
                ('~/vel_input', '/teleop/twistCommand'), 
                ('~/vel_output', '/commands/velocity'), 
            ],
            output='screen'),
    ])
