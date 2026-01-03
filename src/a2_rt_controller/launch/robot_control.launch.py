from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='a2_rt_controller',
            executable='robot_controller',
            name='robot_controller',
            output='screen'
        ),
        Node(
            package='a2_rt_controller',
            executable='terminal_interface',
            name='terminal_interface',
            output='screen',
            prefix='xterm -e',
        ),
    ])
