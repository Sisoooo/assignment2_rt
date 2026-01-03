from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='a2_rt_interfaces',
            executable='threshold_service',
            name='threshold_service',
            output='screen',
            prefix='xterm -e',
        ),
        Node(
            package='a2_rt_interfaces',
            executable='avg_vel_service',
            name='avg_vel_service',
            output='screen',
            prefix='xterm -e',
        ),
    ])