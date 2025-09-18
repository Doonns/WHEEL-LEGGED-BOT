from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmd_vel_serial',
            executable='cmd_vel_to_serial',
            name='cmd_vel_to_serial',
            output='screen',
        ),
    ])
