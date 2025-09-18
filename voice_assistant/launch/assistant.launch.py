from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    voice_assistant_node = Node(
        package='voice_assistant',
        executable='voice_assistant_node',
        name='voice_assistant_node',
        output='screen'
    )


    sensor_publisher_node = Node(
        package='voice_assistant',
        executable='sensor_publisher_node',
        name='sensor_publisher_node',
        output='screen'
    )
    status_aggregator_node = Node(
        package='voice_assistant',
        executable='status_aggregator_node',
        name='status_aggregator_node',
        output='screen'
    )
    return LaunchDescription([
        voice_assistant_node,
        sensor_publisher_node, 
        status_aggregator_node,
    ])