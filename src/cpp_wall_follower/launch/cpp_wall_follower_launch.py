from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_wall_follower',
            executable='sensor',
            name='distance_sensor_node'
        ),
        Node(
            package='cpp_wall_follower',
            executable='filter',
            name='distance_filter_node',
            parameters=[{'alpha': 0.1}]
        )
    ])