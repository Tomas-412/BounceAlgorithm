import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bounce_alg',
            executable='bounce_alg_node',
            name='bounce_alg_node',
            output='screen',
            parameters=[os.path.join(os.path.dirname(__file__), '..', 'config', 'config.yaml')],

        )
    ])