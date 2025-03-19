"""
Launch module for the car's Simple Drive Mode (SDM).

To interact with the navigator, set goal positions in the simulator (with RViz)
or publish to the '/target_pos' topic.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_drive',
            executable='sdm_navigator',
            name='sdm_navigator'
        ),
    ])
