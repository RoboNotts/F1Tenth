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
