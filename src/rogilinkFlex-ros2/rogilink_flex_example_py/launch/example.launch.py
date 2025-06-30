import os
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rogilink_flex_example_py',
            executable='example_node',
            name='example_node'
        ),
        Node(
            package='rogilink_flex',
            executable='rogilink_flex',
            name='rogilink_flex',
            parameters=[{
                'config_path': (Path(os.environ['CATCHROBO_2024_WS']) / 'settings' / 'rogilinkflex.json').as_posix()
            }]
        )
    ])

if __name__ == "__main__":
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()