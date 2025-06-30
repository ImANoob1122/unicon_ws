import os
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name='config_path', description='Path to the config file'),
        Node(
            package='rogilink_flex_gui',
            executable='gui',
            name='gui',
            output='screen',
            parameters=[
                {
                    'config_path': LaunchConfiguration('config_path')
                }
            ]
        ),
        Node(
            package='rogilink_flex',
            executable='rogilink_flex',
            name='rogilink_flex_uart_node',
            output='screen',
            parameters=[
                {
                    'config_path': LaunchConfiguration('config_path')
                }
            ]
        ),
    ])

if __name__ == "__main__":
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()