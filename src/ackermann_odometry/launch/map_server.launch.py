import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    map_yaml_path = LaunchConfiguration('map')

    default_map_path = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws',
        'maps',
        'my_map.yaml'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': map_yaml_path,
            }
        ]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {
                'autostart': True,
                'node_names': ['map_server'],
            }
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map_path,
            description='Path to map yaml'
        ),


        map_server_node,

        lifecycle_manager_node,
    ])
