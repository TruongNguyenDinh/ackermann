import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    
    # =========================================================
    # Package description
    # =========================================================

    description_pkg = get_package_share_directory(
        'ackermann_description'
    )
    odometry_pkg = get_package_share_directory(
            'ackermann_odometry'
    )

    description_launch = os.path.join(
        description_pkg,
        'launch',
        'model.launch.py'
    )
    slam_config = os.path.join(
        odometry_pkg,
        'config',
        'slam.yaml'
    )
    # =========================================================
    # Ackermann odometry
    # =========================================================

    odom_node = Node(
        package='ackermann_odometry',
        executable='ackermann_odometry',
        name='ackermann_odometry',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    # =========================================================
    # SLAM Toolbox
    # =========================================================

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': True}
        ],
        remappings=[
            ('scan', '/scan')
        ]
    )

    # =========================================================
    # Launch description first
    # =========================================================

    start_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            description_launch
        )
    )

    # =========================================================
    # Start odometry after description
    # =========================================================

    start_odom = TimerAction(
        period=3.0,
        actions=[
            odom_node
        ]
    )

    # =========================================================
    # Start SLAM after odometry
    # =========================================================

    start_slam = TimerAction(
        period=6.0,
        actions=[
            slam_node
        ]
    )

    return LaunchDescription([
        start_description,
        start_odom,
        start_slam,
    ])