import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # =========================================================
    # Launch arguments
    # =========================================================

    slam_params_file = LaunchConfiguration('slam_params_file')
    slam_executable = LaunchConfiguration('slam_executable')

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

    default_mapping_config = os.path.join(
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
    #
    # executable mặc định: async_slam_toolbox_node (chế độ mapping)
    # để dùng localization, override qua launch argument:
    #   slam_executable:=localization_slam_toolbox_node
    #   slam_params_file:=<path_toi_slam_localization.yaml>
    # =========================================================

    slam_node = Node(
        package='slam_toolbox',
        executable=slam_executable,
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
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

    start_odom = TimerAction(
        period=3.0,
        actions=[
            odom_node
        ]
    )

    start_slam = TimerAction(
        period=6.0,
        actions=[
            slam_node
        ]
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_mapping_config,
            description='File config cho slam_toolbox (mapping hoac localization)'
        ),
        DeclareLaunchArgument(
            'slam_executable',
            default_value='async_slam_toolbox_node',
            description='async_slam_toolbox_node (mapping) hoac localization_slam_toolbox_node (localization)'
        ),

        start_description,
        start_odom,
        start_slam,
    ])