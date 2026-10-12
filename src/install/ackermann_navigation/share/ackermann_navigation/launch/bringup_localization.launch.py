import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # =========================================================
    # Package paths
    # =========================================================

    odometry_pkg = get_package_share_directory(
        'ackermann_odometry'
    )
    navigation_pkg = get_package_share_directory(
        'ackermann_navigation'
    )

    sim_slam_launch = os.path.join(
        odometry_pkg,
        'launch',
        'sim_slam.launch.py'
    )
    nav2_launch = os.path.join(
        navigation_pkg,
        'launch',
        'nav2_bringup.launch.py'
    )
    default_nav2_params = os.path.join(
        navigation_pkg,
        'config',
        'nav2_params.yaml'
    )
    wheelbase = LaunchConfiguration('wheelbase')

    # Config localization nam trong chinh package ackermann_navigation,
    # KHONG phai ackermann_odometry nua
    localization_config = os.path.join(
        navigation_pkg,
        'config',
        'slam.yaml'
    )

    # =========================================================
    # Include sim_slam.launch.py o che do localization
    # =========================================================

    start_sim_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_slam_launch),
        launch_arguments={
            'slam_params_file': localization_config,
            'slam_executable': 'localization_slam_toolbox_node',
            'wheelbase': wheelbase,
        }.items()
    )

    # Give Gazebo, odometry, and SLAM time to publish /scan, /odom, /map,
    # and map->odom before Nav2 activates its costmaps.
    start_navigation = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    'params_file': LaunchConfiguration('params_file'),
                    'use_sim_time': 'true',
                    'wheelbase': wheelbase,
                }.items()
            )
        ]
    )

    # =========================================================
    # Node ackerman_kinematic
    # =========================================================

    kinematic_node = Node(
        package='ackermann_kinematics_py',
        executable='ackerman_kinematic',
        name='ackerman_kinematic',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'wheelbase': ParameterValue(wheelbase, value_type=float),
            }
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params,
            description='Path to the Nav2 parameter file'
        ),
        DeclareLaunchArgument(
            'wheelbase',
            default_value='0.3187',
            description='Wheelbase in meters, shared by URDF and goal conversion',
        ),
        start_sim_slam,
        kinematic_node,
        start_navigation,
    ])
