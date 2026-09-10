import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
        }.items()
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
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        start_sim_slam,
        kinematic_node,
    ])