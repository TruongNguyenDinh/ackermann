import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('ackermann_description')
    xacro_file = os.path.join(description_share, 'urdf', 'ackermann.urdf.xacro')
    rviz_config = os.path.join(description_share, 'rviz2', 'rviz.rviz')

    wheelbase = LaunchConfiguration('wheelbase')
    lidar_x_rear_axle = LaunchConfiguration('lidar_x_rear_axle')
    lidar_height = LaunchConfiguration('lidar_height')
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' wheelbase:=', wheelbase,
            ' lidar_x_rear_axle:=', lidar_x_rear_axle,
            ' lidar_height:=', lidar_height,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        condition=IfCondition(LaunchConfiguration('publish_joint_states')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'wheelbase', default_value='0.3187', description='Wheelbase in meters'),
        DeclareLaunchArgument(
            'lidar_x_rear_axle', default_value='0.240',
            description='LiDAR optical frame x position from rear axle in meters'),
        DeclareLaunchArgument(
            'lidar_height', default_value='0.146',
            description='LiDAR optical frame height above ground in meters'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Reserved for a future map display; not loaded by this robot-only launch'),
        DeclareLaunchArgument(
            'publish_joint_states', default_value='true',
            description='Publish demo joint states for robot-only visualization'),
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='Open RViz for robot-only visualization'),
        robot_state_publisher,
        joint_state_publisher,
        rviz,
    ])
