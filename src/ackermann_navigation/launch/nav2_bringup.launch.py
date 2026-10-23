import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    navigation_pkg = get_package_share_directory('ackermann_navigation')

    default_params_path = os.path.join(
        navigation_pkg,
        'config',
        'nav2_params.yaml'
    )
    navigate_to_pose_tree = os.path.join(
        navigation_pkg,
        'behavior_trees',
        'navigate_to_pose_ackermann.xml'
    )
    navigate_through_poses_tree = os.path.join(
        navigation_pkg,
        'behavior_trees',
        'navigate_through_poses_ackermann.xml'
    )

    params_file = LaunchConfiguration('params_file')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        # controller_server publish "cmd_vel_nav" (chua qua smoother)
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            params_file,
            {
                'default_nav_to_pose_bt_xml': navigate_to_pose_tree,
                'default_nav_through_poses_bt_xml': navigate_through_poses_tree,
            },
        ]
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file]
    )

    # Lam min lenh van toc truoc khi day xuong ackerman_kinematic:
    # nhan "cmd_vel_nav" tu controller_server, xuat ra "/cmd_vel" that
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {
                'autostart': True,
                'node_names': lifecycle_nodes,
            }
        ]
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_path,
            description='Duong dan file nav2_params.yaml'
        ),
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        lifecycle_manager_node,
    ])
