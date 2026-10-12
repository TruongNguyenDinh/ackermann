import os
import glob

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    RegisterEventHandler,
    ExecuteProcess,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def find_gazebo_system_share_dir():
    """
    Tìm thư mục hệ thống của Gazebo (chứa 'media', bao gồm shader lib
    dùng cho RTShaderSystem). Đây là thư mục thường bị THIẾU khi ta tự
    set GAZEBO_RESOURCE_PATH bằng tay ở một phiên shell chưa từng
    source /usr/share/gazebo/setup.sh, dẫn tới lỗi:

        Unable to find shader lib... GAZEBO_RESOURCE_PATH is probably
        improperly set

    và crash 'Camera px != 0' vì scene không dựng được.
    """
    candidates = sorted(glob.glob('/usr/share/gazebo-*')) + ['/usr/share/gazebo']
    for candidate in candidates:
        if os.path.exists(os.path.join(candidate, 'media')):
            return candidate
    return None


def generate_launch_description():

    # =========================================================
    # PARAMETERS
    # =========================================================

    use_sim_time = LaunchConfiguration('use_sim_time')
    wheelbase = LaunchConfiguration('wheelbase')
    gui = LaunchConfiguration('gui')

    # =========================================================
    # PACKAGE PATHS
    # =========================================================

    ackermann_pkg = get_package_share_directory(
        'ackermann_description'
    )

    warehouse_pkg = get_package_share_directory(
        'aws_robomaker_small_warehouse_world'
    )

    # =========================================================
    # FILE PATHS
    # =========================================================

    xacro_file = os.path.join(
        ackermann_pkg,
        'urdf',
        'ackermann.urdf.xacro'
    )

    rviz_config_path = os.path.join(
        ackermann_pkg,
        'rviz2',
        'rviz.rviz'
    )

    warehouse_world = os.path.join(
        warehouse_pkg,
        'worlds',
        'no_roof_small_warehouse',
        'no_roof_small_warehouse.world'
    )

    warehouse_models = os.path.join(
        warehouse_pkg,
        'models'
    )

    # =========================================================
    # CHECK PATHS
    # =========================================================

    if not os.path.exists(warehouse_world):
        raise FileNotFoundError(
            f'Warehouse world not found:\n{warehouse_world}'
        )

    if not os.path.exists(warehouse_models):
        raise FileNotFoundError(
            f'Warehouse models directory not found:\n{warehouse_models}'
        )

    # =========================================================
    # GAZEBO ENVIRONMENT
    #
    # GAZEBO_MODEL_PATH: thư mục models của warehouse + ~/.gazebo/models
    #                     + giá trị cũ (nếu shell đã có sẵn)
    #
    # GAZEBO_RESOURCE_PATH: thư mục gốc package warehouse
    #                     + thư mục hệ thống Gazebo (BẮT BUỘC, chứa
    #                       shader lib cho RTShaderSystem — nếu thiếu
    #                       sẽ gây lỗi "Failed to initialize scene" và
    #                       crash gzclient/gazebo ngay lập tức)
    #                     + giá trị cũ (nếu shell đã có sẵn)
    # =========================================================

    home_gazebo_models = os.path.join(os.path.expanduser('~'), '.gazebo', 'models')
    gazebo_system_share = find_gazebo_system_share_dir()

    gazebo_model_path_parts = [warehouse_models, home_gazebo_models]
    if os.environ.get('GAZEBO_MODEL_PATH'):
        gazebo_model_path_parts.append(os.environ['GAZEBO_MODEL_PATH'])
    gazebo_model_path_value = os.pathsep.join(gazebo_model_path_parts)

    gazebo_resource_path_parts = [warehouse_pkg]
    if gazebo_system_share:
        gazebo_resource_path_parts.append(gazebo_system_share)
    if os.environ.get('GAZEBO_RESOURCE_PATH'):
        gazebo_resource_path_parts.append(os.environ['GAZEBO_RESOURCE_PATH'])
    gazebo_resource_path_value = os.pathsep.join(gazebo_resource_path_parts)

    if gazebo_system_share is None:
        print(
            '[model.launch.py] CANH BAO: khong tim thay thu muc he thong '
            'cua Gazebo (vi du /usr/share/gazebo-11). Shader lib co the '
            'khong duoc tim thay va gazebo/gzclient co the crash. '
            'Hay kiem tra Gazebo da cai dung chua.'
        )

    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path_value
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=gazebo_resource_path_value
    )

    # =========================================================
    # ROBOT DESCRIPTION
    # =========================================================

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file,
            ' wheelbase:=',
            wheelbase,
        ]),
        value_type=str
    )

    # =========================================================
    # Keep Gazebo physics and ROS services in gzserver; launch its GUI only
    # when explicitly requested.
    # =========================================================

    gazebo_process = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            warehouse_world,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen',
        additional_env={
            'GAZEBO_MODEL_PATH': gazebo_model_path_value,
            'GAZEBO_RESOURCE_PATH': gazebo_resource_path_value,
        }
    )

    gazebo_gui_process = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gui),
        additional_env={
            'GAZEBO_MODEL_PATH': gazebo_model_path_value,
            'GAZEBO_RESOURCE_PATH': gazebo_resource_path_value,
        }
    )

    # =========================================================
    # ROBOT STATE PUBLISHER
    # =========================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }
        ]
    )

    # =========================================================
    # SPAWN ACKERMANN ROBOT
    # =========================================================

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'ackermann',
            '-timeout', '120.0',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # =========================================================
    # JOINT STATE BROADCASTER
    # =========================================================

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster'
        ],
        output='screen'
    )
    steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steering_controller'],
        output='screen'
    )

    wheel_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_velocity_controller'],
        output='screen'
    )
    # =========================================================
    # RVIZ2
    # =========================================================

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            rviz_config_path
        ],
        parameters=[
            {
                'use_sim_time': use_sim_time
            }
        ]
    )

    # =========================================================
    # LAUNCH DESCRIPTION
    # =========================================================

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'wheelbase',
            default_value='0.3187',
            description='Wheelbase in meters, passed to the robot Xacro',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Launch the Gazebo GUI client',
        ),

        gazebo_model_path,
        gazebo_resource_path,

        gazebo_process,
        gazebo_gui_process,

        robot_state_publisher,

        TimerAction(
            period=6.0,
            actions=[
                spawn_robot
            ]
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[
                    joint_state_broadcaster_spawner,
                    steering_controller_spawner,
                    wheel_velocity_controller_spawner,
                ],
            )
        ),

        TimerAction(
            period=7.0,
            actions=[
                rviz
            ]
        )
    ])
