"""Record a robot run with rosbag2 and optional static-map snapshot."""

from datetime import datetime, timezone
import hashlib
import os
from pathlib import Path
import re
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import yaml


def _sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _read_topics(path):
    with path.open('r', encoding='utf-8') as source:
        config = yaml.safe_load(source)
    if not isinstance(config, dict) or not isinstance(config.get('topics'), list):
        raise ValueError('topics_file must contain a topics list')
    topics = config['topics']
    if not topics or any(not isinstance(topic, str) or not topic.startswith('/') for topic in topics):
        raise ValueError('Every topic must be a nonempty absolute ROS topic name')
    if len(topics) != len(set(topics)):
        raise ValueError('topics_file contains duplicate topic names')
    return topics


def _copy_map(map_yaml_path, session_dir):
    with map_yaml_path.open('r', encoding='utf-8') as source:
        map_config = yaml.safe_load(source)
    if not isinstance(map_config, dict) or not isinstance(map_config.get('image'), str):
        raise ValueError('map_yaml must contain an image path')

    source_image = Path(map_config['image']).expanduser()
    if not source_image.is_absolute():
        source_image = map_yaml_path.parent / source_image
    source_image = source_image.resolve(strict=True)

    map_dir = session_dir / 'map'
    map_dir.mkdir()
    target_image = map_dir / source_image.name
    shutil.copy2(source_image, target_image)
    map_config['image'] = source_image.name
    with (map_dir / 'map.yaml').open('w', encoding='utf-8') as target:
        yaml.safe_dump(map_config, target, sort_keys=False)
    return {
        'source_yaml': str(map_yaml_path),
        'source_yaml_sha256': _sha256(map_yaml_path),
        'source_image_sha256': _sha256(source_image),
        'snapshot_yaml': 'map/map.yaml',
    }


def _start_recording(context):
    topics_file = Path(LaunchConfiguration('topics_file').perform(context)).expanduser().resolve(strict=True)
    topics = _read_topics(topics_file)
    output_dir = Path(LaunchConfiguration('output_dir').perform(context)).expanduser()
    requested_name = LaunchConfiguration('session_name').perform(context).strip()
    session_name = requested_name or datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%fZ')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', session_name):
        raise ValueError('session_name may contain only letters, digits, _, -, and .')

    session_dir = output_dir / session_name
    if session_dir.exists():
        raise FileExistsError(f'Recording session already exists: {session_dir}')

    map_arg = LaunchConfiguration('map_yaml').perform(context).strip()
    map_yaml_path = Path(map_arg).expanduser().resolve(strict=True) if map_arg else None
    # Validate map references before creating a new recording directory.
    if map_yaml_path:
        with map_yaml_path.open('r', encoding='utf-8') as source:
            map_config = yaml.safe_load(source)
        if not isinstance(map_config, dict) or not isinstance(map_config.get('image'), str):
            raise ValueError('map_yaml must contain an image path')
        image_path = Path(map_config['image']).expanduser()
        if not image_path.is_absolute():
            image_path = map_yaml_path.parent / image_path
        image_path.resolve(strict=True)

    session_dir.mkdir(parents=True)
    manifest = {
        'schema_version': 1,
        'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
        'storage': 'sqlite3',
        'bag_path': 'bag',
        'topics_requested': topics,
        'map_snapshot': _copy_map(map_yaml_path, session_dir) if map_yaml_path else None,
        'notes': 'A requested topic is recorded only while a matching publisher exists.',
    }
    with (session_dir / 'session.yaml').open('w', encoding='utf-8') as target:
        yaml.safe_dump(manifest, target, sort_keys=False, allow_unicode=True)

    return [
        LogInfo(msg=f'Recording Ackermann data in {session_dir}'),
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record', '--storage', 'sqlite3',
                '--output', str(session_dir / 'bag'),
                '--include-hidden-topics', *topics,
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory('ackermann_data_logger')
    return LaunchDescription([
        DeclareLaunchArgument(
            'topics_file',
            default_value=os.path.join(package_share, 'config', 'topics.yaml'),
            description='YAML file listing ROS topics to record',
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='/home/rosdev/ros2_ws/src/ackermann_data_logger/recordings',
            description='Directory for recording sessions; override on the real robot',
        ),
        DeclareLaunchArgument(
            'session_name',
            default_value='',
            description='Optional unique session name; generated from UTC time by default',
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value='',
            description='Optional saved map YAML to copy with its image into the session',
        ),
        OpaqueFunction(function=_start_recording),
    ])
