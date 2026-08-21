import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory('motion_planning')
    config_path = LaunchConfiguration('config').perform(context)
    if not os.path.isabs(config_path):
        config_path = os.path.join(package_share, 'config', config_path)

    with open(config_path, 'r') as config_file:
        config = yaml.safe_load(config_file)

    launch_config = config.get('launch', {})
    node_config = config.get('rrt_node', {}).get('ros__parameters', {})
    namespace = launch_config.get('namespace', 'ego_racecar')

    waypoint_file = node_config.get('waypoint_file_path', '')
    if not waypoint_file:
        raise RuntimeError(
            f'waypoint_file_path is empty. Set it in {config_path}.'
        )
    if not os.path.isabs(waypoint_file):
        waypoint_file = os.path.join(os.path.dirname(config_path), waypoint_file)
    if not os.path.isfile(waypoint_file):
        raise RuntimeError(f'Waypoint file does not exist: {waypoint_file}')
    node_config['waypoint_file_path'] = waypoint_file

    return [Node(
        package='motion_planning',
        executable='rrt_node_sim',
        name='rrt_node',
        namespace=namespace,
        output='screen',
        parameters=[node_config],
    )]


def generate_launch_description():
    package_share = get_package_share_directory('motion_planning')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join(package_share, 'config', 'rrt.yaml'),
            description='Motion planning config: an absolute path or a file in config/.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
