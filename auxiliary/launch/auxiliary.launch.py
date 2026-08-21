import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory('auxiliary')
    config_path = LaunchConfiguration('config').perform(context)
    if not os.path.isabs(config_path):
        config_path = os.path.join(package_share, 'config', config_path)

    with open(config_path, 'r') as config_file:
        config = yaml.safe_load(config_file)

    launch_config = config.get('launch', {})
    namespace = launch_config.get('namespace', 'ego_racecar')
    actions = []

    if launch_config.get('start_recorder', False):
        recorder_config = config.get(
            'waypoint_recorder_node', {}).get('ros__parameters', {})
        output_directory = recorder_config.get('output_directory', '')
        if not output_directory:
            raise RuntimeError(
                f'output_directory is empty. Set it in {config_path}.')
        if not os.path.isabs(output_directory):
            output_directory = os.path.join(
                os.path.dirname(config_path), output_directory)
        if not os.path.isdir(output_directory):
            raise RuntimeError(
                f'Waypoint output directory does not exist: {output_directory}')
        recorder_config['output_directory'] = output_directory

        actions.append(Node(
            package='auxiliary',
            executable='waypoint_recorder_sim',
            name='waypoint_recorder_node',
            namespace=namespace,
            output='screen',
            emulate_tty=True,
            parameters=[recorder_config],
        ))

    if launch_config.get('start_displayer', True):
        displayer_config = config.get(
            'waypoint_displayer_node', {}).get('ros__parameters', {})
        waypoint_file = displayer_config.get('waypoint_file_path', '')
        if not waypoint_file:
            raise RuntimeError(
                f'waypoint_file_path is empty. Set it in {config_path}.')
        if not os.path.isabs(waypoint_file):
            waypoint_file = os.path.join(os.path.dirname(config_path), waypoint_file)
        if not os.path.isfile(waypoint_file):
            raise RuntimeError(f'Waypoint file does not exist: {waypoint_file}')
        displayer_config['waypoint_file_path'] = waypoint_file

        actions.append(Node(
            package='auxiliary',
            executable='waypoints_displayer_sim',
            name='waypoint_displayer_node',
            namespace=namespace,
            output='screen',
            parameters=[displayer_config],
        ))

    if not actions:
        raise RuntimeError('Both start_recorder and start_displayer are false.')

    return actions


def generate_launch_description():
    package_share = get_package_share_directory('auxiliary')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join(package_share, 'config', 'auxiliary.yaml'),
            description='Auxiliary config: an absolute path or a file in config/.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
