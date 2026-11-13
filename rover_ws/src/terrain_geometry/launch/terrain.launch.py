"""Physical terrain only: subscribe to an existing cloud; never open a sensor."""
import os
from pathlib import Path
import sys
import yaml

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    share = get_package_share_directory('terrain_geometry')
    sys.path.insert(0, os.path.join(get_package_prefix('terrain_geometry'), 'lib', 'terrain_geometry'))
    from terrain_geometry_impl.hardware import validate_mount
    target, lidar = value('target_frame'), value('lidar_frame')
    if not target or not lidar or target == lidar:
        raise ValueError('Use a distinct gravity-aligned target_frame and actual lidar_frame')
    config = Path(value('config')).expanduser()
    if not config.is_file():
        raise ValueError(f'Terrain configuration does not exist: {config}')
    actions = []
    mount_file = value('mount_file')
    if mount_file:
        path = Path(mount_file).expanduser()
        if not path.is_file():
            raise ValueError(f'Mount file does not exist: {path}. Run ros2 run terrain_geometry create_mount')
        pose = validate_mount(yaml.safe_load(path.read_text()), target, lidar)
        arguments = []
        for name, number in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), pose):
            arguments.extend(['--' + name, str(number)])
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
            name='terrain_measured_mount', arguments=arguments + [
                '--frame-id', target, '--child-frame-id', lidar],
            parameters=[{'use_sim_time': False}]))
    else:
        actions.append(LogInfo(msg=f'No mount_file: expecting an existing valid TF from {lidar} to {target}.'))
    actions.append(Node(package='terrain_geometry', executable='terrain_node',
        name='terrain_geometry', output='screen', parameters=[str(config), {
            'input_topic': value('input_topic'), 'target_frame': target, 'use_sim_time': False}]))
    if value('nav2_adapter') == 'true':
        adapter_config = Path(value('adapter_config')).expanduser()
        if not adapter_config.is_file():
            raise ValueError(f'Adapter configuration does not exist: {adapter_config}')
        actions.append(Node(package='terrain_geometry', executable='terrain_cost_adapter',
            name='terrain_cost_adapter', output='screen', parameters=[str(adapter_config),
                {'use_sim_time': False}]))
    if value('rviz') == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', name='terrain_rviz',
            arguments=['-d', os.path.join(share, 'config', 'terrain.rviz'), '-f', target],
            parameters=[{'use_sim_time': False}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory('terrain_geometry')
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/unilidar/cloud'),
        DeclareLaunchArgument('target_frame', default_value='terrain_map'),
        DeclareLaunchArgument('lidar_frame', default_value='unilidar_lidar'),
        DeclareLaunchArgument('mount_file', default_value='',
            description='Measured static mount YAML; omit only with an existing valid TF chain'),
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'terrain.yaml')),
        DeclareLaunchArgument('nav2_adapter', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('adapter_config', default_value=os.path.join(share, 'config', 'nav2_adapter.yaml')),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
