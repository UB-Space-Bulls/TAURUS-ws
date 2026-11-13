"""One simulation, upstream FAST-LIO launch, and independent diagnostics."""
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    def share(package):
        try:
            return Path(get_package_share_directory(package))
        except LookupError as exc:
            raise RuntimeError(f'Missing package {package}; build and source the workspace') from exc

    local, description, fast = map(share, ('rover_localization', 'rover_description', 'fast_lio'))
    for package in ('ros_gz_sim', 'ros_gz_bridge', 'controller_manager',
                    'rover_swerve_controller', 'robot_state_publisher'):
        share(package)
    if LaunchConfiguration('rviz').perform(context).lower() in ('true', '1'):
        share('rviz2')
    world = description / 'worlds' / LaunchConfiguration('world').perform(context)
    config = local / 'config' / 'fast_lio_gazebo.yaml'
    for path in (world, config, fast / 'launch' / 'mapping.launch.py'):
        if not path.is_file():
            raise RuntimeError(f'Required launch asset is missing: {path}')
    params = yaml.safe_load(config.read_text())['/**']['ros__parameters']
    if (params['common']['lid_topic'] != '/lidar/points'
            or params['common']['imu_topic'] != '/imu'
            or params['preprocess']['lidar_type'] != 5
            or params.get('use_sim_time') is not True):
        raise RuntimeError('FAST-LIO Gazebo configuration has incompatible topics/type/clock')
    # Source inspection establishes FAST-LIO body == IMU. Verify this repository's
    # fixed mounting rather than inventing a TF edge between disconnected trees.
    root = ET.parse(description / 'urdf' / 'rover.urdf.xacro').getroot()
    joint = root.find("joint[@name='lidar_joint']")
    imu = root.find("gazebo[@reference='base_link']/sensor[@type='imu']")
    mapping = params['mapping']
    aligned = (
        joint is not None and joint.attrib.get('type') == 'fixed'
        and joint.find('parent').attrib['link'] == 'base_link'
        and joint.find('child').attrib['link'] == 'lidar_link'
        and [float(v) for v in joint.find('origin').attrib['xyz'].split()] == [0, 0, 0.25]
        and [float(v) for v in joint.find('origin').attrib['rpy'].split()] == [0, 0, 0]
        and imu is not None and imu.find('pose') is None
        and imu.findtext('gz_frame_id') == 'base_link'
        and mapping['extrinsic_est_en'] is False
        and mapping['extrinsic_T'] == [0, 0, 0.25]
        and mapping['extrinsic_R'] == [1, 0, 0, 0, 1, 0, 0, 0, 1])
    return [
        LogInfo(msg=f'Body alignment verified: {aligned}. Generic LiDAR timing is '
                    'unvalidated. Use separate teleop terminal; see rover_localization/README.md.'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(description / 'launch' / 'gazebo.launch.py')),
            launch_arguments={'world': str(world)}.items()),
        GroupAction(actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(fast / 'launch' / 'mapping.launch.py')),
            launch_arguments={'config_path': str(config.parent), 'config_file': config.name,
                              'use_sim_time': 'true', 'rviz': 'false'}.items())], scoped=True),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', str(local / 'rviz' / 'fast_lio_test.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='rover_localization', executable='odometry_comparator',
             output='screen', emulate_tty=True,
             parameters=[{'use_sim_time': True, 'body_alignment_verified': aligned}],
             condition=IfCondition(LaunchConfiguration('comparator'))),
        Node(package='rover_localization', executable='cmd_vel_watchdog',
             output='screen', parameters=[{'use_sim_time': True}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='second_world.sdf'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('comparator', default_value='true'),
        OpaqueFunction(function=start),
    ])
