"""Physical L1 only: serial reader, raw cloud/IMU, diagnostics and optional RViz."""
import os
from pathlib import Path
import stat

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    port = str(Path(value('port')).expanduser())
    try:
        mode = os.stat(port).st_mode
    except FileNotFoundError:
        raise RuntimeError(
            f'L1 serial device does not exist: {port}. Check lsusb and '
            '/dev/serial/by-id/, then pass port:=/dev/your_device. '
            'Motor rotation alone does not confirm a USB data connection.') from None
    if not stat.S_ISCHR(mode):
        raise RuntimeError(f'L1 port must be a serial character device: {port}')
    if not os.access(port, os.R_OK | os.W_OK):
        raise RuntimeError(f'No read/write access to {port}; check serial group/ACL permissions.')
    scans = int(value('cloud_scan_num'))
    if not 1 <= scans <= 180:
        raise ValueError('cloud_scan_num must be in [1, 180]')
    actions = [Node(package='unitree_l1_driver', executable='l1_ros.py',
                    output='screen', parameters=[{'port': port,
                    'cloud_scan_num': scans, 'use_sim_time': False}])]
    if value('publish_imu_tf') == 'true':
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
            name='l1_imu_static_tf', arguments=[
                '--x', '-0.007698', '--y', '-0.014655', '--z', '0.00667',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'unilidar_lidar', '--child-frame-id', 'unilidar_imu'],
            parameters=[{'use_sim_time': False}]))
    if value('rviz') == 'true':
        # Display reference only; it is not a world pose or gravity estimate.
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
            name='l1_imu_visual_reference', arguments=[
                '--frame-id', 'unilidar_lidar', '--child-frame-id', 'imu_orientation_reference'],
            parameters=[{'use_sim_time': False}]))
        actions.append(Node(package='rviz2', executable='rviz2', name='l1_rviz',
            arguments=['-d', os.path.join(get_package_share_directory('unitree_l1_driver'),
                                         'rviz', 'l1.rviz')],
            parameters=[{'use_sim_time': False}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('cloud_scan_num', default_value='18'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('publish_imu_tf', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
