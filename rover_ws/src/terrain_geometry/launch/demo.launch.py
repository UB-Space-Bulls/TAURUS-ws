"""Isolated P3 test stand: fixed 3D LiDAR, known world orientation, no SLAM."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('terrain_geometry')
    gz_share = get_package_share_directory('ros_gz_sim')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('l1_variant', default_value='PM', choices=['PM', 'RM']),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(share) + os.pathsep
                               + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz_share, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': [os.path.join(share, 'worlds') + '/',
                PythonExpression(["'terrain_demo.sdf' if '", LaunchConfiguration('l1_variant'),
                                  "' == 'PM' else 'terrain_demo_l1_rm.sdf'"]), ' -r']}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml')}], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0', '--y', '0', '--z', '1.2',
                        '--roll', '3.141592653589793', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'terrain_map', '--child-frame-id', 'unilidar_lidar'],
             parameters=[{'use_sim_time': True}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '-0.007698', '--y', '-0.014655', '--z', '0.00667',
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'unilidar_lidar', '--child-frame-id', 'unilidar_imu'],
             parameters=[{'use_sim_time': True}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'terrain.launch.py')),
            launch_arguments={'rviz': LaunchConfiguration('rviz'), 'use_sim_time': 'true'}.items()),
    ])
