"""Optional official L1 driver + terrain node; caller supplies measured level TF.

Requires a separately built/sourced unitree_lidar_ros2 from unilidar_sdk (L1).
Never run this beside the simulated L1: they intentionally share topic names.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('terrain_geometry')
    return LaunchDescription([
        DeclareLaunchArgument('target_frame', description='Measured gravity-aligned frame; TF must already exist'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('l1_variant', default_value='PM', choices=['PM', 'RM']),
        DeclareLaunchArgument('cloud_scan_num', default_value='18'),
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'terrain.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='unitree_lidar_ros2', executable='unitree_lidar_ros2_node',
             name='unitree_lidar_ros2_node', output='screen', parameters=[{
                 'use_sim_time': False, 'port': LaunchConfiguration('port'),
                 'rotate_yaw_bias': 0.0, 'range_scale': 0.001, 'range_bias': 0.0,
                 'range_min': 0.05,
                 'range_max': ParameterValue(PythonExpression([
                     "30.0 if '", LaunchConfiguration('l1_variant'), "' == 'RM' else 20.0"]), value_type=float),
                 'cloud_frame': 'unilidar_lidar', 'cloud_topic': 'unilidar/cloud',
                 'cloud_scan_num': ParameterValue(LaunchConfiguration('cloud_scan_num'), value_type=int),
                 'imu_frame': 'unilidar_imu', 'imu_topic': 'unilidar/imu',
             }]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'terrain.launch.py')),
            launch_arguments={'input_topic': '/unilidar/cloud', 'use_sim_time': 'false',
                              'target_frame': LaunchConfiguration('target_frame'),
                              'config': LaunchConfiguration('config'),
                              'rviz': LaunchConfiguration('rviz')}.items()),
    ])
