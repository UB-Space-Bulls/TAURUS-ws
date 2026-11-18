"""Offline map fixture, separate from FAST-LIO and live navigation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', description='Absolute patch.yaml created by evaluate_patch.py'),
        Node(package='terrain_geometry', executable='site_patch_publisher',
             name='mdrs_patch_publisher', output='screen',
             parameters=[LaunchConfiguration('config'), {'use_sim_time': False}]),
        Node(package='terrain_geometry', executable='terrain_node', name='terrain_geometry',
             output='screen', parameters=[LaunchConfiguration('config'), {'use_sim_time': False}]),
    ])
