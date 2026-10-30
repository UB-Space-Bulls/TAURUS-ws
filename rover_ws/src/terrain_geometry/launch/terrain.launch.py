"""Processor for an existing cloud. The caller must supply a valid level TF."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('terrain_geometry')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([share, 'config', 'terrain.yaml'])),
        DeclareLaunchArgument('input_topic', default_value='/unilidar/cloud'),
        DeclareLaunchArgument('target_frame', default_value='terrain_map'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='terrain_geometry', executable='terrain_node', name='terrain_geometry',
             output='screen', parameters=[LaunchConfiguration('config'), {
                 'input_topic': LaunchConfiguration('input_topic'),
                 'target_frame': LaunchConfiguration('target_frame'),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution([share, 'config', 'terrain.rviz']),
                        '-f', LaunchConfiguration('target_frame')],
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
