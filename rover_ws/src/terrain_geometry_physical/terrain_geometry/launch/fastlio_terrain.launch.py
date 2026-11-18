"""Consume existing FAST-LIO output; do not start another simulator or TF tree."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('terrain_geometry')
    def value(name, kind):
        return ParameterValue(LaunchConfiguration(name), value_type=kind)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('body_ground_clearance', description='Measured vertical body/IMU height above ground in metres'),
        DeclareLaunchArgument('level_frame_confirmed', default_value='false'),
        DeclareLaunchArgument('target_frame', default_value='camera_init'),
        DeclareLaunchArgument('tracking_frame', default_value='body'),
        DeclareLaunchArgument('input_topic', default_value='/cloud_registered'),
        DeclareLaunchArgument('nav2_adapter', default_value='false'),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([share, 'config', 'fastlio_terrain.yaml'])),
        Node(package='terrain_geometry', executable='terrain_node', name='terrain_geometry',
             output='screen', parameters=[LaunchConfiguration('config'), {
                 'use_sim_time': value('use_sim_time', bool),
                 'body_ground_clearance': value('body_ground_clearance', float),
                 'level_frame_confirmed': value('level_frame_confirmed', bool),
                 'target_frame': value('target_frame', str),
                 'tracking_frame': value('tracking_frame', str),
                 'input_topic': value('input_topic', str), 'rolling_window': True}]),
        Node(package='terrain_geometry', executable='terrain_cost_adapter', name='terrain_cost_adapter',
             condition=IfCondition(LaunchConfiguration('nav2_adapter')), output='screen',
             parameters=[PathJoinSubstitution([share, 'config', 'nav2_adapter.yaml']), {
                 'use_sim_time': value('use_sim_time', bool)}]),
    ])
