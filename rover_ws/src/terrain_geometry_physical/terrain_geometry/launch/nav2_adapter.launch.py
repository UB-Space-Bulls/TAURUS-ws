"""Start only the TerrainGrid -> OccupancyGrid adapter; no driver or navigation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('terrain_geometry'), 'config', 'nav2_adapter.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='terrain_geometry', executable='terrain_cost_adapter',
             name='terrain_cost_adapter', output='screen', parameters=[LaunchConfiguration('config'), {
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
