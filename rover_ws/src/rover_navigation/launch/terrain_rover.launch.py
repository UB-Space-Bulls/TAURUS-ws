"""Drive the simulated rover on the STL tile with live terrain-aware Nav2."""
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    navigation = Path(get_package_share_directory('rover_navigation'))
    simulation = Path(get_package_share_directory('lidar_terrain_description'))
    rover = Path(get_package_share_directory('rover_description'))
    view = LaunchConfiguration('rviz_view').perform(context)
    rviz_file = 'terrain_inspection.rviz' if view == 'terrain' else 'terrain_rover.rviz'
    template = (simulation / 'worlds/terrain.sdf.in').read_text()
    # Reuse the supplied tile's visual/collision surface and world plugins;
    # remove the stationary scanner. Only the rover-mounted sensors run here.
    mesh_uri = (simulation / 'meshes/terrain_tile.stl').as_uri()
    root = ET.fromstring(template.replace('@MESH@', mesh_uri)
                         .replace('@SCALE@', '1').replace('@X@', '0')
                         .replace('@Y@', '0').replace('@Z@', '1.2'))
    world = root.find('world')
    world.remove(world.find("model[@name='stationary_scanner']"))
    temporary = tempfile.TemporaryDirectory(prefix='terrain_rover_')
    world_file = Path(temporary.name) / 'terrain_rover.sdf'
    ET.ElementTree(root).write(world_file, encoding='unicode')

    def cleanup(_context):
        temporary.cleanup()
        return []

    def include(share, filename, arguments):
        # Child launch arguments must not overwrite the outer viewer settings.
        return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / filename)),
            launch_arguments=arguments.items())])

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        include(rover, 'gazebo.launch.py', {
            'world': str(world_file), 'terrain_lidar': 'true', 'odometry_dimensions': '3',
            'spawn_z': '0.5', 'gui': LaunchConfiguration('gui'),
            'headless_rendering': LaunchConfiguration('headless_rendering')}),
        # Gazebo ground-truth odometry uses the world coordinates of this scene.
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'terrain_map', '--child-frame-id', 'odom'],
             parameters=[{'use_sim_time': True}]),
        Node(package='terrain_geometry', executable='terrain_node', name='terrain_geometry',
             output='screen', parameters=[LaunchConfiguration('terrain_config')]),
        Node(package='terrain_geometry', executable='terrain_cost_adapter',
             name='terrain_cost_adapter', output='screen', parameters=[
                 LaunchConfiguration('adapter_config'), {'use_sim_time': True}]),
        include(navigation, 'nav2_sim.launch.py', {
            'use_sim_time': 'true', 'rviz': 'false',
            'cmd_vel_output': 'cmd_vel_terrain_input',
            'params_file': LaunchConfiguration('nav2_params')}),
        Node(package='rover_navigation', executable='terrain_velocity_gate.py',
             output='screen'),
        Node(package='lidar_terrain_description', executable='mesh_marker',
             output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[{'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2',
             output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(navigation / 'config' / rviz_file)],
             parameters=[{'use_sim_time': True}]),
    ]


def generate_launch_description():
    navigation = Path(get_package_share_directory('rover_navigation'))
    terrain = Path(get_package_share_directory('terrain_geometry'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz_view', default_value='navigation',
                              choices=['navigation', 'terrain']),
        DeclareLaunchArgument('headless_rendering', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('terrain_config', default_value=str(
            navigation / 'config/terrain_moving.yaml')),
        DeclareLaunchArgument('adapter_config', default_value=str(
            terrain / 'config/nav2_adapter.yaml')),
        DeclareLaunchArgument('nav2_params', default_value=str(
            navigation / 'config/nav2_moving_terrain.yaml')),
        OpaqueFunction(function=setup),
    ])
