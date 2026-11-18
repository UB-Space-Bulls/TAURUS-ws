"""Stationary Gazebo perception + Nav2 bench; no moving rover or localization."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    navigation = FindPackageShare('rover_navigation')
    simulation = FindPackageShare('lidar_terrain_description')
    terrain = FindPackageShare('terrain_geometry')

    def include(share, filename, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', filename])),
            launch_arguments=arguments.items())

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('headless_rendering', default_value='false'),
        include(simulation, 'gazebo.launch.py', {
            'gui': LaunchConfiguration('gui'), 'rviz': LaunchConfiguration('rviz'),
            'rviz_config': PathJoinSubstitution([navigation, 'config', 'terrain_bench.rviz']),
            'headless_rendering': LaunchConfiguration('headless_rendering')}),
        # These identity transforms describe this known stationary test fixture.
        # Real rover operation must supply localization and moving odometry instead.
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'terrain_map', '--child-frame-id', 'odom'],
             parameters=[{'use_sim_time': True}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link'],
             parameters=[{'use_sim_time': True}]),
        Node(package='rover_navigation', executable='stationary_odometry.py',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='terrain_scan_converter', output='screen',
             remappings=[('cloud_in', '/sim/terrain/cloud'), ('scan', '/scan')],
             parameters=[PathJoinSubstitution([navigation, 'config', 'terrain_scan.yaml'])]),
        Node(package='terrain_geometry', executable='terrain_node',
             name='terrain_geometry', output='screen', parameters=[
                 PathJoinSubstitution([terrain, 'config', 'terrain.yaml']),
                 {'use_sim_time': True, 'input_topic': '/sim/terrain/cloud',
                  'processing_rate': 1.0}]),
        include(terrain, 'nav2_adapter.launch.py', {'use_sim_time': 'true'}),
        include(navigation, 'nav2_sim.launch.py', {
            'use_sim_time': 'true', 'rviz': 'false',
            'params_file': PathJoinSubstitution([
                navigation, 'config', 'nav2_perception_params.yaml'])}),
    ])
