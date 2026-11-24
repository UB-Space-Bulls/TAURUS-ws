"""FAST-LIO -> terrain -> Nav2 -> moving rover around a physical Gazebo box."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav = Path(get_package_share_directory('rover_navigation'))
    rover = Path(get_package_share_directory('rover_description'))
    terrain = Path(get_package_share_directory('terrain_geometry'))
    fast = Path(get_package_share_directory('fast_lio'))

    def include(share, filename, arguments):
        return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / filename)),
            launch_arguments=arguments.items())])

    def static(parent, child, z=0.0):
        return Node(package='tf2_ros', executable='static_transform_publisher',
                    name=f'{parent}_to_{child}', output='screen', arguments=[
                        '--frame-id', parent, '--child-frame-id', child, '--z', str(z)],
                    parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('headless_rendering', default_value='false', choices=['true', 'false']),
        include(rover, 'gazebo.launch.py', {
            'world': str(nav / 'worlds/obstacle_course.sdf'),
            'terrain_lidar': 'true', 'terrain_lidar_height': '2.0', 'imu': 'true',
            'terrain_lidar_horizontal_samples': '360', 'terrain_lidar_vertical_samples': '128',
            'estimate_odometry': 'true', 'odometry_dimensions': '3', 'spawn_z': '0.32',
            'gui': LaunchConfiguration('gui'),
            'headless_rendering': LaunchConfiguration('headless_rendering')}),
        # Known level start. The sole moving pose authority is FAST-LIO.
        static('terrain_map', 'odom', 0.2921),
        static('odom', 'camera_init'),
        static('body', 'base_link'),
        Node(package='rover_navigation', executable='fastlio_sim_bridge.py',
             parameters=[{'use_sim_time': True}], output='screen'),
        include(fast, 'mapping.launch.py', {
            'config_path': str(nav / 'config'), 'config_file': 'fastlio_sim.yaml',
            'use_sim_time': 'true', 'rviz': 'false'}),
        Node(package='terrain_geometry', executable='terrain_node', name='terrain_geometry',
             parameters=[str(nav / 'config/terrain_fastlio_sim.yaml')], output='screen'),
        Node(package='terrain_geometry', executable='terrain_cost_adapter',
             name='terrain_cost_adapter', parameters=[str(terrain / 'config/nav2_adapter.yaml'),
                                                      {'use_sim_time': True}], output='screen'),
        include(nav, 'nav2_sim.launch.py', {
            'use_sim_time': 'true', 'rviz': 'false', 'cmd_vel_output': 'cmd_vel_terrain_input',
            # LiDAR-rate estimated TF cannot supply a transform at wall "now".
            # Use the scanner's fixed body mounting; retain collision checking.
            'base_shift_correction': 'false',
            'rotate_to_heading': 'true',
            'params_file': str(nav / 'config/nav2_moving_terrain.yaml')}),
        Node(package='rover_navigation', executable='terrain_velocity_gate.py',
             parameters=[{'require_localization': True}], output='screen'),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[{'use_sim_time': True}], arguments=['-d', str(nav / 'config/terrain_rover.rviz')],
             output='screen'),
    ])
