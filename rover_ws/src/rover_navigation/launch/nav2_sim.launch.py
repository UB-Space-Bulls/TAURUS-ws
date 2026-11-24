from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def server_with_option(context, package, name, argument, parameter, remappings):
    overrides = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    selected = LaunchConfiguration(argument).perform(context)
    # Ordinary launches retain the value in the user's chosen parameter file.
    if selected != 'configured':
        overrides[parameter] = selected == 'true'
    return [Node(package=package, executable=name, name=name, output='screen',
                 parameters=[LaunchConfiguration('params_file'), overrides],
                 remappings=remappings)]


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock',
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically activate Nav2 lifecycle nodes',
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [
                FindPackageShare('rover_navigation'),
                'config',
                'nav2_params.yaml',
            ]
        ),
        description='Nav2 parameter file',
    )

    controller_server = OpaqueFunction(function=server_with_option, kwargs={
        'package': 'nav2_controller', 'name': 'controller_server',
        'argument': 'rotate_to_heading', 'parameter': 'FollowPath.use_rotate_to_heading',
        'remappings': [('cmd_vel', 'cmd_vel_nav')]})

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    collision_monitor = OpaqueFunction(function=server_with_option, kwargs={
        'package': 'nav2_collision_monitor', 'name': 'collision_monitor',
        'argument': 'base_shift_correction', 'parameter': 'base_shift_correction',
        'remappings': [('cmd_vel', LaunchConfiguration('cmd_vel_output'))]})

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'velocity_smoother',
                    'collision_monitor',
                    'bt_navigator',
                ]
            },
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution(
                [
                    FindPackageShare('rover_navigation'),
                    'rviz',
                    'nav2.rviz',
                ]
            ),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_autostart,
            declare_params_file,
            DeclareLaunchArgument('rviz', default_value='true'),
            DeclareLaunchArgument('cmd_vel_output', default_value='cmd_vel'),
            DeclareLaunchArgument('base_shift_correction', default_value='configured', choices=['configured', 'true', 'false']),
            DeclareLaunchArgument('rotate_to_heading', default_value='configured', choices=['configured', 'true', 'false']),
            controller_server,
            planner_server,
            behavior_server,
            velocity_smoother,
            collision_monitor,
            bt_navigator,
            lifecycle_manager,
            rviz,
        ]
    )
