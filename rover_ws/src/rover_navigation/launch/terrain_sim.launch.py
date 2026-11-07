from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("rover_navigation")

    nav2_launch = PathJoinSubstitution([
        share, "launch", "nav2_sim.launch.py"
    ])

    terrain_config = PathJoinSubstitution([
        share, "config", "nav2_terrain_params.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "terrain_enabled",
            default_value="true",
            description="Enable synthetic rough terrain"
        ),

        Node(
            package="rover_navigation",
            executable="terrain_cost_grid.py",
            name="a1_terrain_grid",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "terrain_enabled": ParameterValue(
                    LaunchConfiguration("terrain_enabled"),
                    value_type=bool
                )
            }]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "params_file": terrain_config,
                "use_sim_time": "true"
            }.items()
        )
    ])
