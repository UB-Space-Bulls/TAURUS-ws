import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    target_tag_id = LaunchConfiguration("target_tag_id")
    stop_distance = LaunchConfiguration("stop_distance")
    tag_family = LaunchConfiguration("tag_family")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_fake_tags = LaunchConfiguration("launch_fake_tags")

    declare_target_tag_id = DeclareLaunchArgument(
        "target_tag_id",
        default_value="5",
        description="Target AprilTag numeric ID (-1 for any tag)",
    )

    declare_stop_distance = DeclareLaunchArgument(
        "stop_distance",
        default_value="0.5",
        description="Standoff distance in front of tag in meters",
    )

    declare_tag_family = DeclareLaunchArgument(
        "tag_family",
        default_value="tag36h11",
        description="AprilTag family name / frame prefix",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_launch_fake_tags = DeclareLaunchArgument(
        "launch_fake_tags",
        default_value="false",
        description="Launch fake tag publisher for standalone testing",
    )

    nav2tag_node = Node(
        package="rover_nav2tag",
        executable="nav2tag_node",
        name="navigate_to_tag",
        output="screen",
        parameters=[
            {
                "target_tag_id": target_tag_id,
                "stop_distance": stop_distance,
                "tag_family": tag_family,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    fake_tags_node = Node(
        package="rover_nav2tag",
        executable="fake_tag_publisher",
        name="fake_tag_publisher",
        output="screen",
        condition=IfCondition(launch_fake_tags),
    )

    return LaunchDescription(
        [
            declare_target_tag_id,
            declare_stop_distance,
            declare_tag_family,
            declare_use_sim_time,
            declare_launch_fake_tags,
            nav2tag_node,
            fake_tags_node,
        ]
    )
