from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # If your bring-up package already starts the ZED wrapper, leave launch_zed:=false
    # and just point image_topic at it.
    return LaunchDescription([
        DeclareLaunchArgument("image_topic", default_value="/zed/zed_node/rgb/image_rect_color"),
        DeclareLaunchArgument("launch_zed", default_value="false"),
        DeclareLaunchArgument("camera_model", default_value="zed2i"),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([FindPackageShare("zed_gesture"), "config", "gesture.yaml"])),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py"])),
            launch_arguments={"camera_model": LaunchConfiguration("camera_model")}.items(),
            condition=IfCondition(LaunchConfiguration("launch_zed"))),

        Node(
            package="zed_gesture",
            executable="zed_gesture_node",
            name="zed_gesture_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file"),
                        {"image_topic": LaunchConfiguration("image_topic")}]),
    ])
