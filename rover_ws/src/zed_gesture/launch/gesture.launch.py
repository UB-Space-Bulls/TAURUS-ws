from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Set launch_zed:=false when another launch already owns the physical camera.
    return LaunchDescription([
        DeclareLaunchArgument("camera_model", default_value="zed2"),
        DeclareLaunchArgument("camera_name", default_value="zed2"),
        DeclareLaunchArgument("serial_number", default_value="0"),
        DeclareLaunchArgument("image_topic", default_value=[
            "/", LaunchConfiguration("camera_name"), "/zed_node/rgb/color/rect/image"]),
        DeclareLaunchArgument("launch_zed", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("publish_tf", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("device", default_value="0", description="GPU index, or cpu for CPU inference"),
        DeclareLaunchArgument(
            "zed_params_file",
            default_value=PathJoinSubstitution([FindPackageShare("zed_gesture"), "config", "zed_camera.yaml"])),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([FindPackageShare("zed_gesture"), "config", "gesture.yaml"])),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py"])),
            launch_arguments={
                "camera_model": LaunchConfiguration("camera_model"),
                "camera_name": LaunchConfiguration("camera_name"),
                "namespace": "",
                "node_name": "zed_node",
                "serial_number": LaunchConfiguration("serial_number"),
                "ros_params_override_path": LaunchConfiguration("zed_params_file"),
                "publish_tf": LaunchConfiguration("publish_tf"),
                "publish_map_tf": "false",
                "enable_ipc": "false",
                "enable_gnss": "false",
                "use_sim_time": "false",
                "sim_mode": "false",
                "svo_path": "live",
            }.items(),
            condition=IfCondition(LaunchConfiguration("launch_zed"))),

        Node(
            package="zed_gesture",
            executable="zed_gesture_node",
            name="zed_gesture_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file"),
                        {"image_topic": LaunchConfiguration("image_topic"),
                         "device": ParameterValue(LaunchConfiguration("device"), value_type=str),
                         "use_sim_time": False}]),
    ])
