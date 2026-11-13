"""Launch the physical ZED 2 for standalone acceptance or rover integration.

Upstream interface reference: zed-ros2-wrapper v5.2.0.
The camera name and node name are fixed to match the checked-in RViz preset.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('rover_bringup')
    config_path = PathJoinSubstitution([package_share, 'config', 'zed2.yaml'])
    rviz_path = PathJoinSubstitution([package_share, 'rviz', 'zed2.rviz'])

    arguments = [
        DeclareLaunchArgument(
            'serial_number', default_value='0',
            description='ZED serial number; 0 selects the first available camera.',
        ),
        DeclareLaunchArgument(
            'params_file', default_value=config_path,
            description='DEIMOS overrides layered after upstream camera defaults.',
        ),
        DeclareLaunchArgument(
            'publish_tf', default_value='true', choices=['true', 'false'],
            description=(
                'Publish zed2_odom -> zed2_camera_link for standalone testing. '
                'Disable when the rover localization system owns camera motion.'
            ),
        ),
        DeclareLaunchArgument(
            'publish_urdf', default_value='true', choices=['true', 'false'],
            description=(
                'Publish camera internal transforms from the upstream model. '
                'Disable only when another publisher supplies that subtree.'
            ),
        ),
        DeclareLaunchArgument(
            'publish_imu_tf', default_value='true', choices=['true', 'false'],
            description=(
                'Publish the SDK camera-to-IMU transform. Upstream ignores this '
                'when publish_tf is false; integration must supply the IMU transform.'
            ),
        ),
        DeclareLaunchArgument(
            'rviz', default_value='false', choices=['true', 'false'],
            description='Open RViz on this computer; requires a graphical session.',
        ),
    ]

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py',
        ])),
        launch_arguments={
            'camera_model': 'zed2',
            'camera_name': 'zed2',
            # A nonempty upstream namespace changes the node name in v5.2.0.
            # Leave it empty to obtain /zed2/zed_node, matching our RViz topics.
            'namespace': '',
            'node_name': 'zed_node',
            'serial_number': LaunchConfiguration('serial_number'),
            'ros_params_override_path': LaunchConfiguration('params_file'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'publish_map_tf': 'false',
            'publish_urdf': LaunchConfiguration('publish_urdf'),
            'publish_imu_tf': LaunchConfiguration('publish_imu_tf'),
            'enable_ipc': 'false',
            'enable_gnss': 'false',
            'use_sim_time': 'false',
            'sim_mode': 'false',
            'svo_path': 'live',
        }.items(),
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='zed2_rviz',
        arguments=['-d', rviz_path],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription(arguments + [camera, rviz])
