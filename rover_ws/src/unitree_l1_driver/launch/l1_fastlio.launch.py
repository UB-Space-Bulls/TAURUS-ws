"""Physical L1 + FAST-LIO, with SDK times and timed clouds; no rover controls."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    driver = Path(get_package_share_directory('unitree_l1_driver'))
    fast = Path(get_package_share_directory('fast_lio'))
    config = Path(LaunchConfiguration('config').perform(context)).expanduser().resolve()
    if not config.is_file():
        raise RuntimeError(f'FAST-LIO config does not exist: {config}')

    def include(path, arguments):
        return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(path)), launch_arguments=arguments.items())])

    return [
        include(driver / 'launch/l1.launch.py', {
            'port': LaunchConfiguration('port'), 'cloud_scan_num': '18',
            'profile': 'fastlio', 'rviz': 'false', 'publish_imu_tf': 'false'}),
        include(fast / 'launch/mapping.launch.py', {
            'config_path': str(config.parent), 'config_file': config.name,
            'use_sim_time': 'false', 'rviz': LaunchConfiguration('rviz')}),
    ]


def generate_launch_description():
    driver = Path(get_package_share_directory('unitree_l1_driver'))
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('config', default_value=str(driver / 'config/fast_lio_l1.yaml')),
        OpaqueFunction(function=setup),
    ])
