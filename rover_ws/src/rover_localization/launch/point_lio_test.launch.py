"""Point-LIO on the Gazebo rover, mirroring fast_lio_test.launch.py.

sim:=true (default) starts Gazebo as well. sim:=false starts only the cloud
filter, Point-LIO, and diagnostics, for replaying a recorded bag
(`ros2 bag play <bag> --clock`) so Point-LIO and FAST-LIO see identical data.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    local = Path(get_package_share_directory('rover_localization'))
    description = Path(get_package_share_directory('rover_description'))
    config = local / 'config' / 'point_lio_gazebo.yaml'
    params = yaml.safe_load(config.read_text())['/**']['ros__parameters']
    if (params['common']['lid_topic'] != '/lidar/points_filtered'
            or params['common']['imu_topic'] != '/imu'
            or params['odom_child_frame_id'] != 'body'
            or params.get('use_sim_time') is not True):
        raise RuntimeError('Point-LIO Gazebo configuration has incompatible topics/frames/clock')

    # Point-LIO's state is the IMU pose, and the sim IMU sits exactly on
    # base_link. Check the URDF still says so before trusting the comparison.
    # The xacro file is plain URDF XML apart from $(find), so ET can read it.
    root = ET.parse(description / 'urdf' / 'rover.urdf.xacro').getroot()
    joint = root.find("joint[@name='lidar_joint']")
    imu = root.find("gazebo[@reference='base_link']/sensor[@type='imu']")
    mapping = params['mapping']
    aligned = (
        joint is not None and joint.attrib.get('type') == 'fixed'
        and joint.find('parent').attrib['link'] == 'base_link'
        and [float(v) for v in joint.find('origin').attrib['xyz'].split()] == mapping['extrinsic_T']
        and [float(v) for v in joint.find('origin').attrib['rpy'].split()] == [0, 0, 0]
        and imu is not None and imu.find('pose') is None
        and imu.findtext('gz_frame_id') == 'base_link'
        and mapping['extrinsic_est_en'] is False
        and mapping['extrinsic_R'] == [1, 0, 0, 0, 1, 0, 0, 0, 1])

    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    actions = [LogInfo(msg=f'Point-LIO body alignment verified: {aligned}')]
    if sim:
        actions += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(description / 'launch' / 'gazebo.launch.py')),
                launch_arguments={'world': LaunchConfiguration('world').perform(context),
                                  'headless': LaunchConfiguration('headless').perform(context)
                                  }.items()),
            Node(package='rover_localization', executable='cmd_vel_watchdog',
                 output='screen', parameters=[{'use_sim_time': True}]),
        ]
    actions += [
        Node(package='rover_localization', executable='lidar_cloud_filter', output='screen',
             parameters=[{'use_sim_time': True}],
             remappings=[('input', '/lidar/points'), ('output', '/lidar/points_filtered')]),
        Node(package='point_lio', executable='pointlio_mapping', name='laserMapping',
             output='screen', parameters=[str(config)],
             # Same odometry topic FAST-LIO uses, so both feed one comparator.
             remappings=[('/aft_mapped_to_init', '/Odometry')]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', str(local / 'rviz' / 'fast_lio_test.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='rover_localization', executable='odometry_comparator',
             output='screen', emulate_tty=True,
             parameters=[{'use_sim_time': True, 'body_alignment_verified': aligned,
                          'estimator_name': 'Point-LIO'}],
             condition=IfCondition(LaunchConfiguration('comparator'))),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='second_world.sdf'),
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('comparator', default_value='true'),
        OpaqueFunction(function=start),
    ])
