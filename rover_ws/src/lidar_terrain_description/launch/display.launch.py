from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mesh',default_value='tile',choices=['tile','overview']),
        DeclareLaunchArgument('scale',default_value='1.0'),
        Node(package='tf2_ros',executable='static_transform_publisher',output='screen',arguments=[
            '--x','0','--y','0','--z','0','--roll','0','--pitch','0','--yaw','0',
            '--frame-id','world','--child-frame-id','terrain_map']),
        Node(package='lidar_terrain_description',executable='mesh_marker',output='screen',parameters=[{
            'mesh':LaunchConfiguration('mesh'),'scale':ParameterValue(LaunchConfiguration('scale'),value_type=float)}]),
        Node(package='rviz2',executable='rviz2',arguments=['-d',PathJoinSubstitution([
            FindPackageShare('lidar_terrain_description'),'config','terrain.rviz'])]),
    ])
