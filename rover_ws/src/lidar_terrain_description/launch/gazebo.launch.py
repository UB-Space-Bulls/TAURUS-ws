from pathlib import Path
import math
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value=lambda name: LaunchConfiguration(name).perform(context)
    share=Path(get_package_share_directory('lidar_terrain_description'))
    numbers={name:float(value(name)) for name in ('scale','sensor_x','sensor_y','sensor_z')}
    if not all(math.isfinite(v) for v in numbers.values()) or numbers['scale']<=0:
        raise ValueError('Finite sensor coordinates and positive scale required')
    mesh=share/'meshes'/f'terrain_{value("mesh")}.stl'
    template=(share/'worlds/terrain.sdf.in').read_text()
    for key,val in {'MESH':mesh.as_uri(),'SCALE':numbers['scale'],'X':numbers['sensor_x'],
                    'Y':numbers['sensor_y'],'Z':numbers['sensor_z']}.items():
        template=template.replace('@'+key+'@',str(val))
    tmp=tempfile.TemporaryDirectory(prefix='lidar_terrain_')
    world=Path(tmp.name)/'terrain.sdf'; world.write_text(template)
    def cleanup(_context):
        tmp.cleanup(); return []
    cmd=['gz','sim','-r',str(world)]
    if value('gui')=='false': cmd.insert(2,'-s')
    if value('headless_rendering')=='true': cmd.append('--headless-rendering')
    actions=[RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        ExecuteProcess(cmd=cmd,output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{
            'config_file':str(share/'config/bridge.yaml'),'use_sim_time':True}],output='screen'),
        Node(package='tf2_ros',executable='static_transform_publisher',arguments=[
            '--x',str(numbers['sensor_x']),'--y',str(numbers['sensor_y']),'--z',str(numbers['sensor_z']),
            '--roll','0','--pitch','0','--yaw','0','--frame-id','terrain_map','--child-frame-id','terrain_sim_lidar'],
            parameters=[{'use_sim_time':True}])]
    if value('rviz')=='true':
        actions.extend([Node(package='lidar_terrain_description',executable='mesh_marker',parameters=[{
            'mesh':value('mesh'),'scale':numbers['scale'],'use_sim_time':True}]),
            Node(package='rviz2',executable='rviz2',arguments=['-d',value('rviz_config') or str(share/'config/terrain.rviz')],parameters=[{'use_sim_time':True}])])
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mesh',default_value='tile',choices=['tile','overview']),
        DeclareLaunchArgument('scale',default_value='1.0'),
        DeclareLaunchArgument('sensor_x',default_value='0.0'),DeclareLaunchArgument('sensor_y',default_value='0.0'),
        DeclareLaunchArgument('sensor_z',default_value='1.2'),
        DeclareLaunchArgument('gui',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('headless_rendering',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('rviz_config',default_value=''),
        DeclareLaunchArgument('rviz',default_value='true',choices=['true','false']),OpaqueFunction(function=setup)])
