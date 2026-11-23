"""
Start the YOLO detector, optionally with a fake camera.

  # real ZED already running:
  ros2 launch yolo_perception yolo.launch.py profile:=hammer
  # no ZED, replay a folder of photos instead:
  ros2 launch yolo_perception yolo.launch.py profile:=keyboard mock:=true
      mock_source:=~/kb_photos mock_fps:=2.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def as_str(name):
    # Without value_type, launch YAML-parses values: 'device:=0' would arrive as int 0
    # and crash a string parameter.
    return ParameterValue(LaunchConfiguration(name), value_type=str)


def generate_launch_description():
    args = [
        DeclareLaunchArgument('profile', default_value='terrain',
                              description='terrain | hammer | keyboard | coco '
                                          '(config/models.yaml)'),
        DeclareLaunchArgument('image_topic',
                              default_value='/zed2/zed_node/rgb/color/rect/image'),
        DeclareLaunchArgument('weights_dir', default_value='',
                              description="'' = $YOLO_WEIGHTS_DIR or ~/yolo_weights"),
        DeclareLaunchArgument('model_path', default_value='',
                              description='override the profile weights'),
        DeclareLaunchArgument('device', default_value='', description="'' auto, 'cpu', '0'"),
        DeclareLaunchArgument('topic_prefix', default_value='/yolo'),
        DeclareLaunchArgument('mock', default_value='false',
                              description='start mock_zed_node too'),
        DeclareLaunchArgument('mock_source', default_value='0',
                              description='webcam index, video, image or image folder'),
        DeclareLaunchArgument('mock_fps', default_value='15.0'),
    ]

    mock_camera = Node(
        package='yolo_perception', executable='mock_zed_node', name='mock_zed_node',
        condition=IfCondition(LaunchConfiguration('mock')),
        parameters=[{
            'source': as_str('mock_source'),
            'topic': as_str('image_topic'),
            'fps': ParameterValue(LaunchConfiguration('mock_fps'), value_type=float),
        }])

    detector = Node(
        package='yolo_perception', executable='zed_yolo_node', name='zed_yolo_node',
        output='screen',
        parameters=[{
            'profile': as_str('profile'),
            'image_topic': as_str('image_topic'),
            'weights_dir': as_str('weights_dir'),
            'model_path': as_str('model_path'),
            'device': as_str('device'),
            'topic_prefix': as_str('topic_prefix'),
        }])

    return LaunchDescription(args + [mock_camera, detector])
