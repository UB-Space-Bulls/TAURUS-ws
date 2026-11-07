"""Brings up the real rover's drivetrain: robot model, ros2_control's
controller manager (with the ODESC CAN plugin), the steer/wheel controllers,
and the swerve kinematics node. The real-hardware counterpart to
rover_description/launch/gazebo.launch.py -- same controllers, same swerve
node, different hardware underneath.

    # real ODESCs (bring the CAN link up first:
    #   sudo ip link set can0 up type can bitrate 1000000)
    ros2 launch rover_bringup rover.launch.py

    # against scripts/fake_odesc.py on a virtual CAN bus
    ros2 launch rover_bringup rover.launch.py can_interface:=vcan0

    # no CAN at all -- ros2_control's built-in fake hardware
    ros2 launch rover_bringup rover.launch.py hardware:=mock
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare("rover_description")
    hardware = LaunchConfiguration("hardware")
    can_interface = LaunchConfiguration("can_interface")

    xacro_file = PathJoinSubstitution([description_share, "urdf", "rover.urdf.xacro"])
    controllers_file = PathJoinSubstitution([description_share, "config", "rover_controllers.yaml"])

    # value_type=str for the same reason as in gazebo.launch.py: a URDF string
    # isn't valid YAML, and launch_ros would otherwise try to parse it as such.
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ", xacro_file,
                " hardware:=", hardware,
                " can_interface:=", can_interface,
            ]),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # In Jazzy the controller manager reads the URDF from the
    # /robot_description topic that robot_state_publisher publishes, rather
    # than taking it as a parameter -- hence the remap.
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controllers_file],
        remappings=[("~/robot_description", "/robot_description")],
    )

    # Heads up: by default the controller manager activates the hardware as
    # soon as it starts, i.e. OdescSystem arms all 8 motors right here
    # (holding current steer angles, zero wheel speed) -- before any
    # controller is even loaded.

    # One spawner loading all three in order, so they don't hit the
    # controller manager concurrently (same concern as gazebo.launch.py).
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "steer_controller",
            "wheel_controller",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    swerve_kinematics_node = Node(
        package="rover_swerve_controller",
        executable="swerve_kinematics_node",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "hardware", default_value="odesc",
            description="odesc (real CAN bus) or mock (ros2_control fake hardware)"),
        DeclareLaunchArgument(
            "can_interface", default_value="can0",
            description="SocketCAN interface the ODESCs are on (vcan0 for fake_odesc.py)"),
        robot_state_publisher,
        controller_manager,
        controller_spawner,
        swerve_kinematics_node,
    ])
