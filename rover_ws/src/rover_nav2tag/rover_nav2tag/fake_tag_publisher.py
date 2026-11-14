#!/usr/bin/env python3
"""
fake_tag_publisher.py
Broadcasts simulated TF transforms for testing AprilTag navigation without physical hardware.

Broadcasts:
- map -> odom (mock localization)
- odom -> base_link (mock rover position)
- map -> tag36h11:3 (an irrelevant/decoy tag at x=2.0, y=1.0)
- map -> tag36h11:5 (the requested target tag at x=5.0, y=2.0)

Tags are published upright (like a sign on a post), using the AprilTag
frame convention. The tagN_yaw params are the approach heading: the
direction the rover drives to face the tag head-on.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros


def upright_tag_quaternion(heading: float):
    """
    (x, y, z, w) quaternion for an upright, wall-mounted AprilTag, using the
    AprilTag frame convention (x right along the face, y down, z INTO the tag).

    `heading` is the map-frame direction the tag's z-axis points, i.e. the
    direction a rover drives to approach the tag head-on. heading=0 means the
    printed face is visible from the -x side and the rover drives along +x.
    """
    # Base orientation at heading=0: tag x=-Y_map, y=-Z_map, z=+X_map
    bx, by, bz, bw = -0.5, 0.5, -0.5, 0.5
    # Pre-multiply by a rotation of `heading` about map Z
    hz, hw = math.sin(heading / 2.0), math.cos(heading / 2.0)
    return (
        hw * bx - hz * by,
        hw * by + hz * bx,
        hw * bz + hz * bw,
        hw * bw - hz * bz,
    )


def create_transform(
    parent_frame: str,
    child_frame: str,
    stamp,
    x: float,
    y: float,
    z: float,
    yaw: float,
    upright_tag: bool = False,
) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame

    t.transform.translation.x = float(x)
    t.transform.translation.y = float(y)
    t.transform.translation.z = float(z)

    if upright_tag:
        qx, qy, qz, qw = upright_tag_quaternion(yaw)
    else:
        qx, qy, qz, qw = 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    t.transform.rotation.x = qx
    t.transform.rotation.y = qy
    t.transform.rotation.z = qz
    t.transform.rotation.w = qw
    return t


class FakeTagPublisher(Node):
    def __init__(self):
        super().__init__("fake_tag_publisher")

        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("tag3_x", 2.0)
        self.declare_parameter("tag3_y", 1.0)
        self.declare_parameter("tag3_yaw", 0.0)

        self.declare_parameter("tag5_x", 5.0)
        self.declare_parameter("tag5_y", 2.0)
        self.declare_parameter("tag5_yaw", 0.0)

        rate = self.get_parameter("publish_rate").get_parameter_value().double_value

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / rate, self.broadcast_transforms)

        self.get_logger().info(
            f"FakeTagPublisher active! Broadcasting fake TF transforms at {rate:.1f}Hz:\n"
            f" - map -> odom\n"
            f" - odom -> base_link\n"
            f" - map -> tag36h11:3 (decoy) at ({self.get_parameter('tag3_x').value}, {self.get_parameter('tag3_y').value})\n"
            f" - map -> tag36h11:5 (target) at ({self.get_parameter('tag5_x').value}, {self.get_parameter('tag5_y').value})"
        )

    def broadcast_transforms(self):
        now = self.get_clock().now().to_msg()

        tag3_x = self.get_parameter("tag3_x").value
        tag3_y = self.get_parameter("tag3_y").value
        tag3_yaw = self.get_parameter("tag3_yaw").value

        tag5_x = self.get_parameter("tag5_x").value
        tag5_y = self.get_parameter("tag5_y").value
        tag5_yaw = self.get_parameter("tag5_yaw").value

        transforms = [
            # 1. Map to Odom (simulating slam_toolbox)
            create_transform("map", "odom", now, 0.0, 0.0, 0.0, 0.0),
            # 2. Odom to Base Link (simulating wheel odometry)
            create_transform("odom", "base_link", now, 0.0, 0.0, 0.0, 0.0),
            # 3. Decoy Tag 3 (should be ignored when looking for 5)
            create_transform("map", "tag36h11:3", now, tag3_x, tag3_y, 0.5, tag3_yaw, upright_tag=True),
            # 4. Target Tag 5 (should be targeted when requested)
            create_transform("map", "tag36h11:5", now, tag5_x, tag5_y, 0.5, tag5_yaw, upright_tag=True),
        ]

        self.tf_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = FakeTagPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
