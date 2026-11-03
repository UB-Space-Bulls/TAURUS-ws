#!/usr/bin/env python3
"""
fake_tag_publisher.py
Broadcasts simulated TF transforms for testing AprilTag navigation without physical hardware.

Broadcasts:
- map -> odom (mock localization)
- odom -> base_link (mock rover position)
- map -> tag36h11:3 (an irrelevant/decoy tag at x=2.0, y=1.0)
- map -> tag36h11:5 (the requested target tag at x=5.0, y=2.0)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros


def create_transform(
    parent_frame: str,
    child_frame: str,
    stamp,
    x: float,
    y: float,
    z: float,
    yaw: float,
) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame

    t.transform.translation.x = float(x)
    t.transform.translation.y = float(y)
    t.transform.translation.z = float(z)

    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = math.sin(yaw / 2.0)
    t.transform.rotation.w = math.cos(yaw / 2.0)
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
            create_transform("map", "tag36h11:3", now, tag3_x, tag3_y, 0.0, tag3_yaw),
            # 4. Target Tag 5 (should be targeted when requested)
            create_transform("map", "tag36h11:5", now, tag5_x, tag5_y, 0.0, tag5_yaw),
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
