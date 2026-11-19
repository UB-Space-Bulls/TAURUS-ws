#!/usr/bin/env python3
"""
fake_tag_publisher.py
Broadcasts simulated TF transforms for testing marker navigation without physical hardware.

Broadcasts (with the default tag_family "aruco"):
- map -> odom (mock localization)
- odom -> base_link (mock rover position)
- map -> aruco_3 (an irrelevant/decoy marker at x=2.0, y=1.0)
- map -> aruco_5 (the requested target marker at x=5.0, y=2.0)

Markers are published upright (like a sign on a post), using the axis
convention set by `marker_z_axis` ("out" = OpenCV ArUco, "in" = AprilTag) —
keep it the same as nav2tag's. The tagN_yaw params are the approach heading:
the direction the rover drives to face the marker head-on.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros


# Marker orientation at heading=0 (rover drives along +X to face it), as (x, y, z, w):
#   "in"  — AprilTag: x right = -Y_map, y down = -Z_map, z into face = +X_map
#   "out" — OpenCV ArUco: x right = -Y_map, y up = +Z_map, z out of face = -X_map
_BASE_MARKER_QUATERNION = {
    "in": (-0.5, 0.5, -0.5, 0.5),
    "out": (0.5, -0.5, -0.5, 0.5),
}


def upright_marker_quaternion(heading: float, z_axis: str = "out"):
    """
    (x, y, z, w) quaternion for an upright, wall-mounted marker.

    `heading` is the map-frame direction a rover drives to approach the marker
    head-on. heading=0 means the printed face is visible from the -x side and
    the rover drives along +x. `z_axis` picks the detector's axis convention.
    """
    bx, by, bz, bw = _BASE_MARKER_QUATERNION[z_axis]
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
    marker_z_axis: str = None,
) -> TransformStamped:
    """Flat yaw-only transform, or an upright marker if `marker_z_axis` is given."""
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame

    t.transform.translation.x = float(x)
    t.transform.translation.y = float(y)
    t.transform.translation.z = float(z)

    if marker_z_axis is not None:
        qx, qy, qz, qw = upright_marker_quaternion(yaw, marker_z_axis)
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
        self.declare_parameter("tag_family", "aruco")
        self.declare_parameter("marker_z_axis", "out")

        self.declare_parameter("tag3_x", 2.0)
        self.declare_parameter("tag3_y", 1.0)
        self.declare_parameter("tag3_yaw", 0.0)

        self.declare_parameter("tag5_x", 5.0)
        self.declare_parameter("tag5_y", 2.0)
        self.declare_parameter("tag5_yaw", 0.0)

        rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        family = self.get_parameter("tag_family").get_parameter_value().string_value
        self.marker_z_axis = self.get_parameter("marker_z_axis").get_parameter_value().string_value
        if self.marker_z_axis not in _BASE_MARKER_QUATERNION:
            raise ValueError(f"marker_z_axis must be 'out' or 'in', got '{self.marker_z_axis}'")
        self.tag3_frame = f"{family}_3"
        self.tag5_frame = f"{family}_5"

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / rate, self.broadcast_transforms)

        self.get_logger().info(
            f"FakeTagPublisher active! Broadcasting fake TF transforms at {rate:.1f}Hz "
            f"(marker z-axis '{self.marker_z_axis}'):\n"
            f" - map -> odom\n"
            f" - odom -> base_link\n"
            f" - map -> {self.tag3_frame} (decoy) at ({self.get_parameter('tag3_x').value}, {self.get_parameter('tag3_y').value})\n"
            f" - map -> {self.tag5_frame} (target) at ({self.get_parameter('tag5_x').value}, {self.get_parameter('tag5_y').value})"
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
            # 1. Map to Odom (simulating SLAM / localization)
            create_transform("map", "odom", now, 0.0, 0.0, 0.0, 0.0),
            # 2. Odom to Base Link (simulating wheel odometry)
            create_transform("odom", "base_link", now, 0.0, 0.0, 0.0, 0.0),
            # 3. Decoy marker 3 (should be ignored when looking for 5)
            create_transform("map", self.tag3_frame, now, tag3_x, tag3_y, 0.5, tag3_yaw,
                             marker_z_axis=self.marker_z_axis),
            # 4. Target marker 5 (should be targeted when requested)
            create_transform("map", self.tag5_frame, now, tag5_x, tag5_y, 0.5, tag5_yaw,
                             marker_z_axis=self.marker_z_axis),
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
