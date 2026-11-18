#!/usr/bin/env python3
"""Zero-motion odometry for the stationary terrain bench, never a rover estimator."""
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node('stationary_bench_odometry')
    publisher = node.create_publisher(Odometry, '/odom', 10)

    def publish():
        message = Odometry()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        message.pose.pose.orientation.w = 1.0
        publisher.publish(message)

    node.create_timer(0.1, publish)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
