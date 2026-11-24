#!/usr/bin/env python3
"""Stop the moving simulation when terrain validity or velocity input expires."""
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class TerrainVelocityGate(Node):
    def __init__(self):
        super().__init__('terrain_velocity_gate')
        self.valid = False
        self.health_time = self.command_time = float('-inf')
        self.require_localization = self.declare_parameter('require_localization', False).value
        self.localization_valid = False
        self.localization_time = float('-inf')
        self.command = Twist()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Bool, '/terrain/cost_grid_valid', self.health, 1)
        self.create_subscription(Twist, '/cmd_vel_terrain_input', self.receive, 10)
        if self.require_localization:
            self.create_subscription(Bool, '/localization/valid', self.localization, 1)
        # This node intentionally uses wall/steady time to stop even if /clock stops.
        self.create_timer(0.05, self.tick)

    def health(self, message):
        self.valid = message.data
        self.health_time = time.monotonic()

    def receive(self, message):
        self.command = message
        self.command_time = time.monotonic()

    def localization(self, message):
        self.localization_valid = message.data
        self.localization_time = time.monotonic()

    def tick(self):
        now = time.monotonic()
        fresh = (self.valid and now - self.health_time < 1.5
                 and now - self.command_time < 0.5)
        if self.require_localization:
            fresh = fresh and self.localization_valid and now-self.localization_time < 1.5
        self.publisher.publish(self.command if fresh else Twist())


def main():
    rclpy.init()
    node = TerrainVelocityGate()
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
