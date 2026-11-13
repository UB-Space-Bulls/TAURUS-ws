"""Test-session watchdog: periodically send zero after /cmd_vel goes quiet.

Uses monotonic wall time so pausing Gazebo cannot disable the stop. This
observes the existing command topic without changing the controller package.
"""
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node


class Watchdog(Node):
    def __init__(self):
        super().__init__('fast_lio_cmd_vel_watchdog')
        self.last_command = time.monotonic()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.receive, 10)
        self.timer = self.create_timer(
            0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Command watchdog: zero velocity after 0.75 s silence; '
                               'teleop must use repeat_rate:=10.')

    def receive(self, msg):
        # Ignore our own zero messages; repeated zero commands need no timeout.
        if any((msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z)):
            self.last_command = time.monotonic()

    def tick(self):
        if time.monotonic() - self.last_command > 0.75:
            self.publisher.publish(Twist())


def main(args=None):
    # Let Python raise KeyboardInterrupt; close the ROS context exactly once.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Watchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
