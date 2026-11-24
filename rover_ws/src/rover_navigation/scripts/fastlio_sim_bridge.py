#!/usr/bin/env python3
"""Level-start sensor gate and FAST-LIO odometry; never consumes ground truth.

FAST-LIO owns camera_init->body. Launch supplies verified identity links
odom->camera_init and body->base_link; this node does not broadcast TF.
"""
import copy
import math
import time

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def without_rover(message):
    """The elevated sim scanner sees its own rover; discard those body returns.

    XY sensor and body origins coincide in this model. Retain sensor field
    layout and original snapshot stamp for all remaining points.
    """
    points = point_cloud2.read_points(message, skip_nans=False).reshape(-1)
    keep = (np.isfinite(points['x']) & np.isfinite(points['y']) & np.isfinite(points['z'])
            & (np.hypot(points['x'], points['y']) > 0.7))
    return point_cloud2.create_cloud(message.header, message.fields, points[keep],
                                    point_step=message.point_step)


class FastlioSimBridge(Node):
    def __init__(self):
        super().__init__('fastlio_sim_bridge')
        self.ready = False
        self.stable_since = None
        self.previous = None
        self.last = {}
        self.fault = False
        self.imu_pub = self.create_publisher(Imu, '/fastlio/imu', qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, '/fastlio/points', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.valid_pub = self.create_publisher(Bool, '/localization/valid', 1)
        self.create_subscription(Imu, '/rover/imu', self.imu, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/rover/lidar/points', self.cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/Odometry', self.odometry, 10)
        self.create_timer(0.1, self.health, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Waiting for 1 s of stationary, level IMU data before FAST-LIO initialization.')

    def current(self, stamp):
        return stamp > 0 and -0.05 <= self.get_clock().now().nanoseconds*1e-9-stamp <= 0.6

    def remember(self, kind, stamp):
        self.last[kind] = (stamp, time.monotonic())

    def imu(self, message):
        stamp = seconds(message.header.stamp)
        if message.header.frame_id != 'base_link' or not self.current(stamp):
            return
        a, w = message.linear_acceleration, message.angular_velocity
        if not all(math.isfinite(v) for v in (a.x, a.y, a.z, w.x, w.y, w.z)):
            return
        if not self.ready:
            stable = (math.hypot(a.x, a.y) < 0.2 and 9.5 < a.z < 10.1
                      and math.sqrt(w.x*w.x+w.y*w.y+w.z*w.z) < 0.05)
            if not stable:
                self.stable_since = None
                return
            if self.stable_since is None or stamp < self.stable_since:
                self.stable_since = stamp
            if stamp-self.stable_since < 1.0:
                return
            self.ready = True
            self.get_logger().info('Level stationary start confirmed; forwarding simulator sensor messages.')
        self.remember('imu', stamp)
        self.imu_pub.publish(message)

    def cloud(self, message):
        stamp = seconds(message.header.stamp)
        if (not self.ready or message.header.frame_id != 'terrain_lidar_link'
                or not self.current(stamp) or message.width*message.height < 10):
            return
        self.remember('cloud', stamp)
        self.cloud_pub.publish(without_rover(message))

    def odometry(self, message):
        stamp = seconds(message.header.stamp)
        p, q = message.pose.pose.position, message.pose.pose.orientation
        if (self.fault or not self.ready or not self.current(stamp)
                or message.header.frame_id != 'camera_init' or message.child_frame_id != 'body'
                or not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > 0.01):
            return
        if self.previous:
            previous_stamp = seconds(self.previous.header.stamp)
            old = self.previous.pose.pose.position
            if stamp < previous_stamp or math.dist((p.x, p.y, p.z), (old.x, old.y, old.z)) > 1.0:
                self.fault = True
                self.get_logger().error('FAST-LIO reset/jump detected; restart this test before driving.')
                self.valid_pub.publish(Bool(data=False))
                return
            if stamp == previous_stamp:
                return
        output = copy.deepcopy(message)
        output.header.frame_id, output.child_frame_id = 'odom', 'base_link'
        # The pinned estimator leaves twist unset. Supply observed planar body
        # velocity for Nav2 directly from successive estimated poses.
        if self.previous:
            dt = stamp-seconds(self.previous.header.stamp)
            old = self.previous.pose.pose.position
            heading = yaw(q)
            vx, vy = (p.x-old.x)/dt, (p.y-old.y)/dt
            output.twist.twist.linear.x = math.cos(heading)*vx+math.sin(heading)*vy
            output.twist.twist.linear.y = -math.sin(heading)*vx+math.cos(heading)*vy
            output.twist.twist.linear.z = (p.z-old.z)/dt
            change = heading-yaw(self.previous.pose.pose.orientation)
            output.twist.twist.angular.z = math.atan2(math.sin(change), math.cos(change))/dt
        self.previous = message
        self.remember('odom', stamp)
        self.odom_pub.publish(output)

    def health(self):
        valid = self.ready and not self.fault and len(self.last) == 3
        valid = valid and all(self.current(stamp) and time.monotonic()-received < 2.0
                              for stamp, received in self.last.values())
        self.valid_pub.publish(Bool(data=bool(valid)))


def main():
    rclpy.init()
    node = FastlioSimBridge()
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
