"""Publish real terrain into the DEIMOS nav2 branch's existing OccupancyGrid interface."""
from copy import deepcopy
from dataclasses import fields
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from terrain_geometry.msg import TerrainGrid
from .nav2_adapter import CostPolicy, SnapshotLease, occupancy_values, validate_geometry


class TerrainCostAdapter(Node):
    def __init__(self):
        super().__init__('terrain_cost_adapter')
        defaults = CostPolicy()
        for f in fields(defaults):
            self.declare_parameter(f.name, getattr(defaults, f.name))
        self.policy = CostPolicy(**{f.name: self.get_parameter(f.name).value for f in fields(defaults)})
        for name, value in [('input_topic', '/terrain/grid'), ('output_topic', '/terrain_cost_grid'),
                            ('max_age', 2.0), ('future_tolerance', 0.1), ('publish_rate', 2.0)]:
            self.declare_parameter(name, value)
        self.lease = SnapshotLease(float(self.get_parameter('max_age').value),
                                   float(self.get_parameter('future_tolerance').value))
        rate = float(self.get_parameter('publish_rate').value)
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('publish_rate must be finite and positive')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output = self.create_publisher(OccupancyGrid, self.get_parameter('output_topic').value, qos)
        self.health = self.create_publisher(Bool, '/terrain/cost_grid_valid', 1)
        self.subscription = self.create_subscription(TerrainGrid, self.get_parameter('input_topic').value,
            self.receive, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.grid = None
        self.was_fresh = None
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / rate, self.tick, clock=self.steady_clock)
        self.get_logger().info('Waiting for TerrainGrid; publishing /terrain_cost_grid for Nav2. '
                               'Stop the synthetic a1_terrain_grid publisher.')

    def now_ros(self):
        return self.get_clock().now().nanoseconds / 1e9

    def receive(self, msg):
        try:
            validate_geometry(msg.info, msg.header.frame_id)
            values = occupancy_values(msg.status, msg.ground_height, msg.slope, msg.roughness,
                msg.ground_count, msg.obstacle_count, msg.info.width * msg.info.height, self.policy)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if not self.lease.accept(stamp, self.now_ros(), time.monotonic()):
                self.get_logger().warning('Rejected stale, future or repeated terrain snapshot.',
                                          throttle_duration_sec=5.0)
                return
            grid = OccupancyGrid()
            grid.header = deepcopy(msg.header)
            grid.info = deepcopy(msg.info)
            grid.data = values
            self.grid = grid
            self.tick()
        except (ValueError, TypeError, OverflowError) as exc:
            self.get_logger().warning(f'Invalid terrain snapshot: {exc}', throttle_duration_sec=5.0)

    def tick(self):
        fresh = self.grid is not None and self.lease.fresh(self.now_ros(), time.monotonic())
        self.health.publish(Bool(data=bool(fresh)))
        if self.grid is not None:
            output = self.grid
            if not fresh:
                output = deepcopy(self.grid)
                output.data = [-1] * (output.info.width * output.info.height)
            # Never replace the source stamp with the republish time.
            self.output.publish(output)
        if fresh != self.was_fresh:
            if fresh:
                self.get_logger().info('Publishing costs from fresh terrain observations.')
            else:
                self.get_logger().warning('No fresh terrain: output is unknown (or absent before first grid).')
            self.was_fresh = fresh


def main():
    rclpy.init()
    node = TerrainCostAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
