#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid


class TerrainGridPublisher(Node):
    def __init__(self):
        super().__init__('a1_terrain_grid')
        self.declare_parameter('terrain_enabled', False)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.publisher = self.create_publisher(
            OccupancyGrid, '/terrain_cost_grid', qos
        )

        self.previous_mode = None
        self.create_timer(2.0, self.publish_grid)
        self.publish_grid()

    def publish_grid(self):
        enabled = self.get_parameter('terrain_enabled').value

        msg = OccupancyGrid()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = 0.2
        msg.info.width = 150
        msg.info.height = 150
        msg.info.origin.position.x = -10.0
        msg.info.origin.position.y = -15.0
        msg.info.origin.orientation.w = 1.0

        data = [0] * (150 * 150)

        if enabled:
            for iy in range(65, 85):
                for ix in range(95, 105):
                    data[iy * 150 + ix] = 80

        msg.data = data
        self.publisher.publish(msg)

        if enabled != self.previous_mode:
            mode = 'ROUGH TERRAIN' if enabled else 'FLAT BASELINE'
            self.get_logger().info(
                f'{mode}: 150x150 grid, resolution 0.2 m'
            )
            self.previous_mode = enabled


def main():
    rclpy.init()
    node = TerrainGridPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
