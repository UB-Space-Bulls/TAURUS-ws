"""Publish a cropped historical map fixture. This is not a LiDAR simulator."""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header


class SitePatchPublisher(Node):
    def __init__(self):
        super().__init__('mdrs_patch_publisher')
        for name, default in [('data', ''), ('center_x', 0.), ('center_y', 0.),
                              ('half_width', 12.), ('frame', 'mdrs_map')]:
            self.declare_parameter(name, default)
        self.frame = self.get_parameter('frame').value
        x = float(self.get_parameter('center_x').value)
        y = float(self.get_parameter('center_y').value)
        half = float(self.get_parameter('half_width').value)
        if not self.frame or not np.isfinite([x,y,half]).all() or not 0 < half <= 50:
            raise ValueError('Invalid patch coordinates, half_width or frame')
        with np.load(self.get_parameter('data').value, allow_pickle=False) as data:
            xyz = data['xyz']
        if xyz.ndim != 2 or xyz.shape[1] != 3:
            raise ValueError('Expected an N-by-3 xyz array')
        valid = (np.isfinite(xyz).all(axis=1) & (abs(xyz[:,0]-x) <= half)
                 & (abs(xyz[:,1]-y) <= half))
        self.xyz = xyz[valid].astype(np.float32)
        if not len(self.xyz): raise ValueError('No data in selected patch')
        self.publisher = self.create_publisher(PointCloud2, '/mdrs/map_patch', 1)
        self.timer = self.create_timer(.5, self.publish)
        self.get_logger().warning('HISTORICAL AIRBORNE MAP FIXTURE: repeated full patch, '
            'synthetic publication stamps, no ray casting, occlusion model or IMU. Do not feed FAST-LIO.')

    def publish(self):
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.frame)
        self.publisher.publish(create_cloud_xyz32(header,self.xyz))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SitePatchPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
