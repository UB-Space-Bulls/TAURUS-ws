"""ROS 2 adapter: transform at the input stamp, process, publish one snapshot."""

from dataclasses import fields
import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from geometry_msgs.msg import Point
from nav_msgs.msg import MapMetaData, OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker
from terrain_geometry.msg import TerrainGrid

from .core import Config, preview_costs, process, transform_xyz
from .hardware import finite_xyz
from .rolling import rolling_config


class TerrainNode(Node):
    def __init__(self):
        super().__init__('terrain_geometry')
        defaults = Config()
        for field in fields(defaults):
            self.declare_parameter(field.name, getattr(defaults, field.name))
        self.cfg = Config(**{f.name: self.get_parameter(f.name).value for f in fields(defaults)})
        for name, value in {
            'input_topic': '/unilidar/cloud', 'target_frame': 'terrain_map',
            'processing_rate': 1.0, 'max_scan_age': 1.5,
            'preview_slope_deg': 20.0, 'preview_roughness': 0.06,
            'rolling_window': False, 'tracking_frame': 'body',
            'body_ground_clearance': 0.0, 'level_frame_confirmed': False,
            'self_filter_radius': 0.0,
        }.items():
            self.declare_parameter(name, value)
        self.frame = self.get_parameter('target_frame').value
        rate = float(self.get_parameter('processing_rate').value)
        self.max_age = float(self.get_parameter('max_scan_age').value)
        self.preview_slope = float(self.get_parameter('preview_slope_deg').value)
        self.preview_roughness = float(self.get_parameter('preview_roughness').value)
        if not self.frame or not math.isfinite(rate) or rate <= 0:
            raise ValueError('target_frame and a finite positive processing_rate are required')
        if not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError('max_scan_age must be finite and positive')
        if not (0 < self.preview_slope < 90 and self.preview_roughness > 0):
            raise ValueError('invalid preview thresholds')
        self.rolling = self.get_parameter('rolling_window').value
        self.tracking_frame = self.get_parameter('tracking_frame').value
        self.clearance = float(self.get_parameter('body_ground_clearance').value)
        self.self_filter_radius = float(self.get_parameter('self_filter_radius').value)
        if not math.isfinite(self.self_filter_radius) or self.self_filter_radius < 0:
            raise ValueError('self_filter_radius must be finite and nonnegative')
        if self.rolling:
            if not self.get_parameter('level_frame_confirmed').value:
                raise ValueError('Confirm a gravity-aligned target_frame before enabling rolling_window')
            if not self.tracking_frame or self.tracking_frame == self.frame:
                raise ValueError('tracking_frame must be a distinct moving body/IMU frame')
            rolling_config(self.cfg, (0., 0., 0.), self.clearance)
        self.buffer = Buffer(cache_time=Duration(seconds=10))
        self.listener = TransformListener(self.buffer, self)
        input_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST)
        output_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.last_received = time.perf_counter()
        self.latest = None
        self.subscription = self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value, self.receive, input_qos)
        self.grid_pub = self.create_publisher(TerrainGrid, '/terrain/grid', output_qos)
        self.cloud_pubs = {name: self.create_publisher(
            PointCloud2, f'/terrain/{name}_points', output_qos)
            for name in ('ground', 'obstacle', 'unclassified', 'grid')}
        self.unknown_pub = self.create_publisher(Marker, '/terrain/unknown_cells', output_qos)
        self.preview_pub = self.create_publisher(OccupancyGrid, '/terrain/costmap_preview', output_qos)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'Using level frame {self.frame}; rolling={self.rolling}; one scan, no accumulation.')

    def receive(self, msg):
        self.last_received = time.perf_counter()
        self.latest = msg  # Bound latency by keeping the newest cloud only.

    def tick(self):
        if self.latest is None:
            if time.perf_counter() - self.last_received > 5.0:
                self.get_logger().warning(
                    "No input cloud received for 5 seconds; check the separate LiDAR driver and input_topic.",
                    throttle_duration_sec=5.0)
            return
        msg, self.latest = self.latest, None
        start = time.perf_counter()
        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if stamp.nanoseconds == 0:
            self.get_logger().warning('Zero-stamped cloud dropped; exact acquisition time is required.', throttle_duration_sec=5.0)
            return
        if age > self.max_age or age < -0.05:
            self.get_logger().warning('Dropping stale/future cloud; check driver timestamps and use_sim_time.', throttle_duration_sec=5.0)
            return
        if not msg.header.frame_id:
            self.get_logger().warning('Cloud has no frame_id; dropped.', throttle_duration_sec=5.0)
            return
        required_frames = []
        if msg.header.frame_id != self.frame:
            required_frames.append(msg.header.frame_id)
        if self.rolling or self.self_filter_radius > 0:
            required_frames.append(self.tracking_frame)
        if any(not self.buffer.can_transform(self.frame, source, stamp)
               for source in required_frames):
            # TF and cloud callbacks can arrive in either order. Retry only this
            # newest scan, at its original stamp; receive() may replace it. The
            # age check above still bounds how long a missing TF can defer it.
            self.latest = msg
            return
        try:
            xyz = point_cloud2.read_points_numpy(
                msg, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3)
            xyz = finite_xyz(xyz)
            if msg.header.frame_id != self.frame:
                tf = self.buffer.lookup_transform(self.frame, msg.header.frame_id, stamp)
                t, q = tf.transform.translation, tf.transform.rotation
                xyz = transform_xyz(xyz, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
            cfg = self.cfg
            if self.rolling or self.self_filter_radius > 0:
                body_tf = self.buffer.lookup_transform(self.frame, self.tracking_frame, stamp)
                body = body_tf.transform.translation
                if self.self_filter_radius > 0:
                    # Remove returns inside the rover's horizontal body envelope.
                    # This discards points; it never labels missing ground free.
                    outside = np.hypot(xyz[:, 0] - body.x, xyz[:, 1] - body.y)
                    xyz = xyz[outside > self.self_filter_radius]
            if self.rolling:
                cfg = rolling_config(self.cfg, (body.x, body.y, body.z), self.clearance)
            result = process(xyz, cfg)
        except (TransformException, ValueError, AssertionError, KeyError) as exc:
            self.get_logger().warning(f'Cloud dropped: {exc}', throttle_duration_sec=5.0)
            return
        # Preserve the input header stamp; the driver defines acquisition/receipt semantics.
        header = Header(stamp=msg.header.stamp, frame_id=self.frame)
        info = MapMetaData()
        info.map_load_time = msg.header.stamp
        info.resolution = float(cfg.resolution)
        info.width, info.height = cfg.width, cfg.height
        info.origin.position.x, info.origin.position.y = cfg.origin_x, cfg.origin_y
        info.origin.orientation.w = 1.0
        grid = TerrainGrid(header=header, info=info)
        for name, values in (
            ('status', result.status), ('ground_height', result.height),
            ('slope', result.slope), ('roughness', result.roughness),
            ('ground_count', result.ground_count), ('obstacle_count', result.obstacle_count),
        ):
            setattr(grid, name, values.ravel(order='C').tolist())
        self.grid_pub.publish(grid)
        for name, label in (('ground', 1), ('obstacle', 2), ('unclassified', 0)):
            cloud = point_cloud2.create_cloud_xyz32(
                header, result.points[result.labels == label].astype(np.float32))
            self.cloud_pubs[name].publish(cloud)
        rows, cols = np.nonzero(result.status & 1)
        grid_xyz = np.column_stack((
            cfg.origin_x + (cols + 0.5) * cfg.resolution,
            cfg.origin_y + (rows + 0.5) * cfg.resolution,
            result.height[rows, cols], np.degrees(result.slope[rows, cols]),
            result.roughness[rows, cols])).astype(np.float32)
        cloud_fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z', 'slope_deg', 'roughness_m'))]
        self.cloud_pubs['grid'].publish(point_cloud2.create_cloud(header, cloud_fields, grid_xyz))
        unknown = Marker(header=header, ns='unknown', id=0, type=Marker.CUBE_LIST, action=Marker.ADD)
        unknown.pose.orientation.w = 1.0
        unknown.scale.x = unknown.scale.y = cfg.resolution * 0.95
        unknown.scale.z = 0.012
        unknown.color.r = unknown.color.g = unknown.color.b = 0.45
        unknown.color.a = 0.28
        unknown.lifetime = Duration(seconds=self.max_age).to_msg()
        rows, cols = np.nonzero((result.status & 1) == 0)
        unknown.points = [Point(x=float(cfg.origin_x + (x+0.5)*cfg.resolution),
                                y=float(cfg.origin_y + (y+0.5)*cfg.resolution),
                                z=float(cfg.seed_height - 0.025))
                          for y, x in zip(rows, cols)]
        self.unknown_pub.publish(unknown)
        preview = OccupancyGrid(header=header, info=info)
        preview.data = preview_costs(result, self.preview_slope, self.preview_roughness).ravel().tolist()
        self.preview_pub.publish(preview)
        elapsed = time.perf_counter() - start
        if elapsed > 1.0 / float(self.get_parameter('processing_rate').value):
            self.get_logger().warning(
                f'Processing took {elapsed:.3f}s; lower processing_rate or reduce the ROI.',
                throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TerrainNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
