"""Make Gazebo's simulated LiDAR cloud safe for FAST-LIO and Point-LIO.

Gazebo's gpu_lidar marks rays that hit nothing as +/-inf (about 40% of each
720 x 32 scan here). Both estimators only reject points closer than `blind`,
so inf points would pass straight into their maps. This node drops every
non-finite point and adds a per-point `time` field of 0 s.

time = 0 is the truth for this simulator, not a placeholder: Gazebo renders
the whole scan at one instant, so there is no motion distortion to undo. A
real spinning LiDAR (the Unitree L1/L2) supplies its own per-point time and
must NOT go through this node.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

_OUT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
                       ('ring', '<u2'), ('time', '<f4')], align=False)
_OUT_FIELDS = [PointField(name=name, offset=_OUT_DTYPE.fields[name][1],
                          datatype=datatype, count=1)
               for name, datatype in (('x', PointField.FLOAT32), ('y', PointField.FLOAT32),
                                      ('z', PointField.FLOAT32),
                                      ('intensity', PointField.FLOAT32),
                                      ('ring', PointField.UINT16), ('time', PointField.FLOAT32))]
_IN_TYPES = {PointField.FLOAT32: '<f4', PointField.UINT16: '<u2'}


def filter_cloud(msg: PointCloud2) -> PointCloud2:
    """Return a dense copy of `msg` with only finite points plus time = 0."""
    if msg.is_bigendian:
        raise ValueError('big-endian clouds are not supported')
    fields = {f.name: f for f in msg.fields}
    missing = {'x', 'y', 'z', 'intensity', 'ring'} - fields.keys()
    if missing:
        raise ValueError(f'cloud is missing fields {sorted(missing)}')
    in_dtype = np.dtype({
        'names': ['x', 'y', 'z', 'intensity', 'ring'],
        'formats': [_IN_TYPES[fields[n].datatype] for n in ('x', 'y', 'z', 'intensity', 'ring')],
        'offsets': [fields[n].offset for n in ('x', 'y', 'z', 'intensity', 'ring')],
        'itemsize': msg.point_step,
    })
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.row_step)
    points = rows[:, :msg.width * msg.point_step].copy().view(in_dtype).reshape(-1)
    keep = np.isfinite(points['x']) & np.isfinite(points['y']) & np.isfinite(points['z'])
    kept = points[keep]

    out = np.zeros(kept.size, dtype=_OUT_DTYPE)
    for name in ('x', 'y', 'z', 'intensity', 'ring'):
        out[name] = kept[name]

    result = PointCloud2()
    result.header = msg.header
    result.height, result.width = 1, out.size
    result.fields = _OUT_FIELDS
    result.is_bigendian = False
    result.point_step = _OUT_DTYPE.itemsize
    result.row_step = result.point_step * out.size
    result.is_dense = True
    result.data = out.tobytes()
    return result


class LidarCloudFilter(Node):
    def __init__(self):
        super().__init__('lidar_cloud_filter')
        self.pub = self.create_publisher(PointCloud2, 'output', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, 'input', self.callback, qos_profile_sensor_data)

    def callback(self, msg):
        try:
            self.pub.publish(filter_cloud(msg))
        except ValueError as exc:
            self.get_logger().error(f'Dropping cloud: {exc}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = LidarCloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
