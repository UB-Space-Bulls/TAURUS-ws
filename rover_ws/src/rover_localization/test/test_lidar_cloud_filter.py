import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField

from rover_localization.lidar_cloud_filter import filter_cloud

# Same layout Gazebo publishes: intensity at 16, ring (uint16) at 24, 32-byte points.
GZ_DTYPE = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring'],
                     'formats': ['<f4', '<f4', '<f4', '<f4', '<u2'],
                     'offsets': [0, 4, 8, 16, 24], 'itemsize': 32})


def gazebo_cloud(points, height=1):
    arr = np.zeros(len(points), dtype=GZ_DTYPE)
    for i, (x, y, z, intensity, ring) in enumerate(points):
        arr[i] = (x, y, z, intensity, ring)
    msg = PointCloud2()
    msg.header.frame_id = 'lidar_link'
    msg.height, msg.width = height, len(points) // height
    msg.fields = [PointField(name=n, offset=o, datatype=d, count=1) for n, o, d in
                  (('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('intensity', 16, 7), ('ring', 24, 4))]
    msg.point_step, msg.row_step = 32, 32 * msg.width
    msg.is_dense = False
    msg.data = arr.tobytes()
    return msg


def decode(msg):
    dtype = np.dtype({'names': [f.name for f in msg.fields],
                      'formats': ['<u2' if f.datatype == 4 else '<f4' for f in msg.fields],
                      'offsets': [f.offset for f in msg.fields], 'itemsize': msg.point_step})
    return np.frombuffer(bytes(msg.data), dtype=dtype)


def test_drops_non_finite_and_keeps_values():
    inf, nan = float('inf'), float('nan')
    msg = gazebo_cloud([(1, 2, 3, 0.5, 4), (inf, inf, inf, 0, 5), (nan, 0, 0, 0, 6),
                        (-1, 0, inf, 0, 7), (4, 5, 6, 1.5, 31), (0, 0, 0, 0, 0)], height=2)
    out = filter_cloud(msg)
    pts = decode(out)
    assert out.is_dense and out.height == 1 and out.width == 3
    assert out.header.frame_id == 'lidar_link'
    assert pts['x'].tolist() == [1, 4, 0]
    assert pts['ring'].tolist() == [4, 31, 0]
    assert pts['intensity'].tolist() == [0.5, 1.5, 0]
    assert pts['time'].tolist() == [0, 0, 0]
    assert [f.name for f in out.fields] == ['x', 'y', 'z', 'intensity', 'ring', 'time']


def test_rejects_cloud_without_ring():
    msg = gazebo_cloud([(1, 2, 3, 0, 0)])
    msg.fields = msg.fields[:4]
    with pytest.raises(ValueError, match='ring'):
        filter_cloud(msg)
