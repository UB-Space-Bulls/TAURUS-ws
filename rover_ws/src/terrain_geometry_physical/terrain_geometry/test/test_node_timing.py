"""A cloud can precede its TF callback; keep one exact-stamp retry, with expiry."""
from types import SimpleNamespace

from builtin_interfaces.msg import Time as Stamp
from sensor_msgs.msg import PointCloud2
from rclpy.time import Time
from rclpy.clock import ClockType
from terrain_geometry_impl.node import TerrainNode


def waiting_node():
    node = object.__new__(TerrainNode)
    node.frame, node.tracking_frame = 'odom', 'base_link'
    node.rolling, node.self_filter_radius, node.max_age = True, 0.7, 1.5
    node.latest = PointCloud2()
    node.latest.header.frame_id = 'camera_init'
    node.latest.header.stamp = Stamp(sec=4)
    node.now = 4.1
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(
        nanoseconds=int(node.now*1e9), clock_type=ClockType.ROS_TIME))
    node.get_logger = lambda: SimpleNamespace(warning=lambda *a, **kw: None)
    calls = []
    def can_transform(target, source, stamp):
        calls.append((target, source, stamp.nanoseconds))
        return False
    node.buffer = SimpleNamespace(can_transform=can_transform)
    return node, calls


def test_missing_tf_retries_only_the_original_stamp():
    node, calls = waiting_node()
    message = node.latest
    node.tick()
    assert node.latest is message
    assert calls == [('odom', 'camera_init', 4_000_000_000)]


def test_pending_tf_cannot_keep_a_stale_scan_alive():
    node, calls = waiting_node()
    node.tick()
    node.now = 6.
    node.tick()
    assert node.latest is None
    assert len(calls) == 1


def test_new_scan_replaces_pending_retry_instead_of_accumulating():
    node, calls = waiting_node()
    node.tick()
    new = PointCloud2()
    new.header.frame_id = 'camera_init'
    new.header.stamp = Stamp(sec=4, nanosec=100_000_000)
    node.receive(new)
    node.tick()
    assert node.latest is new
    assert calls[-1][2] == 4_100_000_000
