"""Estimator routing, level startup, stale data and reset behavior."""
import math
from pathlib import Path
import runpy
from types import SimpleNamespace
from unittest.mock import patch

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def bridge():
    cls = runpy.run_path(str(Path(__file__).parents[1] / 'scripts/fastlio_sim_bridge.py'))['FastlioSimBridge']
    node = object.__new__(cls)
    node.ready, node.fault = False, False
    node.stable_since = node.previous = None
    node.last = {}
    outputs = {name: [] for name in ('imu', 'odom', 'valid')}
    for name, values in outputs.items():
        setattr(node, name+'_pub', SimpleNamespace(publish=values.append))
    node.now = 2.0
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(node.now*1e9)))
    node.get_logger = lambda: SimpleNamespace(info=lambda *a: None, error=lambda *a: None)
    return node, outputs


def stamp(message, seconds):
    ns = round(seconds*1e9)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, 1_000_000_000)
    return message


def estimate(seconds, x=0., y=0., heading=0.):
    msg = stamp(Odometry(), seconds)
    msg.header.frame_id, msg.child_frame_id = 'camera_init', 'body'
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.z = math.sin(heading/2)
    msg.pose.pose.orientation.w = math.cos(heading/2)
    return msg


def test_estimator_is_not_started_while_tilted_or_moving():
    node, output = bridge()
    msg = Imu()
    msg.header.frame_id = 'base_link'
    msg.linear_acceleration.x, msg.linear_acceleration.z = 1.0, 9.7
    for t in (2., 3., 4.):
        node.now = t
        node.imu(stamp(msg, t))
    assert not node.ready and not output['imu']
    msg.linear_acceleration.x, msg.linear_acceleration.z = 0., 9.81
    for t in (5., 5.5, 6.):
        node.now = t
        node.imu(stamp(msg, t))
    assert node.ready and len(output['imu']) == 1


def test_pose_is_preserved_and_planar_twist_comes_from_estimate():
    node, output = bridge()
    node.ready = True
    node.odometry(estimate(2., heading=math.pi/2))
    node.now = 2.1
    original = estimate(2.1, y=0.02, heading=math.pi/2)
    node.odometry(original)
    result = output['odom'][-1]
    assert result.header.frame_id == 'odom' and result.child_frame_id == 'base_link'
    assert result.pose == original.pose and original.header.frame_id == 'camera_init'
    assert abs(result.twist.twist.linear.x-0.2) < 1e-6
    assert abs(result.twist.twist.linear.y) < 1e-6


def test_stale_invalid_frame_and_nonfinite_estimates_do_not_route():
    node, output = bridge()
    node.ready = True
    node.odometry(estimate(1.))
    invalid = estimate(2.)
    invalid.header.frame_id = 'odom'
    node.odometry(invalid)
    node.odometry(estimate(2., x=float('nan')))
    assert not output['odom']


def test_estimator_reset_latches_invalidity_until_restart():
    node, output = bridge()
    node.ready = True
    node.odometry(estimate(2.))
    node.odometry(estimate(1.9))
    assert node.fault and not output['valid'][-1].data
    node.now = 2.1
    node.odometry(estimate(2.1))
    assert len(output['odom']) == 1


def test_wall_silence_stops_validity_when_sim_clock_is_paused():
    node, output = bridge()
    node.ready = True
    node.last = {key: (2., 10.) for key in ('imu', 'cloud', 'odom')}
    with patch('time.monotonic', return_value=10.1):
        node.health()
    assert output['valid'][-1].data
    with patch('time.monotonic', return_value=12.1):
        node.health()
    assert not output['valid'][-1].data


def test_rover_returns_are_removed_before_the_estimator_without_restamping():
    module = runpy.run_path(str(Path(__file__).parents[1] / 'scripts/fastlio_sim_bridge.py'))
    cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='terrain_lidar_link'),
        [(0., 0., -2.), (0.3, 0.2, -1.9), (2., 0., -2.29), (float('nan'), 0., 0.)])
    cloud.header.stamp.sec = 7
    result = module['without_rover'](cloud)
    xyz = point_cloud2.read_points_numpy(result)
    assert result.header == cloud.header
    assert len(xyz) == 1
    assert tuple(xyz[0][:2]) == (2., 0.) and math.isclose(xyz[0][2], -2.29, abs_tol=1e-6)
