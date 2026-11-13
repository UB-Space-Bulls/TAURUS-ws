"""Exercise ROS node callbacks with synthetic simulation-stamped messages."""
import math
import time

import pytest
import rclpy
from nav_msgs.msg import Odometry

from rover_localization.odometry_comparator import OdometryComparator
from rover_localization.cmd_vel_watchdog import Watchdog


def message(index, stamp, x=0.0, y=0.0, angle=0.0):
    msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = (
        ('odom', 'base_link') if index == 0 else ('camera_init', 'body'))
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.z = math.sin(angle/2)
    msg.pose.pose.orientation.w = math.cos(angle/2)
    return msg


@pytest.fixture
def node():
    rclpy.init()
    node = OdometryComparator()
    node.aligned = True
    node.imu_frame = 'base_link'
    yield node
    node.destroy_node()
    rclpy.shutdown()


def pair(node, stamp, x):
    node.receive(0, message(0, stamp-0.01, 10, 20+x, math.pi/2))
    node.receive(0, message(0, stamp+0.01, 10, 20+x, math.pi/2))
    node.receive(1, message(1, stamp, -5+x, 3))


def test_synchronized_reference_and_motion(node):
    pair(node, 1, 0)
    pair(node, 1.1, 0.1)
    assert node.references[0].stamp == node.references[1].stamp == 1
    from rover_localization.pose_math import relative, errors
    g, e, _ = node.last_pair
    assert errors(relative(node.references[0], g), relative(node.references[1], e))[1] == pytest.approx(0, abs=1e-10)


def test_missing_input_and_alignment(node, capsys):
    node.display()
    assert 'Comparison unavailable' in capsys.readouterr().out
    node.aligned = False
    pair(node, 1, 0)
    assert node.references is None


def test_restart_and_bad_frame_clear_reference(node):
    pair(node, 2, 0)
    assert node.references is not None
    node.receive(1, message(1, 1, 0, 0))
    assert node.references is None
    pair(node, 3, 0)
    bad = message(0, 4)
    bad.child_frame_id = 'unknown'
    node.receive(0, bad)
    assert node.last_pair is None


def test_watchdog_zero_after_silence():
    rclpy.init()
    node = Watchdog()
    published = []
    class Publisher:
        def publish(self, msg):
            published.append(msg)
    node.publisher = Publisher()
    try:
        node.last_command = time.monotonic()-1
        node.tick()
        assert len(published) == 1
        assert published[0].linear.x == published[0].angular.z == 0
        node.last_command = time.monotonic()
        node.tick()
        assert len(published) == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
