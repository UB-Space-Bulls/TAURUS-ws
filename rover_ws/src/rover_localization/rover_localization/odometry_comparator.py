"""Compare synchronized motion in initially coincident IMU/base_link axes.

No TF is published. The launch verifies the fixed sensor mounting and disables
comparison if it cannot establish body == base_link. Ground truth is linearly
interpolated at FAST-LIO timestamps, with quaternion SLERP for orientation.
"""
from collections import deque
import math
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from std_srvs.srv import Trigger

from .pose_math import Sample, errors, interpolate, normalize, relative


class OdometryComparator(Node):
    def __init__(self):
        super().__init__('odometry_comparator')
        self.aligned = self.declare_parameter('body_alignment_verified', False).value
        self.buffers = [deque(maxlen=1000), deque(maxlen=300)]
        self.arrivals = [None, None]
        self.latest = [None, None]
        self.references = None
        self.last_pair = None
        self.status = 'Waiting for synchronized odometry'
        self.frames = [None, None]
        self.publisher_ids = None
        self.sensor_times = {}
        self.imu_frame = None
        self.last_clock = None
        self.subscriptions_owned = [
            self.create_subscription(Odometry, topic,
                                     lambda msg, i=i: self.receive(i, msg),
                                     qos_profile_sensor_data)
            for i, topic in enumerate(('/odom', '/Odometry'))]
        self.subscriptions_owned += [
            self.create_subscription(Imu, '/imu', self.imu, qos_profile_sensor_data),
            self.create_subscription(PointCloud2, '/lidar/points',
                                     lambda msg: self.sensor_times.update(
                                         cloud=time.monotonic()), qos_profile_sensor_data)]
        self.reset_service = self.create_service(Trigger, '~/reset', self.reset_service_cb)
        self.timer = self.create_timer(
            1.0, self.display, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Comparing synchronized relative motion; '
                               'error sign = FAST-LIO minus Gazebo. '
                               'Generic LiDAR timing is unvalidated.')

    def imu(self, msg):
        if self.imu_frame is not None and self.imu_frame != msg.header.frame_id:
            self.reset('IMU frame changed')
        self.imu_frame = msg.header.frame_id
        self.sensor_times['imu'] = time.monotonic()

    def reset(self, reason):
        for buffer in self.buffers:
            buffer.clear()
        self.references = self.last_pair = None
        self.arrivals = [None, None]
        self.latest = [None, None]
        self.status = reason + '; waiting for new synchronized reference'

    def reset_service_cb(self, request, response):
        self.reset('Manual reset')
        response.success, response.message = True, self.status
        return response

    def receive(self, index, msg):
        now = time.monotonic()
        expected = [('odom', 'base_link'), ('camera_init', 'body')][index]
        frames = (msg.header.frame_id, msg.child_frame_id)
        if frames != expected:
            self.frames[index] = frames
            self.reset(f'Unexpected frames {frames}, expected {expected}')
            return
        self.frames[index] = frames
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            sample = Sample(stamp, (p.x, p.y, p.z), normalize((q.x, q.y, q.z, q.w)))
            if stamp <= 0 or not all(math.isfinite(v) for v in sample.position):
                raise ValueError('Invalid position or timestamp')
        except ValueError as exc:
            self.reset(str(exc))
            return
        buffer = self.buffers[index]
        if self.latest[index] is not None:
            previous = self.latest[index]
            if stamp == previous.stamp:
                return
            gap = now - self.arrivals[index]
            jump = math.dist(previous.position, sample.position)
            # Conservative discontinuity detection; re-reference rather than show
            # an error that may be caused by estimator reset, not rover motion.
            if stamp < previous.stamp or gap > 2.0 or jump > 1.0:
                self.reset('Time regression, input interruption, or pose discontinuity')
        self.latest[index] = sample
        self.buffers[index].append(sample)
        self.arrivals[index] = now
        self.match()

    def match(self):
        if not self.aligned or self.imu_frame != 'base_link':
            self.status = 'Comparison unavailable: body/IMU/base_link alignment unverified'
            return
        ground, estimates = self.buffers
        while ground and estimates:
            estimate = estimates[0]
            if estimate.stamp < ground[0].stamp:
                estimates.popleft()
                continue
            if estimate.stamp > ground[-1].stamp:
                return  # Wait for a bracketing ground-truth sample.
            for a, b in zip(ground, list(ground)[1:]):
                if a.stamp <= estimate.stamp <= b.stamp:
                    estimates.popleft()
                    if b.stamp-a.stamp > 0.1:
                        self.status = 'Comparison unavailable: ground-truth bracket exceeds 0.1 s'
                        self.last_pair = None
                        break
                    truth = interpolate(a, b, estimate.stamp)
                    if self.references is None:
                        self.references = (truth, estimate)
                    self.last_pair = (truth, estimate, b.stamp-a.stamp)
                    self.status = 'Synchronized; displacements in initial base_link axes'
                    break
            else:
                return

    def display(self):
        now = time.monotonic()
        clock = self.get_clock().now().nanoseconds * 1e-9
        if self.last_clock is not None and clock < self.last_clock:
            self.reset('Simulation clock rewound')
        self.last_clock = clock
        ids = tuple(tuple(sorted(bytes(p.endpoint_gid) for p in
                    self.get_publishers_info_by_topic(topic)))
                    for topic in ('/odom', '/Odometry'))
        if self.publisher_ids is not None and ids != self.publisher_ids:
            self.reset('Odometry publisher changed (possible estimator restart)')
        self.publisher_ids = ids
        warnings = []
        if self.get_parameter('use_sim_time').value and clock == 0.0:
            warnings.append('/clock: simulation time has not advanced')
        for i, topic in enumerate(('/odom', '/Odometry')):
            arrival = self.arrivals[i]
            if arrival is None or now-arrival > 2.0:
                warnings.append(f'{topic}: missing/stale (wall time >2 s)')
        for key in ('imu', 'cloud'):
            if now-self.sensor_times.get(key, -math.inf) > 2.0:
                warnings.append(f'{key}: missing/stale')
        if any(len(group) != 1 for group in ids):
            warnings.append('Expected exactly one publisher per odometry topic')
        lines = [f'FAST-LIO diagnostic | sim clock {clock:.3f} s', self.status]
        valid = (not warnings and self.last_pair and self.references
                 and self.aligned and self.imu_frame == 'base_link')
        if valid:
            truth, estimate, width = self.last_pair
            if not -0.1 <= clock-estimate.stamp <= 2.0:
                valid = False
                warnings.append('Matched timestamps are stale or ahead of /clock')
        if valid:
            g = relative(self.references[0], truth)
            e = relative(self.references[1], estimate)
            delta, norm, yaw = errors(g, e)
            lines += ['Coordinate | Gazebo disp (m) | FAST-LIO disp (m) | Error (m)']
            for axis, a, b, d in zip('XYZ', g.position, e.position, delta):
                lines.append(f'{axis:^10} | {a:15.4f} | {b:17.4f} | {d:9.4f}')
            lines += [f'3D error: {norm:.4f} m | yaw error: {yaw:+.3f} deg',
                      f'Matched sim stamp: {estimate.stamp:.6f} s | '
                      f'GT interpolation bracket: {width:.6f} s',
                      f'Reference sim stamp: {self.references[0].stamp:.6f} s']
        else:
            lines.append('Comparison unavailable; no numeric errors displayed')
        lines += warnings
        lines.append('Ground truth is planar; LiDAR per-point timing is unvalidated.')
        prefix = '\033[2J\033[H' if sys.stdout.isatty() else '\n'
        print(prefix + '\n'.join(lines), flush=True)


def main(args=None):
    # Let Python raise KeyboardInterrupt; close the ROS context exactly once.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = OdometryComparator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
