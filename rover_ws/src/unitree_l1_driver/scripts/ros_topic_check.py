#!/usr/bin/env python3
"""Check what ROS subscribers actually receive, including QoS, values and gaps."""
import argparse
import math
import struct
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField, Imu
from l1_common import Health, save_report


class Check(Node):
    def __init__(self, start):
        super().__init__('l1_topic_check')
        self.health = Health(start)
        self.layout_errors = 0
        self.counts = {'cloud': 0, 'imu': 0}
        self.create_subscription(PointCloud2, '/unilidar/cloud', self.cloud, qos_profile_sensor_data)
        self.create_subscription(Imu, '/unilidar/imu', self.imu, qos_profile_sensor_data)

    def cloud(self, m):
        now = time.monotonic()
        if now < self.health.start:
            return
        fields = {f.name: f for f in m.fields}
        try:
            if not all(n in fields and fields[n].datatype == PointField.FLOAT32 and fields[n].count == 1
                       and fields[n].offset+4 <= m.point_step for n in ('x', 'y', 'z', 'intensity', 'time')):
                raise ValueError('Missing or invalid FLOAT32 cloud fields')
            if m.header.frame_id != 'unilidar_lidar' or m.height < 1 or m.point_step < 20:
                raise ValueError('Invalid frame/dimensions')
            if m.row_step < m.width*m.point_step or len(m.data) != m.row_step*m.height:
                raise ValueError('Inconsistent cloud layout')
            invalid = 0
            endian = '>' if m.is_bigendian else '<'
            ranges = []
            for row in range(m.height):
                for col in range(m.width):
                    offset = row*m.row_step+col*m.point_step
                    values = [struct.unpack_from(endian+'f', m.data, offset+fields[n].offset)[0]
                              for n in ('x', 'y', 'z', 'intensity', 'time')]
                    if all(math.isfinite(x) for x in values):
                        ranges.append(math.sqrt(sum(x*x for x in values[:3])))
                    else:
                        invalid += 1
            self.counts['cloud'] += 1
            event = {'type': 'cloud', 'id': self.counts['cloud'],
                     'stamp': m.header.stamp.sec+m.header.stamp.nanosec/1e9,
                     'valid': len(ranges), 'invalid': invalid,
                     'range_min': min(ranges) if ranges else None,
                     'range_max': max(ranges) if ranges else None}
            self.health.add(event, now)
        except (ValueError, struct.error):
            self.layout_errors += 1

    def imu(self, m):
        now = time.monotonic()
        if now < self.health.start:
            return
        if m.header.frame_id != 'unilidar_imu':
            self.layout_errors += 1
        self.counts['imu'] += 1
        self.health.add({'type': 'imu', 'id': self.counts['imu'],
                         'stamp': m.header.stamp.sec+m.header.stamp.nanosec/1e9,
                         'q': [m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w],
                         'gyro': [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z],
                         'accel': [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]}, now)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds', type=float, default=60)
    p.add_argument('--warmup', type=float, default=5)
    p.add_argument('--min-cloud-hz', type=float, default=5)
    p.add_argument('--min-imu-hz', type=float, default=100)
    p.add_argument('--max-gap', type=float, default=0.5)
    p.add_argument('--report', default='l1_ros_report.json')
    args, ros_args = p.parse_known_args()
    if not all(math.isfinite(x) and x > 0 for x in
               [args.seconds, args.warmup, args.min_cloud_hz, args.min_imu_hz, args.max_gap]):
        p.error('Numeric options must be finite and positive')
    rclpy.init(args=ros_args)
    node = Check(time.monotonic()+args.warmup)
    completed = False
    try:
        while rclpy.ok() and time.monotonic() < node.health.start+args.seconds:
            rclpy.spin_once(node, timeout_sec=0.02)
        completed = rclpy.ok()
    except KeyboardInterrupt:
        pass
    finally:
        r = node.health.report(time.monotonic(), args.min_cloud_hz, args.min_imu_hz, args.max_gap)
        r['checks']['message_layout'] = node.layout_errors == 0
        r['checks']['full_duration'] = completed
        r['layout_errors'] = node.layout_errors
        r['scope'] = 'ROS subscriber receipt test; sequence IDs are local counters, not device IDs.'
        r['passed'] = all(r['checks'].values())
        save_report(args.report, r)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(('PASS' if r['passed'] else 'FAIL')+f' — {args.report}')
    return 0 if r['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
