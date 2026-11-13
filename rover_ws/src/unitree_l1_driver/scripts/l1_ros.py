#!/usr/bin/env python3
"""ROS 2 bench driver. A child process owns all blocking SDK calls."""
import json
import math
import struct
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField, Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from l1_common import Health, Worker, valid_imu, worker_path


class L1Node(Node):
    def __init__(self):
        super().__init__('unitree_l1_driver')
        port = self.declare_parameter('port', '/dev/ttyUSB0').value
        scans = self.declare_parameter('cloud_scan_num', 18).value
        self.warmup = self.declare_parameter('warmup_seconds', 10.0).value
        self.stall = self.declare_parameter('stall_timeout', 3.0).value
        if not 1 <= scans <= 180 or self.warmup < 0 or self.stall <= 0:
            raise ValueError('Invalid scan count/warmup/stall timeout')
        self.cloud_pub = self.create_publisher(PointCloud2, '/unilidar/cloud', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, '/unilidar/imu', qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, '/unilidar/imu_orientation', qos_profile_sensor_data)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.worker = Worker([worker_path('sdk'), '--port', port, '--scans', str(scans), '--points'])
        self.started = time.monotonic()
        self.ready = None
        self.health = None
        self.failed = False
        self.reason = None
        self.min_cloud = 90.0/scans  # Half of 180 scans/s; bench threshold, not a spec.
        self.last_diag = self.started
        self.create_timer(0.001, self.pump)
        self.create_timer(1.0, self.diagnose)
        self.get_logger().info(f'Opening {port}; cloud_scan_num={scans}. Host receipt stamps; no SLAM.')

    def pump(self):
        if self.failed:
            return
        try:
            events = self.worker.poll(0)
            now_mono = time.monotonic()
            for event in events:
                if event['type'] == 'ready':
                    self.ready = now_mono
                    self.health = Health(now_mono+self.warmup)
                if self.health:
                    self.health.add(event, now_mono)
                now_ros = self.get_clock().now()
                stamp = now_ros.to_msg()
                if event['type'] == 'cloud':
                    msg = PointCloud2()
                    msg.header.frame_id = 'unilidar_lidar'
                    msg.header.stamp = stamp
                    msg.height, msg.width = 1, len(event['points'])
                    msg.fields = [PointField(name=n, offset=i*4,
                        datatype=PointField.UINT32 if n == 'ring' else PointField.FLOAT32, count=1)
                        for i, n in enumerate(('x', 'y', 'z', 'intensity', 'time', 'ring'))]
                    msg.is_bigendian, msg.is_dense = False, True
                    msg.point_step = 24
                    msg.row_step = 24*msg.width
                    msg.data = b''.join(struct.pack('<5fI', *point) for point in event['points'])
                    self.cloud_pub.publish(msg)
                elif event['type'] == 'imu' and valid_imu(event):
                    msg = Imu()
                    msg.header.stamp = stamp
                    msg.header.frame_id = 'unilidar_imu'
                    q = event['q']
                    norm = math.sqrt(sum(x*x for x in q))
                    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = [x/norm for x in q]
                    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = event['gyro']
                    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = event['accel']
                    # All-zero covariance means covariance unknown, not perfect certainty.
                    self.imu_pub.publish(msg)
                    pose = PoseStamped()
                    pose.header.stamp = stamp
                    pose.header.frame_id = 'imu_orientation_reference'
                    pose.pose.orientation = msg.orientation
                    self.pose_pub.publish(pose)
            if self.worker.process.poll() is not None:
                raise RuntimeError(f'Reader exited {self.worker.process.returncode}: {self.worker.log_tail()}')
            if self.ready is None and now_mono-self.started > 10:
                raise RuntimeError('SDK initialization timeout')
            if self.health and now_mono >= self.health.start:
                for name, stream in [('cloud', self.health.cloud), ('IMU', self.health.imu)]:
                    age = now_mono-(stream.last if stream.last is not None else self.health.start)
                    if age > self.stall:
                        raise RuntimeError(f'{name} absent/stalled > {self.stall}s; restart after fixing the connection')
        except Exception as exc:
            self.failed, self.reason = True, str(exc)
            self.get_logger().error(self.reason)
            self.worker.stop()
            self.diagnose()  # Remain alive to publish ERROR; never silently reconnect.

    def diagnose(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(name='L1 bench stream', hardware_id='Unitree L1')
        now = time.monotonic()
        if self.failed:
            status.level, status.message = DiagnosticStatus.ERROR, self.reason
        elif not self.health or now < self.health.start+1:
            status.level, status.message = DiagnosticStatus.WARN, 'Initializing/warming up'
        else:
            report = self.health.report(now, self.min_cloud, 100, 0.5)
            status.level = DiagnosticStatus.OK if report['passed'] else DiagnosticStatus.WARN
            status.message = 'Cumulative data check passed' if report['passed'] else 'Data check needs attention'
            status.values = [KeyValue(key=k, value=json.dumps(v, allow_nan=False)) for k, v in report.items()]
        status.values += [KeyValue(key='power_verified', value='false'),
                          KeyValue(key='malformed_worker_records', value=str(self.worker.malformed))]
        if self.worker.malformed:
            status.level, status.message = DiagnosticStatus.ERROR, 'Malformed worker records'
        msg.status = [status]
        self.diag_pub.publish(msg)

    def destroy_node(self):
        self.worker.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = L1Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
