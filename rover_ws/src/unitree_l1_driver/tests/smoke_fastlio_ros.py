"""Bounded PTY -> vendor SDK -> ROS driver -> FAST-LIO transport check."""
import json
import math
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField, Imu
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray

ROOT = Path(__file__).resolve().parents[3]
os.environ.setdefault('ROS_DOMAIN_ID', '192')
os.environ.setdefault('ROS_LOG_DIR', '/tmp/l1_fastlio_ros_logs')
sys.path.insert(0, str(ROOT / 'src/unitree_l1_driver/tests'))
from test_serial import frame


def packets(seq):
    # Broad synthetic scan geometry, not a model of real L1 optics or accuracy.
    aux = bytearray(209)
    struct.pack_into('<5I', aux, 0, 0, 10+seq//180, (seq % 180)*5555, 100000, 5555)
    struct.pack_into('<4f', aux, 20, 0.0, 360.0, -15.0+(seq % 18)*2.0, 0.0)
    struct.pack_into('<HHB', aux, 84, seq % 65536, 120, 1)
    aux[89:] = bytes([50])*120
    dist = struct.pack('<HHH', seq % 65536, 1, 240)+struct.pack('<120H', *([3000]*120))
    imu = struct.pack('<10fH', 1, 0, 0, 0, 0, 0, 0, 0, 0, 9.81, seq % 65536)
    return frame(17, aux, seq*3)+frame(16, dist, seq*3+1)+frame(19, imu, seq*3+2)

master, slave = pty.openpty()
port = os.ttyname(slave)
os.set_blocking(master, False)
stop = threading.Event()


def produce():
    seq = 0
    deadline = time.monotonic()
    while not stop.is_set():
        payload = packets(seq)
        try:
            os.write(master, payload)
        except BlockingIOError:
            pass
        seq += 1
        deadline += 1 / 180
        stop.wait(max(0, deadline-time.monotonic()))


rclpy.init()
node = rclpy.create_node('l1_fastlio_transport_check')
counts = {'cloud': 0, 'imu': 0, 'odometry': 0}
errors = []
frames = {}
first_odom = None
last_odom = None


def cloud(message):
    counts['cloud'] += 1
    fields = {field.name: field for field in message.fields}
    if fields['ring'].datatype != PointField.UINT16:
        errors.append('ring is not UINT16')
    if fields['time'].datatype != PointField.FLOAT32:
        errors.append('time is not FLOAT32')
    import struct
    offsets = [struct.unpack_from('<f', message.data, i*message.point_step+16)[0]
               for i in range(message.width)]
    if offsets != sorted(offsets) or offsets[-1] <= 0:
        errors.append('point times not ordered/positive')
    frames['cloud'] = message.header.frame_id


def imu(message):
    counts['imu'] += 1
    frames['imu'] = message.header.frame_id


def odometry(message):
    global first_odom, last_odom
    counts['odometry'] += 1
    frames['odometry'] = [message.header.frame_id, message.child_frame_id]
    stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
    if first_odom is None:
        first_odom = stamp
    last_odom = stamp
    if not all(math.isfinite(x) for x in (
            message.pose.pose.position.x, message.pose.pose.position.y,
            message.pose.pose.position.z)):
        errors.append('nonfinite odometry')


def diagnostics(message):
    for status in message.status:
        if status.name == 'L1 bench stream' and status.level == 2:
            errors.append(status.message)


node.create_subscription(PointCloud2, '/unilidar/cloud', cloud, qos_profile_sensor_data)
node.create_subscription(Imu, '/unilidar/imu', imu, qos_profile_sensor_data)
node.create_subscription(Odometry, '/Odometry', odometry, 10)
node.create_subscription(DiagnosticArray, '/diagnostics', diagnostics, 10)
log = open('/tmp/l1_fastlio_runtime.log', 'w')
launch = subprocess.Popen([
    'ros2', 'launch', 'unitree_l1_driver', 'l1_fastlio.launch.py',
    f'port:={port}', 'rviz:=false'], cwd=ROOT, stdout=log,
    stderr=subprocess.STDOUT, start_new_session=True)
producer = threading.Thread(target=produce, daemon=True)
producer.start()
try:
    deadline = time.monotonic()+18
    while time.monotonic() < deadline and launch.poll() is None and not errors:
        rclpy.spin_once(node, timeout_sec=0.05)
finally:
    if launch.poll() is None:
        launch.send_signal(signal.SIGINT)  # launch forwards it once to its nodes.
        try:
            launch.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGTERM)
            launch.wait(timeout=3)
    stop.set()
    producer.join(timeout=1)
    node.destroy_node()
    rclpy.shutdown()
    os.close(master)
    os.close(slave)
    log.close()

result = {'scope': 'Synthetic PTY transport/initialization only; no physical accuracy test',
          'counts': counts, 'frames': frames, 'errors': errors,
          'first_odom_stamp': first_odom, 'last_odom_stamp': last_odom,
          'launch_exit_code': launch.returncode}
runtime_log = Path('/tmp/l1_fastlio_runtime.log').read_text()
result['clean_shutdown'] = ('Traceback' not in runtime_log and
                            'process has died' not in runtime_log and launch.returncode == 0)
result['passed'] = (not errors and all(value > 1 for value in counts.values())
                    and last_odom > first_odom and result['clean_shutdown'])
Path('/tmp/l1_fastlio_runtime_result.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
sys.exit(0 if result['passed'] else 1)
