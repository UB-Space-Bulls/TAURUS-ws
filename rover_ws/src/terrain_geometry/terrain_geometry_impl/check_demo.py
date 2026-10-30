"""Collect live demo snapshots and print geometry checks as JSON."""
import argparse
import json
from types import SimpleNamespace
import time
import numpy as np
import rclpy
from rclpy.node import Node
from terrain_geometry.msg import TerrainGrid
from .core import Config
from .demo_scene import summarize, acceptance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20.)
    args, ros_args = parser.parse_known_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    rclpy.init(args=ros_args)
    node = Node('check_terrain_demo')
    records = []

    def received(msg):
        if msg.header.frame_id != 'terrain_map':
            node.get_logger().error('These fixed-region checks require the isolated terrain_map demo.')
            return
        shape = (msg.info.height, msg.info.width)
        if any(len(v) != shape[0]*shape[1] for v in
               (msg.status, msg.ground_height, msg.slope, msg.roughness)):
            node.get_logger().error('Malformed grid arrays.')
            return
        result = SimpleNamespace(
            status=np.asarray(msg.status).reshape(shape),
            height=np.asarray(msg.ground_height).reshape(shape),
            slope=np.asarray(msg.slope).reshape(shape),
            roughness=np.asarray(msg.roughness).reshape(shape))
        cfg = Config(resolution=msg.info.resolution, width=msg.info.width,
                     height=msg.info.height, origin_x=msg.info.origin.position.x,
                     origin_y=msg.info.origin.position.y)
        metrics = summarize(result, cfg)
        records.append({'stamp_sec': msg.header.stamp.sec,
                        'stamp_nanosec': msg.header.stamp.nanosec,
                        'metrics': metrics, 'checks': acceptance(metrics)})

    subscription = node.create_subscription(TerrainGrid, '/terrain/grid', received, 1)
    end = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    # Distinct acquisition stamps ensure replayed duplicates do not count.
    unique = {(r['stamp_sec'], r['stamp_nanosec']): r for r in records}
    snapshots = list(unique.values())[-10:]
    passed = len(snapshots) >= 5 and all(all(r['checks'].values()) for r in snapshots)
    print(json.dumps({'source': 'live ROS topic /terrain/grid',
                      'distinct_snapshots': len(unique), 'passed': passed,
                      'last_snapshots': snapshots}, indent=2, allow_nan=False))
    raise SystemExit(0 if passed else 1)
