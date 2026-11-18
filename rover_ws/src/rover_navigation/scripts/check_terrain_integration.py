#!/usr/bin/env python3
"""Inspect the running terrain bench; optionally test expiry by pausing Gazebo."""
import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from terrain_geometry.msg import TerrainGrid
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--pause-test', action='store_true',
                        help='Pause/resume stl_terrain to test stale data')
    args = parser.parse_args()
    rclpy.init(args=[])
    node = rclpy.create_node('terrain_integration_check')
    buffer = Buffer()
    node.tf_listener = TransformListener(buffer, node)
    latest, counts, cloud_times = {}, Counter(), []
    reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

    def receive(key, message):
        latest[key] = message
        counts[key] += 1
        if key == 'cloud':
            cloud_times.append((time.monotonic(),
                                message.header.stamp.sec + message.header.stamp.nanosec / 1e9))

    topics = {
        'cloud': (PointCloud2, '/sim/terrain/cloud', sensor),
        'scan': (LaserScan, '/scan', sensor),
        'terrain': (TerrainGrid, '/terrain/grid', reliable),
        'cost': (OccupancyGrid, '/terrain_cost_grid', latched),
        'valid': (Bool, '/terrain/cost_grid_valid', reliable),
        'local': (OccupancyGrid, '/local_costmap/costmap', latched),
        'global': (OccupancyGrid, '/global_costmap/costmap', latched),
        'odom': (Odometry, '/odom', reliable),
    }
    node.test_subscriptions = [node.create_subscription(
        kind, topic, lambda message, key=key: receive(key, message), qos)
        for key, (kind, topic, qos) in topics.items()]

    def wait_until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def ready():
        return (all(key in latest for key in topics)
                and latest['valid'].data and counts['cloud'] >= 10
                and any(value >= 0 for value in latest['cost'].data)
                and all(any(value >= 0 for value in latest[key].data)
                        for key in ('local', 'global')))

    report = {}
    try:
        completed = wait_until(ready, args.timeout)
        known = {key: sum(v >= 0 for v in latest[key].data)
                 for key in ('cost', 'local', 'global') if key in latest}
        assert completed, f'Pipeline not ready: counts={dict(counts)}, known cells={known}'
        cloud = latest['cloud']
        xyz = point_cloud2.read_points_numpy(
            cloud, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        assert len(xyz) > 100, f'Only {len(xyz)} finite LiDAR returns'
        ranges = np.linalg.norm(xyz, axis=1)
        assert (ranges >= 0.09).all() and (ranges <= 30.1).all(), 'Returns outside sensor range'
        assert np.ptp(xyz[:, 0]) > 1 and np.ptp(xyz[:, 1]) > 1, 'Degenerate cloud'
        assert cloud.header.frame_id == 'terrain_sim_lidar'
        assert latest['scan'].header.frame_id == cloud.header.frame_id
        scan_ranges = np.asarray(latest['scan'].ranges)
        assert len(scan_ranges) > 300 and not np.isnan(scan_ranges).any()
        finite_scan = scan_ranges[np.isfinite(scan_ranges)]
        assert len(finite_scan) > 0, 'No finite returns in the horizontal obstacle slice'
        assert (finite_scan >= 0.1).all() and (finite_scan <= 30.0).all()
        for frame in ('terrain_sim_lidar', 'odom', 'base_link'):
            assert buffer.can_transform('terrain_map', frame, rclpy.time.Time()), (
                f'Missing TF to {frame}')
        tf = buffer.lookup_transform('terrain_map', cloud.header.frame_id,
                                     rclpy.time.Time.from_msg(cloud.header.stamp))
        assert abs(tf.transform.translation.z - 1.2) < 1e-6
        assert latest['terrain'].header.frame_id == latest['cost'].header.frame_id == 'terrain_map'
        assert latest['odom'].header.frame_id == 'odom'
        assert latest['odom'].child_frame_id == 'base_link'
        publishers = node.get_publishers_info_by_topic('/terrain_cost_grid')
        assert len(publishers) == 1 and publishers[0].node_name == 'terrain_cost_adapter'
        assert publishers[0].qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
        subscribers = node.get_subscriptions_info_by_topic('/terrain_cost_grid')
        assert {'local_costmap', 'global_costmap'} <= {entry.node_name for entry in subscribers}

        for name in ('controller_server', 'planner_server', 'behavior_server',
                     'velocity_smoother', 'collision_monitor', 'bt_navigator'):
            client = node.create_client(GetState, f'/{name}/get_state')
            assert client.wait_for_service(timeout_sec=5), f'{name} service missing'
            result = client.call_async(GetState.Request())
            assert wait_until(result.done, 5), f'{name} state timed out'
            state = result.result().current_state
            assert state.id == State.PRIMARY_STATE_ACTIVE, f'{name}: {state.label}'
            report[name] = state.label

        for name in ('controller_server', 'planner_server', 'behavior_server',
                     'velocity_smoother', 'collision_monitor', 'bt_navigator',
                     'terrain_geometry', 'terrain_cost_adapter', 'terrain_scan_converter',
                     'local_costmap/local_costmap', 'global_costmap/global_costmap'):
            client = node.create_client(GetParameters, f'/{name}/get_parameters')
            assert client.wait_for_service(timeout_sec=5), f'{name} parameter service missing'
            future = client.call_async(GetParameters.Request(names=['use_sim_time']))
            assert wait_until(future.done, 5)
            assert future.result().values[0].bool_value, f'{name}: wrong clock'
        report['simulation_clocks_checked'] = 11

        for key in ('cost', 'local', 'global'):
            data = np.asarray(latest[key].data)
            assert (data == -1).any(), f'{key}: unknown terrain lost'
            assert ((data > 0) & (data < 100)).any(), f'{key}: no graded terrain costs'
            report[key] = {'frame': latest[key].header.frame_id,
                           'known_cells': int((data >= 0).sum()),
                           'unknown_cells': int((data == -1).sum()),
                           'graded_cells': int(((data > 0) & (data < 100)).sum())}

        # Away from boundaries and measured obstacles, unknown input must survive
        # both costmaps. Neither scan clearing nor the footprint may free it.
        source = latest['cost']
        checked = Counter()
        for key in ('local', 'global'):
            target = latest[key]
            assert target.header.frame_id == 'odom'  # identity bench transform only
            for row in range(1, target.info.height - 1, 4):
                for col in range(1, target.info.width - 1, 4):
                    x = target.info.origin.position.x + (col + 0.5) * target.info.resolution
                    y = target.info.origin.position.y + (row + 0.5) * target.info.resolution
                    sx = int(np.floor(
                        (x - source.info.origin.position.x) / source.info.resolution))
                    sy = int(np.floor(
                        (y - source.info.origin.position.y) / source.info.resolution))
                    if not (1 <= sx < source.info.width - 1 and 1 <= sy < source.info.height - 1):
                        continue
                    # Sample inside a source cell, away from rounding at its edges.
                    fx = (x - source.info.origin.position.x) / source.info.resolution - sx
                    fy = (y - source.info.origin.position.y) / source.info.resolution - sy
                    if (0.2 < fx < 0.8 and 0.2 < fy < 0.8
                            and source.data[sy * source.info.width + sx] == -1):
                        assert target.data[row * target.info.width + col] == -1, (
                            f'{key}: unknown cleared')
                        checked[key] += 1
            report[key]['unknown_samples_checked'] = checked[key]
            assert checked[key] > 0, f'{key}: no unknown samples verified'

        # Request a plan between observed cells. This never requests robot motion;
        # use_start explicitly supplies the start instead of the bench's shadow.
        grid = latest['global']
        data = np.asarray(grid.data).reshape(grid.info.height, grid.info.width)
        rows, cols = np.nonzero((data >= 0) & (data < 50))
        xs = grid.info.origin.position.x + (cols + 0.5) * grid.info.resolution
        ys = grid.info.origin.position.y + (rows + 0.5) * grid.info.resolution
        selected = (np.abs(ys) < 0.2) & (np.abs(xs) < 2.5)
        candidates = np.column_stack((xs[selected], ys[selected]))
        assert len(candidates) > 2
        start = candidates[np.argmin(candidates[:, 0])]
        goal = candidates[np.argmax(candidates[:, 0])]
        assert goal[0] - start[0] > 2

        def pose(xy):
            stamped = PoseStamped()
            stamped.header.frame_id = 'odom'
            stamped.pose.position.x, stamped.pose.position.y = map(float, xy)
            stamped.pose.orientation.w = 1.0
            return stamped

        planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        assert planner.wait_for_server(timeout_sec=5), 'Planner action unavailable'
        request = ComputePathToPose.Goal(
            start=pose(start), goal=pose(goal), use_start=True, planner_id='GridBased')
        future = planner.send_goal_async(request)
        assert wait_until(future.done, 5) and future.result().accepted
        result = future.result().get_result_async()
        assert wait_until(result.done, 15), 'Planning timed out'
        outcome = result.result()
        assert outcome.status == GoalStatus.STATUS_SUCCEEDED, f'Planner failed: {outcome.result}'
        path = outcome.result.path.poses
        assert len(path) > 2
        length = sum(
            np.hypot(b.pose.position.x - a.pose.position.x,
                     b.pose.position.y - a.pose.position.y)
            for a, b in zip(path, path[1:]))
        report['planner'] = {
            'start_xy': start.tolist(), 'goal_xy': goal.tolist(), 'poses': len(path),
            'length_m': round(length, 3)}

        duration = cloud_times[-1][0] - cloud_times[0][0]
        sim_duration = cloud_times[-1][1] - cloud_times[0][1]
        report['lidar'] = {'finite_points': len(xyz), 'frame': cloud.header.frame_id,
                           'scan_bins': len(scan_ranges), 'finite_scan_bins': len(finite_scan),
                           'wall_hz': round((len(cloud_times) - 1) / duration, 2),
                           'sim_hz': round((len(cloud_times) - 1) / sim_duration, 2),
                           'xyz_min': xyz.min(axis=0).tolist(),
                           'xyz_max': xyz.max(axis=0).tolist()}

        if args.pause_test:
            def pause(value):
                subprocess.run([
                    'gz', 'service', '-s', '/world/stl_terrain/control',
                    '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000', '--req', f'pause: {str(value).lower()}'],
                    check=True, timeout=10)
            try:
                pause(True)
                started = time.monotonic()
                assert wait_until(lambda: not latest['valid'].data
                                  and all(value == -1 for value in latest['cost'].data), 5), (
                    'Stale terrain did not expire')
                report['expiry_seconds_wall'] = round(time.monotonic() - started, 2)
            finally:
                pause(False)
            assert wait_until(lambda: latest['valid'].data
                              and any(value >= 0 for value in latest['cost'].data), 15), (
                'Terrain failed to recover')
            report['recovered_after_resume'] = True
        report['result'] = 'PASS'
        output = json.dumps(report, indent=2)
        print(output)
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(output + '\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
