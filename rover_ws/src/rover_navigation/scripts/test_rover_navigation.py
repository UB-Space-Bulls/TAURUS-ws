#!/usr/bin/env python3
"""Inspect terrain_rover; --drive/--navigate explicitly command the simulated rover."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time

from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float64MultiArray
from terrain_geometry.msg import TerrainGrid
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drive', action='store_true')
    parser.add_argument('--navigate', action='store_true')
    parser.add_argument('--goal-distance', type=float, default=1.5)
    parser.add_argument('--pause-test', action='store_true')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    assert 0.5 <= args.goal_distance <= 3.0, 'Use a short 0.5..3 m observed-terrain goal'
    rclpy.init(args=[])
    node = rclpy.create_node('moving_terrain_test')
    buffer = Buffer()
    node.tf_listener = TransformListener(buffer, node)
    latest, counts = {}, {}
    positions, wheel_commands, velocity_commands = [], [], []

    def receive(key, message):
        latest[key] = message
        counts[key] = counts.get(key, 0) + 1
        if key == 'odom':
            p = message.pose.pose.position
            positions.append((p.x, p.y, p.z))
        if key == 'wheels':
            wheel_commands.append(max(abs(v) for v in message.data))
        if key == 'velocity':
            velocity_commands.append(abs(message.linear.x) + abs(message.angular.z))

    reliable = QoSProfile(depth=1)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    topics = {
        'cloud': (PointCloud2, '/rover/lidar/points', sensor),
        'scan': (LaserScan, '/scan', sensor),
        'odom': (Odometry, '/odom', reliable),
        'joints': (JointState, '/joint_states', sensor),
        'terrain': (TerrainGrid, '/terrain/grid', reliable),
        'cost': (OccupancyGrid, '/terrain_cost_grid', latched),
        'valid': (Bool, '/terrain/cost_grid_valid', reliable),
        'local': (OccupancyGrid, '/local_costmap/costmap', latched),
        'global': (OccupancyGrid, '/global_costmap/costmap', latched),
        'velocity': (Twist, '/cmd_vel', reliable),
        'wheels': (Float64MultiArray, '/wheel_controller/commands', reliable),
    }
    node.test_subscriptions = [node.create_subscription(
        kind, topic, lambda message, key=key: receive(key, message), qos)
        for key, (kind, topic, qos) in topics.items()]
    command = node.create_publisher(Twist, '/cmd_vel_nav', 10)
    goal_handle = None

    def wait(predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def state(name):
        client = node.create_client(GetState, f'/{name}/get_state')
        assert client.wait_for_service(timeout_sec=5), f'{name} state service missing'
        future = client.call_async(GetState.Request())
        assert wait(future.done, 5), f'{name} state timed out'
        assert future.result().current_state.id == State.PRIMARY_STATE_ACTIVE, (
            f'{name}: {future.result().current_state.label}')

    def position():
        return np.asarray(positions[-1][:2])

    report = {}
    try:
        def ready():
            return (all(key in latest for key in topics)
                    and latest['valid'].data and counts.get('cloud', 0) >= 5
                    and all(any(v >= 0 for v in latest[key].data)
                            for key in ('cost', 'local', 'global')))
        assert wait(ready, 90), f'Pipeline not ready: {counts}'
        # Allow the dropped-in rover to settle before testing its pose.
        wait(lambda: False, 3)
        controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
        assert controllers.wait_for_service(timeout_sec=5)
        future = controllers.call_async(ListControllers.Request())
        assert wait(future.done, 5)
        states = {c.name: c.state for c in future.result().controller}
        assert all(states.get(name) == 'active' for name in (
            'joint_state_broadcaster', 'steer_controller', 'wheel_controller')), states
        report['controllers'] = states
        for name in ('controller_server', 'planner_server', 'behavior_server',
                     'velocity_smoother', 'collision_monitor', 'bt_navigator'):
            state(name)
        report['nav2_lifecycle'] = 'all six nodes active'
        cloud = latest['cloud']
        assert cloud.header.frame_id == 'terrain_lidar_link'
        xyz = point_cloud2.read_points_numpy(
            cloud, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        assert len(xyz) > 100
        assert np.ptp(xyz[:, 0]) > 1 and np.ptp(xyz[:, 1]) > 1
        assert latest['scan'].header.frame_id == 'lidar_link'
        assert len(latest['scan'].ranges) >= 300
        for frame in ('base_link', 'lidar_link', 'terrain_lidar_link'):
            assert buffer.can_transform('odom', frame, rclpy.time.Time()), frame
        assert latest['terrain'].header.frame_id == latest['cost'].header.frame_id == 'odom'
        # Lifecycle/service checks can span an upstream refresh or expiry.
        # Wait for measured costs again before taking the verification snapshot.
        assert wait(lambda: latest['valid'].data
                    and all(any(0 < v < 100 for v in latest[key].data)
                            for key in ('cost', 'local', 'global')), 20), (
            'No fresh graded terrain snapshot after lifecycle checks')
        for key in ('cost', 'local', 'global'):
            values = np.asarray(latest[key].data)
            # The 6 m local window can be entirely observed except for the
            # occupied footprint, which this controlled simulation clears.
            if key != 'local':
                assert (values == -1).any(), f'{key}: missing unknown space'
            assert ((values > 0) & (values < 100)).any(), f'{key}: no graded costs'
            report[key] = {'known': int((values >= 0).sum()),
                           'unknown': int((values == -1).sum()),
                           'graded': int(((values > 0) & (values < 100)).sum())}
        assert len(node.get_publishers_info_by_topic('/terrain_cost_grid')) == 1
        report['finite_lidar_points'] = len(xyz)
        report['initial_xyz'] = positions[-1]
        report['initial_grid_origin'] = [latest['cost'].info.origin.position.x,
                                         latest['cost'].info.origin.position.y]

        if args.drive:
            start = position()
            timer = node.create_timer(0.1, lambda: command.publish(
                Twist(linear=type(Twist().linear)(x=0.1))))
            try:
                wait(lambda: False, 6)
            finally:
                node.destroy_timer(timer)
                command.publish(Twist())
            wait(lambda: False, 1)
            displacement = float(np.linalg.norm(position() - start))
            report['manual_drive_displacement_m'] = round(displacement, 3)
            assert displacement > 0.15, f'Wheel commands did not move rover: {displacement}'

        if args.navigate:
            start = position()
            q = latest['odom'].pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            target = start + args.goal_distance * np.asarray([math.cos(yaw), math.sin(yaw)])
            request = NavigateToPose.Goal()
            request.pose.header.frame_id = 'odom'
            request.pose.pose.position.x, request.pose.pose.position.y = map(float, target)
            request.pose.pose.orientation.z = math.sin(yaw / 2)
            request.pose.pose.orientation.w = math.cos(yaw / 2)
            client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
            assert client.wait_for_server(timeout_sec=10)
            future = client.send_goal_async(request)
            assert wait(future.done, 5) and future.result().accepted
            goal_handle = future.result()
            outcome = goal_handle.get_result_async()
            assert wait(outcome.done, 120), 'Navigation timed out'
            result = outcome.result()
            report['navigation_status'] = result.status
            report['navigation_result'] = str(result.result)
            report['goal_xy'] = target.tolist()
            report['goal_error_m'] = round(float(np.linalg.norm(position() - target)), 3)
            report['navigation_displacement_m'] = round(
                float(np.linalg.norm(position() - start)), 3)
            assert result.status == GoalStatus.STATUS_SUCCEEDED, str(result.result)
            assert report['goal_error_m'] < 0.35
            assert report['navigation_displacement_m'] > args.goal_distance - 0.4

        if args.pause_test:
            def pause(value):
                subprocess.run([
                    'gz', 'service', '-s', '/world/stl_terrain/control',
                    '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000', '--req', f'pause: {str(value).lower()}'],
                    check=True, timeout=10)
            try:
                pause(True)
                assert wait(lambda: not latest['valid'].data
                            and all(v == -1 for v in latest['cost'].data), 6)
                probe = node.create_publisher(Twist, '/cmd_vel_terrain_input', 10)
                timer = node.create_timer(0.1, lambda: probe.publish(
                    Twist(linear=type(Twist().linear)(x=0.1))))
                try:
                    wait(lambda: False, 1)
                finally:
                    node.destroy_timer(timer)
                    probe.publish(Twist())
                assert wait(lambda: abs(latest['velocity'].linear.x) < 1e-9
                            and max(abs(v) for v in latest['wheels'].data) < 1e-9, 2)
                report['stale_terrain_stops_commands'] = True
            finally:
                pause(False)
            assert wait(lambda: latest['valid'].data, 15)
            report['recovered_after_resume'] = True

        report['final_xyz'] = positions[-1]
        report['final_grid_origin'] = [latest['cost'].info.origin.position.x,
                                       latest['cost'].info.origin.position.y]
        report['max_wheel_command_rad_s'] = max(wheel_commands)
        report['max_velocity_command'] = max(velocity_commands)
        report['result'] = 'PASS'
    finally:
        if goal_handle is not None:
            cancellation = goal_handle.cancel_goal_async()
            wait(cancellation.done, 2)
        if args.drive or args.navigate:
            command.publish(Twist())
        text = json.dumps(report, indent=2)
        print(text, flush=True)
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(text + '\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
