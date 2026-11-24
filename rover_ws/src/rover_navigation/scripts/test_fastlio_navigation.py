#!/usr/bin/env python3
"""Inspect fastlio_obstacle; --navigate commands a goal behind the box."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from std_msgs.msg import Bool, Float64MultiArray
from terrain_geometry.msg import TerrainGrid
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def box_clearance(x, y):
    return math.hypot(max(abs(x-2.)-0.4, 0.), max(abs(y)-0.4, 0.))


def region_cost(grid, x=2., y=0., radius=0.3):
    ox, oy, r = grid.info.origin.position.x, grid.info.origin.position.y, grid.info.resolution
    values = np.asarray(grid.data).reshape(grid.info.height, grid.info.width)
    i0, i1 = max(0, int((x-radius-ox)/r)), min(grid.info.width, int((x+radius-ox)/r)+1)
    j0, j1 = max(0, int((y-radius-oy)/r)), min(grid.info.height, int((y+radius-oy)/r)+1)
    return int(values[j0:j1, i0:i1].max()) if i1 > i0 and j1 > j0 else -1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--navigate', action='store_true')
    parser.add_argument('--pause-test', action='store_true')
    parser.add_argument('--report', type=Path, default=Path('/tmp/fastlio_obstacle_validation.json'))
    parser.add_argument('--timeout', type=float, default=240.)
    args = parser.parse_args()
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'],
               signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('fastlio_navigation_check')
    latest, counts, truth_trace, plans, dynamic_edges = {}, {}, [], [], set()
    received_grid_stamps, registered_stamps = set(), set()
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def receive(key, message):
        latest[key] = message
        counts[key] = counts.get(key, 0)+1
        if key == 'truth':
            p = message.pose.pose.position
            truth_trace.append((p.x, p.y))
        if key == 'plan':
            plans.append([(p.pose.position.x, p.pose.position.y) for p in message.poses])
        if key in ('registered', 'terrain'):
            stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
            (registered_stamps if key == 'registered' else received_grid_stamps).add(stamp)

    topics = {
        'imu': (Imu, '/rover/imu', qos_profile_sensor_data),
        'raw': (PointCloud2, '/rover/lidar/points', qos_profile_sensor_data),
        'scan': (LaserScan, '/scan', qos_profile_sensor_data),
        'estimate': (Odometry, '/Odometry', 10),
        'odom': (Odometry, '/odom', 10),
        'truth': (Odometry, '/ground_truth/odom', 10),
        'registered': (PointCloud2, '/cloud_registered', qos_profile_sensor_data),
        'terrain': (TerrainGrid, '/terrain/grid', 10),
        'cost': (OccupancyGrid, '/terrain_cost_grid', latched),
        'local': (OccupancyGrid, '/local_costmap/costmap', latched),
        'global': (OccupancyGrid, '/global_costmap/costmap', latched),
        'terrain_valid': (Bool, '/terrain/cost_grid_valid', 1),
        'localization_valid': (Bool, '/localization/valid', 1),
        'cmd': (Twist, '/cmd_vel', 10),
        'wheels': (Float64MultiArray, '/wheel_controller/commands', 10),
    }
    subscriptions = [node.create_subscription(kind, topic, lambda m, key=key: receive(key, m), qos)
                     for key, (kind, topic, qos) in topics.items()]
    subscriptions.append(node.create_subscription(NavPath, '/plan', lambda m: receive('plan', m), 10))

    def transforms(message):
        for tf in message.transforms:
            edge = (tf.header.frame_id, tf.child_frame_id)
            dynamic_edges.add(edge)

    subscriptions.append(node.create_subscription(TFMessage, '/tf', transforms, 100))
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    handle = None
    report = {'scope': 'Live Gazebo LiDAR/IMU -> FAST-LIO -> terrain/costmaps -> Nav2 -> rover',
              'result': 'FAIL'}

    def wait(predicate, timeout):
        end = time.monotonic()+timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    try:
        assert wait(lambda: all(k in latest for k in topics)
                    and latest['terrain_valid'].data and latest['localization_valid'].data
                    and counts.get('estimate', 0) > 5, 120), f'Pipeline not ready: {counts}'
        publishers = node.get_publishers_info_by_topic('/odom')
        assert len(publishers) == 1 and publishers[0].node_name == 'fastlio_sim_bridge', publishers
        assert latest['registered'].header.frame_id == 'camera_init'
        assert latest['terrain'].header.frame_id == latest['cost'].header.frame_id == 'odom'
        for frame in ('body', 'base_link', 'lidar_link', 'terrain_lidar_link'):
            assert buffer.can_transform('odom', frame, rclpy.time.Time()), frame
        assert ('odom', 'base_link') not in dynamic_edges, 'Ground-truth TF leaked into navigation'
        assert ('camera_init', 'body') in dynamic_edges, dynamic_edges
        tf_publishers = [p.node_name for p in node.get_publishers_info_by_topic('/tf')]
        assert sorted(tf_publishers) == ['laser_mapping', 'robot_state_publisher'], tf_publishers
        assert wait(lambda: bool(received_grid_stamps & registered_stamps), 10), 'Cloud stamps not preserved'
        assert wait(lambda: latest['terrain_valid'].data and all(
            region_cost(latest[key]) == 100 for key in ('cost', 'local', 'global')), 30), (
                {key: region_cost(latest[key]) for key in ('cost', 'local', 'global')})
        report['obstacle_costs'] = {key: region_cost(latest[key]) for key in ('cost', 'local', 'global')}
        scan = latest['scan']
        assert scan.header.frame_id == 'lidar_link'
        forward = [v for i, v in enumerate(scan.ranges)
                   if abs(scan.angle_min+i*scan.angle_increment) < 0.08 and math.isfinite(v)]
        assert forward and min(forward) < 2., '2D scan does not detect the box'
        report['forward_box_scan_range_m'] = min(forward)
        report['odom_publisher'] = publishers[0].node_name
        report['source_stamps_preserved'] = True
        report['single_dynamic_tf_authority'] = True
        report['dynamic_tf_publishers'] = tf_publishers
        print('Pipeline ready; FAST-LIO owns moving TF and all three grids mark the box.', flush=True)
        if args.navigate:
            assert wait(lambda: latest['terrain_valid'].data and
                        0 <= region_cost(latest['global'], 4., 0., 0.05) < 90, 30), 'Goal is unknown/blocked'
            start_truth = np.asarray(truth_trace[-1])
            start_pose = latest['odom'].pose.pose.position
            start_estimate = np.asarray([start_pose.x, start_pose.y])
            truth_trace.clear()
            request = NavigateToPose.Goal()
            request.pose.header.frame_id = 'odom'
            request.pose.pose.position.x = 4.
            request.pose.pose.orientation.w = 1.
            client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
            assert client.wait_for_server(timeout_sec=10)
            last_feedback = [0.]
            def feedback(message):
                if time.monotonic()-last_feedback[0] > 10.:
                    last_feedback[0] = time.monotonic()
                    print(f'Navigation distance remaining: {message.feedback.distance_remaining:.2f} m', flush=True)
            future = client.send_goal_async(request, feedback_callback=feedback)
            assert wait(future.done, 10) and future.result().accepted, 'Goal rejected'
            handle = future.result()
            result = handle.get_result_async()
            assert wait(result.done, args.timeout), 'Navigation timed out'
            outcome = result.result()
            report['navigation_status'] = outcome.status
            report['navigation_result'] = str(outcome.result)
            report['truth_trajectory'] = truth_trace[::max(1, len(truth_trace)//200)]
            assert truth_trace, 'No independent ground-truth diagnostic samples'
            report['min_box_clearance_m'] = min(box_clearance(x, y) for x, y in truth_trace)
            report['max_lateral_detour_m'] = max(abs(y) for x, y in truth_trace)
            final = latest['odom'].pose.pose.position
            report['estimated_goal_error_m'] = math.hypot(final.x-4., final.y)
            report['actual_goal_error_m'] = float(np.linalg.norm(np.asarray(truth_trace[-1])-
                                                               (start_truth+np.asarray([4., 0.])-start_estimate)))
            report['translation_disagreement_m'] = float(np.linalg.norm(
                (np.asarray(truth_trace[-1])-start_truth)-
                (np.asarray([final.x, final.y])-start_estimate)))
            report['planned_detour'] = any(any(1.6 < x < 2.4 and abs(y) > 0.9 for x, y in plan)
                                            for plan in plans)
            assert outcome.status == GoalStatus.STATUS_SUCCEEDED, str(outcome.result)
            assert report['estimated_goal_error_m'] < 0.35
            assert report['actual_goal_error_m'] < 0.5
            assert report['min_box_clearance_m'] > 0.55, 'Rover approached the physical box too closely'
            assert report['max_lateral_detour_m'] > 0.9 and report['planned_detour']
            assert report['translation_disagreement_m'] < 0.35
        if args.pause_test:
            def pause(value):
                subprocess.run(['gz', 'service', '-s', '/world/fastlio_obstacle_course/control',
                    '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000', '--req', f'pause: {str(value).lower()}'],
                    check=True, timeout=10, capture_output=True, text=True)
            probe = node.create_publisher(Twist, '/cmd_vel_terrain_input', 10)
            try:
                pause(True)
                assert wait(lambda: not latest['terrain_valid'].data
                            and not latest['localization_valid'].data
                            and all(v == -1 for v in latest['cost'].data), 8), 'Paused data remained valid'
                end = time.monotonic()+1.
                while time.monotonic() < end:
                    command = Twist()
                    command.linear.x = 0.1
                    probe.publish(command)
                    rclpy.spin_once(node, timeout_sec=0.05)
                assert abs(latest['cmd'].linear.x)+abs(latest['cmd'].angular.z) < 1e-9
                assert max(abs(v) for v in latest['wheels'].data) < 1e-9
                report['paused_sensor_data_blocks_motion'] = True
            finally:
                probe.publish(Twist())
                pause(False)
            assert wait(lambda: latest['terrain_valid'].data
                        and latest['localization_valid'].data, 30), 'Pipeline did not recover after resume'
            report['recovered_after_resume'] = True
        report['messages'] = counts
        report['result'] = 'PASS'
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = str(exc) or 'Interrupted'
        report['messages'] = counts
        raise
    finally:
        if handle is not None and rclpy.ok():
            cancel = handle.cancel_goal_async()
            wait(cancel.done, 2)
        report['dynamic_tf_edges'] = sorted(' -> '.join(edge) for edge in dynamic_edges)
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k: v for k, v in report.items() if k != 'truth_trajectory'}, indent=2), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
