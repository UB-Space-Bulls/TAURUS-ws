from copy import deepcopy
import math
from types import SimpleNamespace as NS
import unittest
import numpy as np

from terrain_geometry_impl.nav2_adapter import CostPolicy, SnapshotLease, occupancy_values, validate_geometry
from terrain_geometry_impl.nav2_config import physical_profile


def convert(flags, slope=None, rough=None, heights=None, counts=None):
    n = len(flags)
    return occupancy_values(flags, heights if heights is not None else [0.0]*n,
        np.radians(slope if slope is not None else [0.0]*n),
        rough if rough is not None else [0.0]*n,
        counts if counts is not None else [9]*n, [0]*n, n, CostPolicy())


class CostTests(unittest.TestCase):
    def test_unknown_and_obstacle_priority(self):
        self.assertEqual(convert([0, 1, 2, 3]), [-1, 0, 100, 100])
        self.assertEqual(convert([2], heights=[float('nan')], counts=[0]), [100])

    def test_gradual_costs_then_lethal(self):
        costs = convert([1]*5, slope=[0, 3, 10, 19, 21])
        self.assertEqual(costs[:2], [0, 0])
        self.assertTrue(0 < costs[2] < costs[3] < 100)
        self.assertEqual(costs[4], 100)
        self.assertEqual(convert([1, 1], rough=[0.035, 0.06]), [50, 100])

    def test_invalid_or_unsupported_ground_is_unknown(self):
        self.assertEqual(convert([1], heights=[float('nan')]), [-1])
        self.assertEqual(convert([1], rough=[float('inf')]), [-1])
        self.assertEqual(convert([1], slope=[-1]), [-1])
        self.assertEqual(convert([1], counts=[0]), [-1])

    def test_float32_threshold_roundoff(self):
        result = occupancy_values([1, 1], [0., 0.], np.radians([3., 20.]).astype(np.float32),
                                  [0., 0.], [9, 9], [0, 0], 2, CostPolicy())
        self.assertEqual(result, [0, 100])

    def test_row_major_order_is_preserved(self):
        self.assertEqual(convert([1, 0, 2, 1, 3, 0]), [0, -1, 100, 0, 100, -1])

    def test_malformed_lengths_flags_and_thresholds(self):
        with self.assertRaises(ValueError):
            occupancy_values([1], [0], [0], [0], [3], [0], 2, CostPolicy())
        with self.assertRaises(ValueError):
            convert([4])
        with self.assertRaises(ValueError):
            CostPolicy(free_slope_deg=20, lethal_slope_deg=10)
        with self.assertRaises(ValueError):
            CostPolicy(lethal_roughness_m=float('nan'))

    def test_geometry_rejects_invalid_frames_orientation_and_resolution(self):
        info = NS(width=2, height=3, resolution=0.3,
                  origin=NS(position=NS(x=-6., y=-6., z=0.), orientation=NS(x=0., y=0., z=0., w=1.)))
        validate_geometry(info, 'terrain_map')
        for frame in ('', '/terrain_map', 'bad frame'):
            with self.assertRaises(ValueError): validate_geometry(info, frame)
        info.origin.orientation.z = .1
        with self.assertRaises(ValueError): validate_geometry(info, 'terrain_map')
        info.origin.orientation.z = 0.
        info.resolution = float('nan')
        with self.assertRaises(ValueError): validate_geometry(info, 'terrain_map')


class ExpiryTests(unittest.TestCase):
    def test_fresh_data_expires_without_new_scan(self):
        lease = SnapshotLease(2.)
        self.assertTrue(lease.accept(100., 100.1, 1.))
        self.assertTrue(lease.fresh(101., 1.9))
        self.assertFalse(lease.fresh(102.1, 3.1))

    def test_stale_and_future_messages_rejected(self):
        lease = SnapshotLease()
        self.assertFalse(lease.accept(90., 100., 1.))
        self.assertFalse(lease.accept(101., 100., 1.))

    def test_repeats_cannot_refresh_paused_clock(self):
        lease = SnapshotLease()
        self.assertTrue(lease.accept(100., 100., 1.))
        self.assertFalse(lease.accept(100., 100., 2.9))
        self.assertFalse(lease.fresh(100., 3.1))

    def test_clock_reset_invalidates_then_accepts_new_epoch(self):
        lease = SnapshotLease()
        lease.accept(100., 100., 1.)
        self.assertFalse(lease.fresh(0., 1.1))
        self.assertTrue(lease.accept(0.2, 0.2, 1.2))

    def test_out_of_order_observation_cannot_replace_latest(self):
        lease = SnapshotLease()
        lease.accept(100., 100., 1.)
        self.assertFalse(lease.accept(99.9, 100.1, 1.1))
        self.assertEqual(lease.stamp, 100.)


def nav_config():
    data = {}
    for name in ('local_costmap','global_costmap'):
        data[name] = {name: {'ros__parameters': {
            'global_frame':'odom', 'rolling_window':True, 'robot_radius':0.7,
            'plugins':['obstacle_layer','inflation_layer'],
            'obstacle_layer':{'plugin':'nav2_costmap_2d::ObstacleLayer',
                              'scan':{'topic':'/scan','clearing':True}},
            'inflation_layer':{'plugin':'nav2_costmap_2d::InflationLayer','inflation_radius':1.2}}}}
    data['planner_server'] = {'ros__parameters': {'GridBased': {
        'plugin':'nav2_navfn_planner::NavfnPlanner','allow_unknown':True}}}
    return data


class ConfigTests(unittest.TestCase):
    def test_profile_preserves_geometry_and_existing_sensors(self):
        original = nav_config()
        saved = deepcopy(original)
        new = physical_profile(original)
        self.assertEqual(original, saved)
        for name in ('local_costmap','global_costmap'):
            p = new[name][name]['ros__parameters']
            self.assertEqual(p['plugins'], ['terrain_layer','obstacle_layer','inflation_layer'])
            self.assertTrue(p['track_unknown_space'])
            self.assertFalse(p['use_maximum'])
            self.assertFalse(p['use_sim_time'])
            self.assertEqual(p['obstacle_layer']['combination_method'], 2)
            self.assertEqual(p['obstacle_layer']['scan'], {'topic':'/scan','clearing':True})
            self.assertEqual(p['robot_radius'], .7)
        self.assertFalse(new['planner_server']['ros__parameters']['GridBased']['allow_unknown'])

    def test_custom_layers_require_review(self):
        data = nav_config()
        data['local_costmap']['local_costmap']['ros__parameters']['plugins'].append('other_layer')
        with self.assertRaises(ValueError): physical_profile(data)

    def test_simulation_clock_is_explicit(self):
        data = physical_profile(nav_config(), True)
        self.assertTrue(data['local_costmap']['local_costmap']['ros__parameters']['use_sim_time'])
        self.assertTrue(data['planner_server']['ros__parameters']['use_sim_time'])
