"""Meaningful geometry/unknown-space regression tests; no ROS runtime needed."""
from dataclasses import replace
import math
import unittest
import numpy as np
from terrain_geometry_impl.core import Config, process, preview_costs, transform_xyz


def dense_surface(function):
    x, y = np.meshgrid(np.arange(-2.95, 3, 0.075), np.arange(-2.95, 3, 0.075))
    return np.column_stack((x.ravel(), y.ravel(), function(x, y).ravel()))


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Config(width=20, height=20, origin_x=-3., origin_y=-3.)

    def test_flat(self):
        r = process(dense_surface(lambda x, y: x*0), self.cfg)
        self.assertTrue(np.all(r.status == 1))
        self.assertLess(np.nanmax(r.slope), 1e-6)
        self.assertLess(np.nanmax(r.roughness), 1e-6)

    def test_known_ramp(self):
        r = process(dense_surface(lambda x, y: x*math.tan(math.radians(15))), self.cfg)
        self.assertGreater(np.count_nonzero(r.status & 1), 350)
        self.assertAlmostEqual(float(np.degrees(np.nanmedian(r.slope))), 15., places=4)

    def test_roughness_above_flat(self):
        r = process(dense_surface(lambda x, y: 0.05*np.sin(17*x)*np.cos(15*y)), self.cfg)
        self.assertGreater(np.nanmedian(r.roughness), 0.015)

    def test_box_top_not_ground(self):
        cloud = dense_surface(lambda x, y: np.where((abs(x-1.5) < 0.45) & (abs(y) < 0.45), 0.7, 0.))
        r = process(cloud, self.cfg)
        top = r.points[:, 2] > 0.6
        self.assertFalse(np.any(r.labels[top] == 1))
        self.assertGreater(np.mean(r.labels[top] == 2), 0.9)

    def test_missing_returns_remain_unknown(self):
        cloud = dense_surface(lambda x, y: x*0)
        cloud = cloud[~((abs(cloud[:, 0]) < 0.7) & (abs(cloud[:, 1]) < 0.7))]
        r = process(cloud, self.cfg)
        self.assertEqual(int(r.status[10, 10]), 0)
        self.assertTrue(np.isnan(r.height[10, 10]))
        self.assertEqual(int(preview_costs(r)[10, 10]), -1)

    def test_empty_scan_clears_every_cell(self):
        r = process(np.empty((0, 3)), self.cfg)
        self.assertTrue(np.all(r.status == 0))
        self.assertTrue(np.isnan(r.height).all())

    def test_no_ground_seed_does_not_invent_ground(self):
        r = process(dense_surface(lambda x, y: x*0+1.), self.cfg)
        self.assertTrue(np.all(r.status == 0))
        self.assertTrue(np.all(r.labels == 0))

    def test_one_point_per_cell_unknown(self):
        x, y = np.meshgrid(np.arange(-2.85, 3, 0.3), np.arange(-2.85, 3, 0.3))
        cloud = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
        r = process(cloud, self.cfg)
        self.assertFalse(np.any(r.status & 1))

    def test_collinear_returns_unknown(self):
        x = np.arange(-2.9, 3, 0.01)
        r = process(np.column_stack((x, x*0, x*0)), self.cfg)
        self.assertFalse(np.any(r.status & 1))

    def test_negative_drop_unknown_not_ground(self):
        r = process(dense_surface(lambda x, y: np.where((x > 1.2), -0.7, 0.)), self.cfg)
        dropped = r.points[:, 2] < -0.5
        self.assertTrue(np.all(r.labels[dropped] == 0))

    def test_mixed_cell_preserves_obstacle_priority(self):
        floor = dense_surface(lambda x, y: x*0)
        raised = floor[(floor[:, 0] > 0.1) & (floor[:, 0] < 0.28) & (floor[:, 1] > 0.1) & (floor[:, 1] < 0.28)].copy()
        raised[:, 2] = 0.7
        r = process(np.vstack((floor, raised)), self.cfg)
        self.assertEqual(int(r.status[10, 10]), 3)
        self.assertEqual(int(preview_costs(r)[10, 10]), 100)

    def test_roi_half_open_and_nonfinite(self):
        cloud = np.array([[-3, -3, 0], [3, 0, 0], [0, 3, 0], [np.nan, 0, 0], [0, 0, np.inf]])
        r = process(cloud, self.cfg)
        self.assertEqual(len(r.points), 1)

    def test_scan_has_no_memory(self):
        process(dense_surface(lambda x, y: x*0), self.cfg)
        self.assertTrue(np.all(process(np.empty((0, 3)), self.cfg).status == 0))

    def test_transform_rotation_and_translation(self):
        q = [0, 0, math.sin(math.pi/4), math.cos(math.pi/4)]
        np.testing.assert_allclose(transform_xyz([[1, 0, 0]], [1, 2, 3], q), [[1, 3, 3]], atol=1e-10)

    def test_tilted_sensor_corrected_before_slope(self):
        cloud = dense_surface(lambda x, y: x*0)
        a = math.radians(20)/2
        sensor = transform_xyz(cloud, [0, 0, 0], [0, math.sin(a), 0, math.cos(a)])
        restored = transform_xyz(sensor, [0, 0, 0], [0, -math.sin(a), 0, math.cos(a)])
        r = process(restored, self.cfg)
        self.assertLess(float(np.nanmax(r.slope)), 1e-6)

    def test_parameter_validation(self):
        for kwargs in ({'resolution': 0}, {'width': -1}, {'fit_radius': 0},
                       {'seed_height': float('nan')}, {'obstacle_height': .01}):
            with self.assertRaises(ValueError):
                replace(self.cfg, **kwargs)
        with self.assertRaises(ValueError):
            transform_xyz([[0, 0, 0]], [0, 0, 0], [0, 0, 0, 0])


if __name__ == '__main__':
    unittest.main()
