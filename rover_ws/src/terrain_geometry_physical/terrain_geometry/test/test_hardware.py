import unittest
import numpy as np
from terrain_geometry_impl.hardware import finite_xyz, validate_mount
from terrain_geometry_impl.core import transform_xyz, process, Config


class HardwareTests(unittest.TestCase):
    def test_missing_rays_filtered_before_rotation(self):
        cloud = np.array([[1, 2, 1.2], [np.inf, 0, 0], [0, -np.inf, 0], [0, 0, np.nan]])
        with np.errstate(invalid='raise'):
            actual = transform_xyz(finite_xyz(cloud), [0, 0, 1.2], [1, 0, 0, 0])
        np.testing.assert_allclose(actual, [[1, -2, 0]])

    def test_invalid_only_cloud_clears_ground(self):
        clean = finite_xyz(np.array([[np.inf, 0, 0], [np.nan, 1, 0]]))
        with np.errstate(invalid='raise'):
            result = process(transform_xyz(clean, [0, 0, 1], [1, 0, 0, 0]), Config())
        self.assertEqual(clean.shape, (0, 3))
        self.assertFalse(result.status.any())
        self.assertTrue(np.isnan(result.height).all())

    def test_measured_mount(self):
        data = dict(parent_frame='level', child_frame='lidar', x=0, y=0, z=0.8,
                    roll=np.pi, pitch=0, yaw=0)
        self.assertEqual(validate_mount(data, 'level', 'lidar'), [0, 0, .8, np.pi, 0, 0])
        for key in ('z', 'roll'):
            for invalid in (None, float('nan'), float('inf'), True, '0.8'):
                with self.assertRaises(ValueError):
                    validate_mount(dict(data, **{key:invalid}), 'level', 'lidar')
        with self.assertRaises(ValueError):
            validate_mount(data, 'wrong_frame', 'lidar')
        with self.assertRaises(ValueError):
            validate_mount(data, 'lidar', 'lidar')

    def test_bad_xyz_shape(self):
        with self.assertRaises(ValueError):
            finite_xyz(np.zeros((5, 4)))
