"""Cross-check L1 envelope, mounting, generated models and sensor integration."""
from dataclasses import replace
from pathlib import Path
import math
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from terrain_geometry_impl.l1_profile import L1Profile
from terrain_geometry_impl.core import Config, process
from terrain_geometry_impl.demo_scene import raycast_scene, summarize, acceptance

ROOT = Path(__file__).resolve().parents[1]


class L1Tests(unittest.TestCase):
    def test_hemisphere_and_mount(self):
        p = L1Profile()
        self.assertTrue(np.all(p.rays_sensor_frame()[:, 2] >= -1e-10))
        self.assertTrue(np.all(p.rays_world_frame()[:, 2] <= 1e-10))
        np.testing.assert_allclose(np.linalg.norm(p.rays_sensor_frame(), axis=1), 1, atol=1e-10)

    def test_published_rate_budget_not_rotation_rate(self):
        p = L1Profile()
        self.assertEqual(p.horizontal_samples*p.vertical_samples*p.update_rate, 21600)
        self.assertEqual(p.update_rate, 1.0)

    def test_pm_rm_limits(self):
        self.assertEqual(L1Profile('PM').max_range, 20.)
        self.assertEqual(L1Profile('RM').max_range, 30.)
        with self.assertRaises(ValueError):
            L1Profile('L2')

    def test_generated_worlds_match_profile_and_tf(self):
        for variant, name in [('PM', 'terrain_demo.sdf'), ('RM', 'terrain_demo_l1_rm.sdf')]:
            p = L1Profile(variant)
            world = ET.parse(ROOT/'worlds'/name)
            model = world.find(f".//model[@name='stationary_unitree_l1_{variant.lower()}']")
            np.testing.assert_allclose(list(map(float, model.find('pose').text.split())), [0, 0, 1.2, math.pi, 0, 0])
            sensor = model.find(".//sensor[@type='gpu_lidar']")
            self.assertEqual(sensor.find('gz_frame_id').text, p.lidar_frame)
            self.assertEqual(float(sensor.find('update_rate').text), p.update_rate)
            self.assertEqual(float(sensor.find('lidar/range/min').text), p.min_range)
            self.assertEqual(float(sensor.find('lidar/range/max').text), p.max_range)
            self.assertEqual(float(sensor.find('lidar/noise/stddev').text), p.range_noise_stddev)
            self.assertEqual(int(sensor.find('lidar/scan/horizontal/samples').text), p.horizontal_samples)
            self.assertEqual(int(sensor.find('lidar/scan/vertical/samples').text), p.vertical_samples)
            self.assertEqual(float(sensor.find('lidar/scan/vertical/min_angle').text), 0.)
            self.assertAlmostEqual(float(sensor.find('lidar/scan/vertical/max_angle').text), math.pi/2)
            imu = model.find(".//sensor[@type='imu']")
            self.assertEqual(imu.find('gz_frame_id').text, p.imu_frame)
            self.assertEqual(float(imu.find('update_rate').text), 250.)
            np.testing.assert_allclose(list(map(float, imu.find('pose').text.split()))[:3], p.imu_offset)

    def test_l1_noise_and_occlusion_acceptance(self):
        # Multiple seeds reduce dependence on a fortunate synthetic noise draw.
        for seed in (0, 7, 42):
            cloud = raycast_scene(noise_seed=seed)
            metrics = summarize(process(cloud, Config()), Config())
            self.assertTrue(all(acceptance(metrics).values()), (seed, metrics))

    def test_rm_envelope_acceptance(self):
        p = L1Profile('RM')
        metrics = summarize(process(raycast_scene(p), Config()), Config())
        self.assertTrue(all(acceptance(metrics).values()), metrics)

    def test_upright_sensor_cannot_see_flat_ground_below_it(self):
        upright = replace(L1Profile(), mount_roll=0.)
        rays = upright.rays_world_frame()
        self.assertFalse(np.any(rays[:, 2] < -1e-8))


if __name__ == '__main__':
    unittest.main()
