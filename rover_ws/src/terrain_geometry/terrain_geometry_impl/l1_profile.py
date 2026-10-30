"""Unitree L1 envelope plus explicit Gazebo approximation choices.

Reference: Unitree L1 User Manual v1.1 (2024.06), printed pp. 6, 9, 14.
The raster below is a stationary one-second-budget snapshot, NOT the hardware
11 Hz / 180 Hz mirror trajectory or an official Unitree simulation plugin.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class L1Profile:
    variant: str = 'PM'
    horizontal_samples: int = 240
    vertical_samples: int = 90
    update_rate: float = 1.0  # Simulation batching choice, not L1 motor speed.
    min_range: float = 0.05
    range_resolution: float = 0.008  # SDF metadata, not enforced quantization.
    range_noise_stddev: float = 0.01  # Chosen test noise, not a vendor sigma.
    mount_roll: float = math.pi  # Native +z hemisphere is directed at ground.
    lidar_frame: str = 'unilidar_lidar'
    imu_frame: str = 'unilidar_imu'
    imu_rate: float = 250.0
    imu_offset: tuple = (-0.007698, -0.014655, 0.00667)

    def __post_init__(self):
        if self.variant not in ('PM', 'RM'):
            raise ValueError('L1 variant must be PM or RM')

    @property
    def max_range(self):
        return 20.0 if self.variant == 'PM' else 30.0

    def rays_sensor_frame(self):
        az, el = np.meshgrid(np.linspace(-math.pi, math.pi, self.horizontal_samples),
                             np.linspace(0., math.pi/2, self.vertical_samples))
        return np.column_stack(((np.cos(el)*np.cos(az)).ravel(),
                                (np.cos(el)*np.sin(az)).ravel(), np.sin(el).ravel()))

    def rays_world_frame(self):
        rays = self.rays_sensor_frame()
        c, s = math.cos(self.mount_roll), math.sin(self.mount_roll)
        return rays @ np.array([[1., 0., 0.], [0., c, -s], [0., s, c]]).T
