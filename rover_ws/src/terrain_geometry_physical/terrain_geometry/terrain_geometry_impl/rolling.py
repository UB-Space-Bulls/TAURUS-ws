"""Position a level, world-oriented window using the scan-time IMU pose."""
from dataclasses import replace
import math


def rolling_config(config, position, clearance):
    if len(position) != 3 or not all(math.isfinite(v) for v in position):
        raise ValueError('tracking position must contain three finite coordinates')
    if not math.isfinite(clearance) or clearance <= 0:
        raise ValueError('body_ground_clearance must be finite and positive')
    if not config.min_z <= -clearance < config.max_z:
        raise ValueError('body-relative min_z/max_z must include ground below body_ground_clearance')
    x, y, z = position
    return replace(config,
        origin_x=math.floor(x / config.resolution) * config.resolution + config.origin_x,
        origin_y=math.floor(y / config.resolution) * config.resolution + config.origin_y,
        seed_x=x + config.seed_x, seed_y=y + config.seed_y,
        seed_height=z - clearance,
        min_z=z + config.min_z, max_z=z + config.max_z)
