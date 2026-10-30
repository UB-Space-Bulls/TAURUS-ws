"""Exact demo surfaces and an offline ray caster (not a Gazebo substitute)."""
import math
import numpy as np
from .l1_profile import L1Profile

RAMP_DEG = 15.0
LIDAR_POSITION = np.array([0.0, 0.0, 1.2])


def ramp_mesh():
    h = 4.0 * math.tan(math.radians(RAMP_DEG))
    vertices = np.array([[1.2, 1.2, 0.0], [5.2, 1.2, 0.0], [5.2, 3.6, 0.0],
                         [1.2, 3.6, 0.0], [5.2, 1.2, h], [5.2, 3.6, h]])
    faces = np.array([[0, 4, 5], [0, 5, 3], [0, 1, 4], [3, 5, 2],
                      [1, 2, 5], [1, 5, 4], [0, 3, 2], [0, 2, 1]])
    return vertices, faces


def rough_mesh():
    x = np.linspace(1.2, 5.2, 41)
    y = np.linspace(-4.2, -1.4, 29)
    xx, yy = np.meshgrid(x, y)
    edge = np.minimum.reduce((xx-1.2, 5.2-xx, yy+4.2, -1.4-yy))
    taper = np.clip(edge / 0.35, 0, 1)
    zz = 0.002 + taper * (0.065 + 0.05 * np.sin(2*np.pi*xx/0.55) * np.cos(2*np.pi*yy/0.65))
    vertices = np.column_stack((xx.ravel(), yy.ravel(), zz.ravel()))
    faces = []
    for row in range(len(y)-1):
        for col in range(len(x)-1):
            a = row*len(x) + col
            faces.extend(((a, a+1, a+len(x)+1), (a, a+len(x)+1, a+len(x))))
    return vertices, np.array(faces)


def box_mesh():
    vertices = np.array([[x, y, z] for z in (0., 0.7)
                         for y in (-0.5, 0.5) for x in (3.05, 3.95)])
    faces = np.array([[4, 5, 7], [4, 7, 6], [0, 2, 3], [0, 3, 1],
                      [0, 1, 5], [0, 5, 4], [2, 6, 7], [2, 7, 3],
                      [0, 4, 6], [0, 6, 2], [1, 3, 7], [1, 7, 5]])
    return vertices, faces


def raycast_scene(profile=None, noise_seed=42):
    """First-return L1-envelope raster, with documented synthetic range noise."""
    profile = profile or L1Profile()
    rays = profile.rays_world_frame()
    distance = np.full(len(rays), np.inf)
    downward = rays[:, 2] < -1e-8
    distance[downward] = -LIDAR_POSITION[2] / rays[downward, 2]
    # The Gazebo floor is a finite 30 x 30 m box, not an infinite plane.
    floor_xy = LIDAR_POSITION[:2] + rays[downward, :2] * distance[downward, None]
    floor_indices = np.flatnonzero(downward)
    distance[floor_indices[np.any(np.abs(floor_xy) > 15., axis=1)]] = np.inf
    # Moller-Trumbore intersection; closest surface wins, including occlusion.
    for make_mesh in (ramp_mesh, rough_mesh, box_mesh):
        vertices, faces = make_mesh()
        for face in faces:
            v0, v1, v2 = vertices[face]
            e1, e2 = v1-v0, v2-v0
            h = np.cross(rays, e2)
            det = np.einsum('ij,j->i', h, e1)
            ok = np.abs(det) > 1e-9
            inv = np.zeros_like(det)
            inv[ok] = 1 / det[ok]
            offset = LIDAR_POSITION-v0
            u = inv * np.einsum('ij,j->i', h, offset)
            q = np.cross(offset, e1)
            v = inv * (rays @ q)
            t = inv * np.dot(e2, q)
            hit = ok & (u >= 0) & (v >= 0) & (u+v <= 1) & (t >= profile.min_range)
            distance[hit] = np.minimum(distance[hit], t[hit])
    valid = np.isfinite(distance) & (distance >= profile.min_range) & (distance <= profile.max_range)
    # Perturb range along each beam, not world z. Gazebo uses a different RNG.
    rng = np.random.default_rng(noise_seed)
    distance[valid] += rng.normal(0., profile.range_noise_stddev, np.count_nonzero(valid))
    valid &= (distance >= profile.min_range) & (distance <= profile.max_range)
    return LIDAR_POSITION + rays[valid] * distance[valid, None]


def summarize(result, cfg):
    """Fixed interior regions exclude the ramp boundaries and edge blends."""
    y, x = np.indices(result.status.shape)
    x = cfg.origin_x + (x + 0.5)*cfg.resolution
    y = cfg.origin_y + (y + 0.5)*cfg.resolution
    valid = (result.status & 1) != 0
    regions = {
        'flat': (x > -3.5) & (x < -1.5) & (abs(y) < 1.5),
        'ramp': (x > 2.1) & (x < 4.5) & (y > 1.95) & (y < 2.85),
        'rough': (x > 1.8) & (x < 4.4) & (y > -3.6) & (y < -2.0),
    }
    metrics = {}
    for name, mask in regions.items():
        known = mask & valid
        metrics[name] = {
            'known_cells': int(np.count_nonzero(known)),
            'region_cells': int(np.count_nonzero(mask)),
            'slope_median_deg': float(np.degrees(np.median(result.slope[known]))) if known.any() else None,
            'roughness_median_m': float(np.median(result.roughness[known])) if known.any() else None,
        }
    obstacle = (x > 2.9) & (x < 4.1) & (abs(y) < 0.65)
    metrics['box_obstacle_cells'] = int(np.count_nonzero(obstacle & ((result.status & 2) != 0)))
    metrics['unknown_cells'] = int(np.count_nonzero(~valid))
    return metrics


def acceptance(metrics):
    flat, ramp, rough = (metrics[k] for k in ('flat', 'ramp', 'rough'))
    return {
        'flat_near_level': flat['known_cells'] >= 6 and flat['slope_median_deg'] < 2.0,
        'ramp_within_2_deg': ramp['known_cells'] >= 6 and abs(ramp['slope_median_deg']-RAMP_DEG) <= 2.0,
        'rough_above_flat': rough['known_cells'] >= 6 and flat['known_cells'] >= 6
        and rough['roughness_median_m'] > flat['roughness_median_m'] + 0.008,
        'raised_obstacle': metrics['box_obstacle_cells'] >= 2,
        'unknown_preserved': metrics['unknown_cells'] > 0,
    }
