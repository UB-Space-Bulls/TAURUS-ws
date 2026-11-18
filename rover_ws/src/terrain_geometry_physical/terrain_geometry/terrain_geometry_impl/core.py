"""ROS-independent terrain geometry. All positions and distances are in metres.

This is a deliberately conservative, single-scan demonstration algorithm.
Ground is seeded at a configured known height, grown through observed cells,
and modeled locally. Unconnected surfaces are not automatically obstacles.
"""

from collections import deque
from dataclasses import dataclass, fields
import math

import numpy as np


@dataclass(frozen=True)
class Config:
    resolution: float = 0.3
    width: int = 50
    height: int = 40
    origin_x: float = -6.0
    origin_y: float = -6.0
    min_z: float = -1.0
    max_z: float = 3.0
    min_points: int = 3
    min_fit_points: int = 9
    min_fit_cells: int = 3
    fit_radius: int = 2
    max_samples_per_cell: int = 40
    low_quantile: float = 0.1
    ground_band: float = 0.14
    obstacle_height: float = 0.22
    seed_x: float = 0.0
    seed_y: float = 0.0
    seed_radius: float = 2.4
    seed_height: float = 0.0
    seed_tolerance: float = 0.08
    max_slope_deg: float = 35.0
    step_tolerance: float = 0.03
    min_xy_eigenvalue: float = 0.0025

    def __post_init__(self):
        for f in fields(self):
            if not math.isfinite(getattr(self, f.name)):
                raise ValueError(f'{f.name} must be finite')
        for name in ('width', 'height', 'min_points', 'min_fit_points',
                     'min_fit_cells', 'fit_radius', 'max_samples_per_cell'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or value <= 0:
                raise ValueError(f'{name} must be a positive integer')
        for name in ('resolution', 'ground_band', 'obstacle_height',
                     'seed_radius', 'seed_tolerance', 'min_xy_eigenvalue'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive')
        if not 0 <= self.low_quantile < 0.5:
            raise ValueError('low_quantile must be in [0, 0.5)')
        if not 0 < self.max_slope_deg < 80:
            raise ValueError('max_slope_deg must be in (0, 80)')
        if self.min_z >= self.max_z or self.step_tolerance < 0:
            raise ValueError('invalid height limits or step tolerance')
        if self.obstacle_height <= self.ground_band:
            raise ValueError('obstacle_height must exceed ground_band')
        if self.min_points < 3 or self.min_fit_points < 3:
            raise ValueError('at least three points are required')
        if self.max_samples_per_cell < self.min_points:
            raise ValueError('max_samples_per_cell must be >= min_points')
        if self.width * self.height > 100000:
            raise ValueError('demo grid is limited to 100000 cells')


@dataclass
class Result:
    """Arrays use [y, x]; point labels are 0 unknown, 1 ground, 2 obstacle."""

    points: np.ndarray
    labels: np.ndarray
    height: np.ndarray
    slope: np.ndarray
    roughness: np.ndarray
    ground_count: np.ndarray
    obstacle_count: np.ndarray
    status: np.ndarray


def transform_xyz(points, translation, quaternion):
    """Apply a target-from-source rigid transform; quaternion order is x,y,z,w."""
    q = np.asarray(quaternion, dtype=float)
    t = np.asarray(translation, dtype=float)
    if q.shape != (4,) or t.shape != (3,) or not np.isfinite(t).all():
        raise ValueError('invalid transform shape or translation')
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid quaternion')
    x, y, z, w = q / norm
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    return np.asarray(points, dtype=float) @ rotation.T + t


def _plane(points, center, cfg):
    """Fit z = a*(x-cx) + b*(y-cy) + c with Huber reweighting."""
    xy = points[:, :2] - center
    covariance = np.cov(xy, rowvar=False, bias=True)
    if np.linalg.eigvalsh(covariance)[0] < cfg.min_xy_eigenvalue:
        return None  # Collinear/single-ring support does not determine a plane.
    design = np.column_stack((xy, np.ones(len(points))))
    weights = np.ones(len(points))
    for _ in range(3):
        root = np.sqrt(weights)
        coeff, _, rank, _ = np.linalg.lstsq(
            design * root[:, None], points[:, 2] * root, rcond=None)
        if rank != 3:
            return None
        residual = points[:, 2] - design @ coeff
        scale = max(0.01, 1.4826 * np.median(np.abs(residual - np.median(residual))))
        weights = np.minimum(1.0, 1.5 * scale / np.maximum(np.abs(residual), 1e-9))
    if math.degrees(math.atan(math.hypot(coeff[0], coeff[1]))) > cfg.max_slope_deg:
        return None
    return coeff


def process(points, cfg=None):
    """Process one cloud already transformed into a level, stationary frame.

Only finite points inside the configured ROI are retained in Result.points.
No missing cell is filled and no earlier scan is accumulated.
"""
    cfg = cfg or Config()
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    valid = np.isfinite(points).all(axis=1)
    valid &= (points[:, 0] >= cfg.origin_x)
    valid &= (points[:, 0] < cfg.origin_x + cfg.width * cfg.resolution)
    valid &= (points[:, 1] >= cfg.origin_y)
    valid &= (points[:, 1] < cfg.origin_y + cfg.height * cfg.resolution)
    valid &= (points[:, 2] >= cfg.min_z) & (points[:, 2] <= cfg.max_z)
    points = points[valid]
    shape = (cfg.height, cfg.width)
    n_cells = cfg.width * cfg.height
    terrain_height = np.full(n_cells, np.nan, dtype=np.float32)
    slope = np.full(n_cells, np.nan, dtype=np.float32)
    roughness = np.full(n_cells, np.nan, dtype=np.float32)
    ground_count = np.zeros(n_cells, dtype=np.uint32)
    obstacle_count = np.zeros(n_cells, dtype=np.uint32)
    status = np.zeros(n_cells, dtype=np.uint8)
    labels = np.zeros(len(points), dtype=np.uint8)

    def result():
        return Result(points, labels, terrain_height.reshape(shape),
                      slope.reshape(shape), roughness.reshape(shape),
                      ground_count.reshape(shape), obstacle_count.reshape(shape),
                      status.reshape(shape))

    if len(points) == 0:
        return result()
    ix = np.floor((points[:, 0] - cfg.origin_x) / cfg.resolution).astype(int)
    iy = np.floor((points[:, 1] - cfg.origin_y) / cfg.resolution).astype(int)
    cell_id = iy * cfg.width + ix
    order = np.argsort(cell_id, kind='stable')
    starts = np.r_[0, np.flatnonzero(np.diff(cell_id[order])) + 1]
    groups = np.split(order, starts[1:])
    by_cell = {int(cell_id[g[0]]): g for g in groups}
    centers = np.column_stack((
        cfg.origin_x + (np.arange(n_cells) % cfg.width + 0.5) * cfg.resolution,
        cfg.origin_y + (np.arange(n_cells) // cfg.width + 0.5) * cfg.resolution))
    low_height = np.full(n_cells, np.nan)
    candidates = {}
    for k, indices in by_cell.items():
        if len(indices) < cfg.min_points:
            continue
        z = points[indices, 2]
        low = np.quantile(z, cfg.low_quantile)
        lower = indices[(z >= low - cfg.ground_band) & (z <= low + cfg.ground_band)]
        if len(lower) < cfg.min_points:
            continue
        low_height[k] = np.median(points[lower, 2])
        # Balanced spatial samples prevent near-field cells dominating a fit.
        lower = lower[np.lexsort((points[lower, 1], points[lower, 0]))]
        if len(lower) > cfg.max_samples_per_cell:
            lower = lower[np.linspace(0, len(lower)-1, cfg.max_samples_per_cell).astype(int)]
        candidates[k] = lower

    def neighbours(k, radius=1):
        row, col = divmod(k, cfg.width)
        for y in range(max(0, row-radius), min(cfg.height, row+radius+1)):
            for x in range(max(0, col-radius), min(cfg.width, col+radius+1)):
                yield y * cfg.width + x

    near_seed = np.linalg.norm(centers - [cfg.seed_x, cfg.seed_y], axis=1) <= cfg.seed_radius
    reachable = near_seed & (np.abs(low_height - cfg.seed_height) <= cfg.seed_tolerance)
    queue = deque(np.flatnonzero(reachable).tolist())
    max_gradient = math.tan(math.radians(cfg.max_slope_deg))
    while queue:
        k = queue.popleft()
        for j in neighbours(k):
            if reachable[j] or not np.isfinite(low_height[j]):
                continue
            distance = np.linalg.norm(centers[j] - centers[k])
            if abs(low_height[j] - low_height[k]) <= max_gradient * distance + cfg.step_tolerance:
                reachable[j] = True
                queue.append(j)

    models = {}
    for k in np.flatnonzero(reachable):
        support_cells = [j for j in neighbours(k, cfg.fit_radius) if reachable[j]]
        if len(support_cells) < cfg.min_fit_cells:
            continue
        support = np.concatenate([candidates[j] for j in support_cells])
        if len(support) < cfg.min_fit_points:
            continue
        model = _plane(points[support], centers[k], cfg)
        if model is None:
            continue
        indices = by_cell[k]
        predicted = (points[indices, :2] - centers[k]) @ model[:2] + model[2]
        residual = points[indices, 2] - predicted
        ground = np.abs(residual) <= cfg.ground_band
        if np.count_nonzero(ground) < cfg.min_points:
            continue
        models[k] = model
        terrain_height[k] = model[2]
        slope[k] = math.atan(math.hypot(model[0], model[1]))
        roughness[k] = np.sqrt(np.mean(residual[ground]**2) / (1 + np.sum(model[:2]**2)))
        ground_count[k] = np.count_nonzero(ground)
        labels[indices[ground]] = 1
        status[k] |= 1

    for k, indices in by_cell.items():
        # Extrapolation is bounded to a nearby VALID ground model; it never
        # creates a valid terrain cell or clears unknown space.
        support_models = [j for j in neighbours(k, cfg.fit_radius) if j in models]
        if not support_models:
            continue
        j = k if k in models else min(
            support_models, key=lambda v: np.linalg.norm(centers[k] - centers[v]))
        model = models[j]
        predicted = (points[indices, :2] - centers[j]) @ model[:2] + model[2]
        obstacle = (points[indices, 2] - predicted > cfg.obstacle_height) & (labels[indices] != 1)
        labels[indices[obstacle]] = 2
        obstacle_count[k] = np.count_nonzero(obstacle)
        if obstacle_count[k]:
            status[k] |= 2
    return result()


def preview_costs(result, slope_limit_deg=20.0, roughness_limit=0.06):
    """Diagnostic occupancy-style costs, not a vehicle-calibrated Nav2 layer."""
    costs = np.full(result.status.shape, -1, dtype=np.int8)
    known = (result.status & 1) != 0
    costs[known] = 0
    hazardous = known & ((result.slope > math.radians(slope_limit_deg)) |
                        (result.roughness > roughness_limit))
    costs[hazardous | ((result.status & 2) != 0)] = 100
    return costs
