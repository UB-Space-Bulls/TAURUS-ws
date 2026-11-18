"""ROS-independent TerrainGrid cost conversion and observation expiry."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class CostPolicy:
    free_slope_deg: float = 3.0
    lethal_slope_deg: float = 20.0
    free_roughness_m: float = 0.01
    lethal_roughness_m: float = 0.06

    def __post_init__(self):
        values = (self.free_slope_deg, self.lethal_slope_deg,
                  self.free_roughness_m, self.lethal_roughness_m)
        if not all(math.isfinite(x) for x in values):
            raise ValueError('Terrain thresholds must be finite')
        if not 0 <= self.free_slope_deg < self.lethal_slope_deg < 90:
            raise ValueError('Require 0 <= free_slope_deg < lethal_slope_deg < 90')
        if not 0 <= self.free_roughness_m < self.lethal_roughness_m:
            raise ValueError('Require 0 <= free_roughness_m < lethal_roughness_m')


def occupancy_values(status, ground_height, slope, roughness, ground_count,
                     obstacle_count, cell_count, policy):
    arrays = [np.asarray(x) for x in
              (status, ground_height, slope, roughness, ground_count, obstacle_count)]
    if cell_count <= 0 or any(a.ndim != 1 or len(a) != cell_count for a in arrays):
        raise ValueError('All TerrainGrid arrays must have width*height entries')
    flags, heights, slopes, rough, ground_n, obstacle_n = arrays
    if not np.issubdtype(flags.dtype, np.integer) or np.any((flags < 0) | (flags > 3)):
        raise ValueError('Unknown TerrainGrid status bits')
    for counts in (ground_n, obstacle_n):
        if not np.issubdtype(counts.dtype, np.integer) or np.any(counts < 0):
            raise ValueError('Point counts must be nonnegative integers')
    costs = np.full(cell_count, -1, dtype=np.int8)
    known = ((flags & 1) != 0) & (ground_n > 0)
    known &= np.isfinite(heights) & np.isfinite(slopes) & np.isfinite(rough)
    known &= (slopes >= 0) & (slopes < math.pi / 2) & (rough >= 0)
    s = np.degrees(slopes[known])
    r = rough[known]
    severity = np.maximum(
        (s - policy.free_slope_deg) / (policy.lethal_slope_deg - policy.free_slope_deg),
        (r - policy.free_roughness_m) / (policy.lethal_roughness_m - policy.free_roughness_m))
    # Suppress round-off at thresholds, including float32 ROS message values.
    graded = np.ceil(99 * np.clip(severity, 0, 1) - 1e-6).astype(np.int8)
    graded[severity >= 1 - 1e-6] = 100
    costs[known] = graded
    # Positive obstacle evidence wins even without a ground estimate beneath it.
    costs[(flags & 2) != 0] = 100
    return costs.tolist()


def validate_geometry(info, frame):
    if not frame or frame.startswith('/') or any(c.isspace() for c in frame):
        raise ValueError('Expected a nonempty TF frame without spaces or a leading slash')
    if info.width <= 0 or info.height <= 0:
        raise ValueError('Grid dimensions must be positive')
    p, q = info.origin.position, info.origin.orientation
    values = (info.resolution, p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(x) for x in values) or info.resolution <= 0:
        raise ValueError('Grid metadata must be finite with positive resolution')
    if max(abs(q.x), abs(q.y), abs(q.z), abs(abs(q.w) - 1)) > 1e-6:
        raise ValueError('TerrainGrid v1 requires an axis-aligned grid orientation')


class SnapshotLease:
    """Expire on input timestamp AND steady elapsed time; repeats cannot renew."""
    def __init__(self, max_age=2.0, future_tolerance=0.1):
        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError('max_age must be finite and positive')
        if not math.isfinite(future_tolerance) or future_tolerance < 0:
            raise ValueError('future_tolerance must be finite and nonnegative')
        self.max_age = max_age
        self.future_tolerance = future_tolerance
        self.stamp = self.received = self.last_ros = None

    def _observe_clock(self, now_ros):
        if self.last_ros is not None and now_ros < self.last_ros - self.future_tolerance:
            self.stamp = self.received = None
        self.last_ros = now_ros

    def accept(self, stamp, now_ros, now_steady):
        self._observe_clock(now_ros)
        age = now_ros - stamp
        if not math.isfinite(stamp) or stamp < 0 or not -self.future_tolerance <= age <= self.max_age:
            return False
        if self.stamp is not None and stamp <= self.stamp:
            return False
        self.stamp, self.received = stamp, now_steady
        return True

    def fresh(self, now_ros, now_steady):
        self._observe_clock(now_ros)
        return (self.stamp is not None and
                -self.future_tolerance <= now_ros - self.stamp <= self.max_age and
                0 <= now_steady - self.received <= self.max_age)
