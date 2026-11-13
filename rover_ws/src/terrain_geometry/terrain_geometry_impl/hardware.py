"""ROS-independent validation for real sensor input and measured mounting."""
import math
import numpy as np


def finite_xyz(points):
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('XYZ input must have shape (N, 3)')
    return points[np.isfinite(points).all(axis=1)]


def validate_mount(data, target_frame, lidar_frame):
    if not isinstance(data, dict):
        raise ValueError('Mount file must contain a YAML mapping')
    if not target_frame or not lidar_frame or target_frame == lidar_frame:
        raise ValueError('Target must be a distinct gravity-aligned frame')
    if data.get('parent_frame') != target_frame or data.get('child_frame') != lidar_frame:
        raise ValueError('Mount parent/child must match target_frame/lidar_frame')
    values = []
    for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        value = data.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'Mount {key} must be a measured finite number (metres/radians)')
        values.append(float(value))
    return values
