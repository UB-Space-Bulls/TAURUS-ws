"""SDK timestamp and timed-cloud contract for the optional FAST-LIO test profile."""
import math
import struct


class SDKClock:
    """Preserve SDK times; reject resets and a clock far from ROS wall time.

    This screens gross clock errors. It does not calibrate sensor latency or
    prove acquisition-time synchronization between the LiDAR and IMU.
    """
    def __init__(self):
        self.last = {}

    def stamp_ns(self, kind, stamp, receipt_ns):
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or stamp <= 0:
            raise ValueError(f'{kind}: invalid SDK timestamp')
        stamp_ns = round(stamp * 1_000_000_000)
        age = (receipt_ns-stamp_ns)/1_000_000_000
        if not -0.05 <= age <= 1.0:
            raise ValueError(f'{kind}: SDK timestamp differs from ROS wall time by '
                             f'{age:.3f}s; check the SDK clock before using FAST-LIO')
        if stamp_ns <= self.last.get(kind, -1):
            raise ValueError(f'{kind}: SDK timestamp repeated/regressed; '
                             'restart the driver and FAST-LIO after fixing the source')
        self.last[kind] = stamp_ns
        return stamp_ns


def timed_cloud(points):
    """Pack seconds-relative-to-SDK-cloud-stamp and UINT16 ring, ordered by time.

    Never infer L1 acquisition times from Velodyne yaw geometry. Reject a cloud
    without positive offsets so FAST-LIO cannot silently enter that fallback.
    SDK-relative offsets are kept, including a nonzero first valid point.
    """
    if not points:
        raise ValueError('FAST-LIO cloud is empty')
    for point in points:
        if len(point) != 6 or not all(
                isinstance(value, (int, float)) and math.isfinite(value)
                for value in point):
            raise ValueError('FAST-LIO cloud has invalid point values')
        if not 0 <= point[4] <= 0.5:
            raise ValueError('L1 point time must be scan-relative seconds in [0, 0.5]')
        if isinstance(point[5], bool) or int(point[5]) != point[5] or not 0 <= point[5] <= 65535:
            raise ValueError('L1 ring cannot be represented as UINT16')
    ordered = sorted(points, key=lambda point: point[4])
    if ordered[-1][4] <= 0:
        raise ValueError('L1 point times are all zero; refusing yaw-derived timing')
    return b''.join(struct.pack('<5fH2x', *point[:5], int(point[5])) for point in ordered)
