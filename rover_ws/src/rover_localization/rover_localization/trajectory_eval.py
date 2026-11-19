"""Score one estimator run offline against Gazebo ground truth.

Usage: ros2 run rover_localization trajectory_eval <bag_dir> [--name NAME]

The bag must hold /odom (Gazebo ground truth, odom -> base_link) and
/Odometry (the estimator, camera_init -> body). Both trajectories are
expressed relative to their first matched pose, exactly like
odometry_comparator, so the numbers are comparable across estimators that
replayed the same input bag.
"""
import argparse
from dataclasses import dataclass
import math

from .pose_math import Sample, errors, interpolate, normalize, relative

JUMP_M = 0.5   # estimate moved this much more than ground truth in one step
GAP_S = 0.5    # no estimate for this long (sim time)


@dataclass
class Report:
    samples: int
    duration_s: float
    rate_hz: float
    path_length_m: float
    rmse_m: float
    max_m: float
    final_m: float
    final_yaw_deg: float
    max_abs_yaw_deg: float
    tracking_failures: int

    def lines(self, name):
        drift = 100 * self.final_m / self.path_length_m if self.path_length_m else float('nan')
        return [f'== {name} ==',
                f'matched estimates      : {self.samples} over {self.duration_s:.1f} s sim '
                f'({self.rate_hz:.1f} Hz)',
                f'ground-truth path      : {self.path_length_m:.2f} m',
                f'translation RMSE       : {self.rmse_m:.4f} m',
                f'translation max error  : {self.max_m:.4f} m',
                f'final error (drift)    : {self.final_m:.4f} m ({drift:.2f} % of path)',
                f'yaw error final / max  : {self.final_yaw_deg:+.3f} / {self.max_abs_yaw_deg:.3f} deg',
                f'tracking failures      : {self.tracking_failures} '
                f'(jumps > {JUMP_M} m or gaps > {GAP_S} s)']


def evaluate(ground: list[Sample], estimates: list[Sample]) -> Report:
    """Pair each estimate with interpolated ground truth and score the run."""
    ground = sorted(ground, key=lambda s: s.stamp)
    pairs, j = [], 0
    for est in sorted(estimates, key=lambda s: s.stamp):
        while j + 1 < len(ground) and ground[j + 1].stamp < est.stamp:
            j += 1
        if j + 1 < len(ground) and ground[j].stamp <= est.stamp <= ground[j + 1].stamp:
            pairs.append((interpolate(ground[j], ground[j + 1], est.stamp), est))
    if len(pairs) < 2:
        raise ValueError(f'only {len(pairs)} estimates overlap ground truth')

    ref_g, ref_e = pairs[0]
    rel = [(relative(ref_g, g), relative(ref_e, e)) for g, e in pairs]
    errs = [errors(g, e) for g, e in rel]
    norms = [n for _, n, _ in errs]
    yaws = [y for _, _, y in errs]

    failures = 0
    for (g0, e0), (g1, e1) in zip(rel, rel[1:]):
        step_err = math.dist(
            [b - a for a, b in zip(g0.position, g1.position)],
            [b - a for a, b in zip(e0.position, e1.position)])
        if step_err > JUMP_M or e1.stamp - e0.stamp > GAP_S:
            failures += 1

    duration = rel[-1][1].stamp - rel[0][1].stamp
    path = sum(math.dist(a.position, b.position) for (a, _), (b, _) in zip(rel, rel[1:]))
    return Report(len(rel), duration, (len(rel) - 1) / duration if duration else 0.0, path,
                  math.sqrt(sum(n * n for n in norms) / len(norms)), max(norms), norms[-1],
                  yaws[-1], max(abs(y) for y in yaws), failures)


def read_bag(path):
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    tracks = {'/odom': [], '/Odometry': []}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in tracks:
            msg = deserialize_message(data, Odometry)
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            tracks[topic].append(Sample(stamp, (p.x, p.y, p.z),
                                        normalize((q.x, q.y, q.z, q.w))))
    return tracks['/odom'], tracks['/Odometry']


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('bag')
    parser.add_argument('--name', default='estimator')
    args = parser.parse_args(argv)
    ground, estimates = read_bag(args.bag)
    print('\n'.join(evaluate(ground, estimates).lines(args.name)))
