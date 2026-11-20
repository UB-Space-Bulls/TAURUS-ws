import math

import pytest

from rover_localization.pose_math import Sample
from rover_localization.trajectory_eval import evaluate

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def line(n, dt, speed, offset=(0.0, 0.0, 0.0), t0=0.0):
    return [Sample(t0 + i * dt, (offset[0] + speed * i * dt, offset[1], offset[2]), IDENTITY)
            for i in range(n)]


def test_perfect_estimate_in_other_frame_scores_zero():
    ground = line(101, 0.01, 1.0)                      # 1 m in 1 s at 100 Hz
    estimate = line(11, 0.1, 1.0, offset=(5, 5, 5))     # 10 Hz, different origin
    report = evaluate(ground, estimate)
    assert report.samples == 11
    assert report.rmse_m == pytest.approx(0, abs=1e-9)
    assert report.path_length_m == pytest.approx(1.0)
    assert report.rate_hz == pytest.approx(10.0)
    assert report.tracking_failures == 0


def test_scale_error_and_jump_and_gap_are_counted():
    ground = line(301, 0.01, 1.0)
    estimate = line(11, 0.1, 1.1)                      # 10 % too fast
    estimate[6] = Sample(0.6, (3.0, 0.0, 0.0), IDENTITY)  # jump out and back = 2 failures
    estimate.append(Sample(2.0, (2.2, 0.0, 0.0), IDENTITY))  # 1.0 s gap = 1 failure
    report = evaluate(ground, estimate)
    assert report.final_m == pytest.approx(0.2)
    assert report.max_m == pytest.approx(3.0 - 0.6)
    assert report.tracking_failures == 3


def test_yaw_error_reported_in_degrees():
    ground = line(101, 0.01, 0.0)
    half = math.radians(5) / 2
    estimate = [Sample(s.stamp, s.position, (0, 0, math.sin(half), math.cos(half)))
                for s in line(11, 0.1, 0.0)]
    estimate[0] = Sample(0.0, (0, 0, 0), IDENTITY)
    report = evaluate(ground, estimate)
    assert report.final_yaw_deg == pytest.approx(5.0)


def test_no_overlap_is_an_error():
    with pytest.raises(ValueError, match='overlap'):
        evaluate(line(10, 0.01, 1.0), line(5, 0.1, 1.0, t0=100))
