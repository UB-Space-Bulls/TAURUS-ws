"""Frame invariance, timestamp interpolation, and quaternion wrap regression tests."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rover_localization.pose_math import Sample, errors, interpolate, relative


def yaw(angle):
    return (0, 0, math.sin(angle/2), math.cos(angle/2))


def test_different_world_origins_and_orientations():
    a = Sample(1, (10, 20, 0), yaw(math.pi/2))
    b = Sample(1, (-5, 3, 8), yaw(0))
    g = relative(a, Sample(2, (10, 22, 0), yaw(math.pi/2)))
    e = relative(b, Sample(2, (-3, 3, 8), yaw(0)))
    delta, norm, heading = errors(g, e)
    assert norm == pytest.approx(0, abs=1e-10)
    assert heading == pytest.approx(0)
    assert g.position == pytest.approx((2, 0, 0), abs=1e-10)


def test_interpolation_and_wrap():
    a = Sample(1, (0, 0, 0), yaw(math.radians(179)))
    b = Sample(3, (2, 4, 6), yaw(math.radians(-179)))
    middle = interpolate(a, b, 2)
    assert middle.position == (1, 2, 3)
    assert abs(middle.rotation[2]) == pytest.approx(1)
    assert errors(a, b)[2] == pytest.approx(2)
    with pytest.raises(ValueError):
        interpolate(a, b, 4)


def test_rotation_is_part_of_relative_translation():
    initial = Sample(1, (1, 2, 3), yaw(math.pi))
    current = Sample(2, (2, 2, 3), yaw(-math.pi))
    motion = relative(initial, current)
    assert motion.position == pytest.approx((-1, 0, 0), abs=1e-10)
    assert errors(Sample(2, (0, 0, 0), yaw(0)), motion)[2] == pytest.approx(0, abs=1e-10)
