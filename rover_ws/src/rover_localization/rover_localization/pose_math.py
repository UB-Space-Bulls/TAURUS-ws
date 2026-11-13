"""Small SE(3) helpers. Quaternions use ROS's (x, y, z, w) order."""
from dataclasses import dataclass
import math

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


def multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def inverse(q: Quaternion) -> Quaternion:
    return (-q[0], -q[1], -q[2], q[3])


def rotate(q: Quaternion, v: Vector3) -> Vector3:
    return multiply(multiply(q, (*v, 0.0)), inverse(q))[:3]


def normalize(q: Quaternion) -> Quaternion:
    length = math.sqrt(sum(x*x for x in q))
    if not math.isfinite(length) or length < 1e-9:
        raise ValueError('Invalid quaternion')
    return tuple(x/length for x in q)


@dataclass(frozen=True)
class Sample:
    stamp: float
    position: Vector3
    rotation: Quaternion


def relative(initial: Sample, current: Sample) -> Sample:
    """inverse(T_initial) * T_current, expressed in the initial body axes."""
    return Sample(current.stamp,
                  rotate(inverse(initial.rotation), tuple(
                      b-a for a, b in zip(initial.position, current.position))),
                  multiply(inverse(initial.rotation), current.rotation))


def interpolate(a: Sample, b: Sample, stamp: float) -> Sample:
    """Linear translation and shortest-arc quaternion SLERP; no extrapolation."""
    if not a.stamp <= stamp <= b.stamp:
        raise ValueError('Timestamp is not bracketed')
    u = (stamp-a.stamp)/(b.stamp-a.stamp) if b.stamp != a.stamp else 0.0
    q = b.rotation
    dot = sum(x*y for x, y in zip(a.rotation, q))
    if dot < 0:
        q, dot = tuple(-x for x in q), -dot
    if dot > 0.9995:
        rotation = normalize(tuple((1-u)*x+u*y for x, y in zip(a.rotation, q)))
    else:
        angle = math.acos(min(1.0, dot))
        rotation = tuple((math.sin((1-u)*angle)*x+math.sin(u*angle)*y)
                         / math.sin(angle) for x, y in zip(a.rotation, q))
    return Sample(stamp, tuple((1-u)*x+u*y for x, y in zip(a.position, b.position)),
                  rotation)


def errors(ground: Sample, estimate: Sample) -> tuple[Vector3, float, float]:
    delta = tuple(b-a for a, b in zip(ground.position, estimate.position))
    # Heading of relative orientation error, normalized by atan2.
    x, y, z, w = multiply(inverse(ground.rotation), estimate.rotation)
    yaw = math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
    return delta, math.sqrt(sum(x*x for x in delta)), yaw
