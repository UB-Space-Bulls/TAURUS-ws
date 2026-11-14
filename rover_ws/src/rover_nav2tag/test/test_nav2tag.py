#!/usr/bin/env python3
"""
test_nav2tag.py
Unit tests and verification for DEIMOS nav2tag module.

Can be run directly via:
    python3 rover_ws/src/rover_nav2tag/test/test_nav2tag.py
or via pytest:
    pytest rover_ws/src/rover_nav2tag/test/test_nav2tag.py
"""

import math
import sys
import os
import unittest

# Ensure the rover_nav2tag package can be imported
current_dir = os.path.dirname(os.path.abspath(__file__))
pkg_dir = os.path.dirname(current_dir)
sys.path.insert(0, pkg_dir)

# Import pure logic functions
from rover_nav2tag.nav2tag_node import (
    parse_tag_frame_id,
    calculate_standoff_pose,
    tag_approach_heading,
)


class TestTagParsing(unittest.TestCase):
    def test_standard_apriltag_frames(self):
        """Test parsing of standard apriltag_ros frames."""
        self.assertEqual(parse_tag_frame_id("tag36h11:5"), 5)
        self.assertEqual(parse_tag_frame_id("tag36h11:3"), 3)
        self.assertEqual(parse_tag_frame_id("tag36h11:0"), 0)
        self.assertEqual(parse_tag_frame_id("tag36h11:105"), 105)

    def test_alternate_delimiters(self):
        """Test underscore, slash, and dash delimiters."""
        self.assertEqual(parse_tag_frame_id("tag36h11_5"), 5)
        self.assertEqual(parse_tag_frame_id("tag36h11-5"), 5)
        self.assertEqual(parse_tag_frame_id("tag36h11/5"), 5)

    def test_generic_tag_frames(self):
        """Test generic tag frame formats."""
        self.assertEqual(parse_tag_frame_id("tag:5"), 5)
        self.assertEqual(parse_tag_frame_id("tag_5"), 5)
        self.assertEqual(parse_tag_frame_id("tag5"), 5)

    def test_non_tag_frames(self):
        """Non-tag frames must return None."""
        self.assertIsNone(parse_tag_frame_id("map"))
        self.assertIsNone(parse_tag_frame_id("odom"))
        self.assertIsNone(parse_tag_frame_id("base_link"))
        self.assertIsNone(parse_tag_frame_id("camera_link"))
        self.assertIsNone(parse_tag_frame_id("tag36h11_unknown"))


class TestTagFilteringLogic(unittest.TestCase):
    """
    Test scenario:
    Requested Tag 5:
      Detect Tag 3 -> Ignore
      Detect Tag 5 -> Use it
    """

    def simulate_find_tag(self, yaml_frames_mock: str, target_tag_id: int):
        detected_tags = []
        for line in yaml_frames_mock.splitlines():
            line = line.strip()
            if line.endswith(":"):
                frame_name = line[:-1]
                tag_id = parse_tag_frame_id(frame_name, "tag36h11")
                if tag_id is not None:
                    detected_tags.append((frame_name, tag_id))

        for frame_name, tag_id in detected_tags:
            if target_tag_id >= 0:
                if tag_id == target_tag_id:
                    return frame_name
            else:
                return frame_name
        return None

    def setUp(self):
        # Simulating TF frames as returned by tf_buffer.all_frames_as_yaml()
        self.sample_tf_yaml = """
  map:
  odom:
  base_link:
  tag36h11:3:
  tag36h11:5:
"""

    def test_filter_selects_tag_5_and_ignores_tag_3(self):
        # When target is Tag 5, Tag 3 must be ignored even though it appears first in TF
        selected = self.simulate_find_tag(self.sample_tf_yaml, target_tag_id=5)
        self.assertEqual(selected, "tag36h11:5")

    def test_filter_selects_tag_3(self):
        # When target is Tag 3, Tag 3 is selected
        selected = self.simulate_find_tag(self.sample_tf_yaml, target_tag_id=3)
        self.assertEqual(selected, "tag36h11:3")

    def test_filter_returns_none_when_tag_not_present(self):
        # When target is Tag 7 (not in TF tree), neither 3 nor 5 is selected
        selected = self.simulate_find_tag(self.sample_tf_yaml, target_tag_id=7)
        self.assertIsNone(selected)

    def test_wildcard_mode(self):
        # When target is -1 (wildcard), the first detected tag is returned
        selected = self.simulate_find_tag(self.sample_tf_yaml, target_tag_id=-1)
        self.assertEqual(selected, "tag36h11:3")


class TestStandoffPoseCalculation(unittest.TestCase):
    def test_standoff_distance_0_5m_horizontal(self):
        """
        Tag at (5.0, 2.0) with yaw = 0.0 (pointing along +X).
        Standoff of 0.5m should yield position (4.5, 2.0).
        Rover orientation should face the tag (facing yaw = 0, along +X).
        """
        tx, ty, yaw = 5.0, 2.0, 0.0
        gx, gy, qz, qw = calculate_standoff_pose(tx, ty, yaw, stop_distance=0.5)

        self.assertAlmostEqual(gx, 4.5, places=5)
        self.assertAlmostEqual(gy, 2.0, places=5)

        # Distance from goal to tag must be exactly 0.5m
        distance = math.hypot(tx - gx, ty - gy)
        self.assertAlmostEqual(distance, 0.5, places=5)

        # Rover at (4.5, 2.0) must point along +X to look at the tag at (5.0, 2.0)
        facing_yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        self.assertAlmostEqual(facing_yaw, 0.0, places=4)

    def test_standoff_distance_0_5m_vertical(self):
        """
        Tag at (1.0, 4.0) with yaw = pi/2 (pointing along +Y).
        Standoff of 0.5m should yield position (1.0, 3.5).
        Rover orientation should face +Y toward the tag.
        """
        tx, ty, yaw = 1.0, 4.0, math.pi / 2.0
        gx, gy, qz, qw = calculate_standoff_pose(tx, ty, yaw, stop_distance=0.5)

        self.assertAlmostEqual(gx, 1.0, places=5)
        self.assertAlmostEqual(gy, 3.5, places=5)

        distance = math.hypot(tx - gx, ty - gy)
        self.assertAlmostEqual(distance, 0.5, places=5)

        # Rover at (1.0, 3.5) must point along +Y to look at the tag at (1.0, 4.0)
        facing_yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        self.assertAlmostEqual(facing_yaw, math.pi / 2.0, places=4)

    def test_standoff_arbitrary_angle(self):
        """
        Tag at (10.0, 10.0) with yaw = pi/4 (45 degrees).
        Distance from standoff pose to tag must equal stop_distance.
        """
        tx, ty, yaw = 10.0, 10.0, math.pi / 4.0
        stop_dist = 0.5
        gx, gy, qz, qw = calculate_standoff_pose(tx, ty, yaw, stop_distance=stop_dist)

        distance = math.hypot(tx - gx, ty - gy)
        self.assertAlmostEqual(distance, stop_dist, places=5)

        # Vector from rover to tag must point at angle pi/4
        rover_to_tag_angle = math.atan2(ty - gy, tx - gx)
        self.assertAlmostEqual(rover_to_tag_angle, yaw, places=5)

        # And the rover's heading must point along that same line (facing the tag)
        facing_yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        self.assertAlmostEqual(facing_yaw, rover_to_tag_angle, places=5)


def quat_from_axes(x_axis, y_axis, z_axis):
    """Quaternion (x, y, z, w) from a rotation matrix given as its column axes."""
    m00, m10, m20 = x_axis
    m01, m11, m21 = y_axis
    m02, m12, m22 = z_axis
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        return ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)


def upright_tag_axes(heading):
    """
    Axes of an upright wall-mounted AprilTag in the map frame.
    AprilTag convention: x right along the face, y down, z INTO the tag.
    `heading` = direction z points = direction the rover drives to approach.
    """
    c, s = math.cos(heading), math.sin(heading)
    z_axis = (c, s, 0.0)
    y_axis = (0.0, 0.0, -1.0)
    x_axis = (s, -c, 0.0)  # y x z = x for a right-handed frame
    return x_axis, y_axis, z_axis


class TestTagApproachHeading(unittest.TestCase):
    """The approach heading must come from the tag's z-axis, not its x-axis."""

    def test_upright_tag_headings(self):
        for heading in [0.0, math.pi / 2, -math.pi / 2, 0.7, -2.5, math.pi - 0.01]:
            qx, qy, qz, qw = quat_from_axes(*upright_tag_axes(heading))
            got = tag_approach_heading(qx, qy, qz, qw)
            self.assertIsNotNone(got)
            diff = math.atan2(math.sin(got - heading), math.cos(got - heading))
            self.assertAlmostEqual(diff, 0.0, places=5, msg=f"heading={heading}")

    def test_flat_tag_returns_none(self):
        """A tag lying flat (z-axis vertical) has no horizontal approach direction."""
        self.assertIsNone(tag_approach_heading(0.0, 0.0, 0.0, 1.0))  # z up
        self.assertIsNone(tag_approach_heading(1.0, 0.0, 0.0, 0.0))  # z down


if __name__ == "__main__":
    print("=" * 60)
    print("Running DEIMOS nav2tag test suite...")
    print("=" * 60)
    suite = unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    if result.wasSuccessful():
        print("\nAll DEIMOS nav2tag tests PASSED successfully!")
        sys.exit(0)
    else:
        print("\nSome tests FAILED.")
        sys.exit(1)
