"""Contract tests for the opt-in SDK-time FAST-LIO profile, without hardware."""
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from l1_fastlio import SDKClock, timed_cloud


class FastlioContractTests(unittest.TestCase):
    def test_ring_layout_order_and_time_preservation(self):
        points = [[2, 0, 0, 50, 0.09, 7], [1, 0, 0, 30, 0.01, 3]]
        data = timed_cloud(points)
        self.assertEqual(len(data), 48)
        first, last = struct.unpack_from('<5fH2x', data), struct.unpack_from('<5fH2x', data, 24)
        self.assertEqual(first[0], 1)
        self.assertEqual(first[5], 3)
        self.assertAlmostEqual(first[4], 0.01)
        self.assertAlmostEqual(last[4], 0.09)
        self.assertEqual(points[0][0], 2)  # Input not rewritten/rebased.

    def test_invalid_timing_cannot_trigger_velodyne_fallback(self):
        for offsets in ([], [0, 0], [-0.1, 0.1], [0, 1], [float('nan')]):
            with self.subTest(offsets=offsets), self.assertRaises(ValueError):
                timed_cloud([[1, 0, 0, 10, offset, 0] for offset in offsets])

    def test_ring_narrowing_never_wraps(self):
        for ring in (-1, 65536, 0.5, True):
            with self.subTest(ring=ring), self.assertRaises(ValueError):
                timed_cloud([[1, 0, 0, 10, 0.01, ring]])

    def test_sdk_stamps_preserved_across_interleaved_streams(self):
        clock = SDKClock()
        receipt = 100_200_000_000
        self.assertEqual(clock.stamp_ns('imu', 100.15, receipt), 100_150_000_000)
        self.assertEqual(clock.stamp_ns('cloud', 100.1, receipt), 100_100_000_000)
        self.assertEqual(clock.stamp_ns('imu', 100.154, receipt), 100_154_000_000)

    def test_reset_repeated_missing_and_wrong_epoch_rejected(self):
        clock = SDKClock()
        clock.stamp_ns('imu', 100.0, 100_100_000_000)
        for stamp in (100.0, 99.9, 10.0, 101.0, None, float('nan'), 0.0):
            with self.subTest(stamp=stamp), self.assertRaises(ValueError):
                clock.stamp_ns('imu', stamp, 100_100_000_000)
