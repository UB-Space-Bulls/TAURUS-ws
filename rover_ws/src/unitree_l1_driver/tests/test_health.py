import math
from pathlib import Path
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from l1_common import Health, Stream, Worker, valid_imu


def imu(seq, stamp):
    return {'type': 'imu', 'id': seq, 'stamp': stamp, 'q': [0, 0, 0, 1],
            'gyro': [0, 0, 0], 'accel': [0, 0, 9.81]}


class HealthTests(unittest.TestCase):
    def healthy(self):
        h = Health(0)
        for n in range(1, 501):
            t = n/250
            h.add(imu(n, 100+t), t)
            if n % 25 == 0:
                h.add({'type': 'cloud', 'id': n//25, 'stamp': 100+t,
                       'valid': 100, 'invalid': 0}, t)
        return h

    def test_healthy_stream_passes_without_claiming_power(self):
        r = self.healthy().report(2)
        self.assertTrue(r['passed'])
        self.assertFalse(r['power_verified'])
        self.assertAlmostEqual(r['imu']['rate_hz'], 250)

    def test_end_of_run_dropout_fails_even_with_high_average_rate(self):
        r = self.healthy().report(2.8)
        self.assertFalse(r['passed'])
        self.assertTrue(r['checks']['imu_rate'])
        self.assertFalse(r['checks']['imu_continuity'])

    def test_empty_and_nonfinite_clouds_fail(self):
        h = self.healthy()
        h.add({'type': 'cloud', 'id': 21, 'stamp': 102.01, 'valid': 0, 'invalid': 1}, 2.01)
        r = h.report(2.01)
        self.assertFalse(r['checks']['finite_clouds'])
        self.assertFalse(r['checks']['nonempty_clouds'])

    def test_imu_only_cannot_pass(self):
        h = Health(0)
        for n in range(1, 251):
            h.add(imu(n, n+1), n/250)
        self.assertFalse(h.report(1)['passed'])

    def test_invalid_and_zero_quaternion(self):
        e = imu(1, 2)
        e['q'] = [0, 0, 0, 0]
        self.assertFalse(valid_imu(e))
        e['q'] = [0, 0, 0, 1]
        e['gyro'][0] = None
        self.assertFalse(valid_imu(e))

    def test_warmup_is_excluded(self):
        h = Health(10)
        h.add(imu(1, 1), 5)
        self.assertEqual(h.imu.count, 0)

    def test_uint16_wrap_is_not_packet_loss(self):
        s = Stream(0, 16)
        for n, seq in enumerate([65534, 65535, 0, 1]):
            s.add(imu(seq, n+1), (n+1)/250)
        self.assertEqual(s.id_discontinuities, 0)

    def test_missing_id_and_stamp_reset_recorded(self):
        s = Stream(0, 16)
        s.add(imu(5, 20), .01)
        s.add(imu(8, 10), .02)
        self.assertEqual(s.estimated_missing_ids, 2)
        self.assertEqual(s.stamp_regressions, 1)

    def test_blocked_child_can_be_stopped(self):
        w = Worker([sys.executable, '-c', 'import time; time.sleep(60)'])
        t = time.monotonic()
        try:
            self.assertEqual(w.poll(.02), [])
            w.stop()
            self.assertLess(time.monotonic()-t, 2)
            self.assertIsNotNone(w.process.poll())
        finally:
            w.close()

    def test_malformed_json_cannot_disappear(self):
        w = Worker([sys.executable, '-c', 'print(\'L1JSON {bad}\'); print(\'vendor log\')'])
        try:
            deadline = time.monotonic()+2
            while not w.eof and time.monotonic() < deadline:
                w.poll(.02)
            self.assertEqual(w.malformed, 1)
        finally:
            w.close()
