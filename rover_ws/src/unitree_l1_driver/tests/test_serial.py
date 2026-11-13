"""Synthetic wire fixtures through a Linux PTY; these are NOT hardware tests."""
import json
import os
from pathlib import Path
import pty
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
BIN = Path(os.environ.get('L1_READER_DIR', ROOT/'build'))
sys.path.insert(0, str(ROOT/'scripts'))
from l1_common import Worker


def frame(msgid, payload, seq=0):
    extra = {16: 74, 17: 99, 19: 110}[msgid]
    body = bytes([len(payload), seq % 256, 1, 1, msgid])+payload
    crc = 0xffff
    for byte in body+bytes([extra]):
        tmp = byte ^ (crc & 0xff)
        tmp ^= (tmp << 4) & 0xff
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff
    return b'\xfe'+body+struct.pack('<H', crc)


def packets(seq):
    aux = bytearray(209)
    struct.pack_into('<5I', aux, 0, 0, 10+seq//180, (seq % 180)*5555, 100000, 5555)
    # angle/calibration fields; all ranges are two metres, with known nonzero reflectivity.
    struct.pack_into('<4f', aux, 20, float(seq % 360), 1.0, 0.0, 0.01)
    struct.pack_into('<HHB', aux, 84, seq % 65536, 120, 1)
    aux[89:] = bytes([50])*120
    dist = struct.pack('<HHH', seq % 65536, 1, 240)+struct.pack('<120H', *([2000]*120))
    # Wire is w,x,y,z; SDK returns x,y,z,w.
    imu = struct.pack('<10fH', 1, 0, 0, 0, 0, 0, 0, 0, 0, 9.81, seq % 65536)
    return frame(17, aux, seq*3)+frame(16, dist, seq*3+1)+frame(19, imu, seq*3+2)


@unittest.skipUnless((BIN/'l1_sdk_reader').exists(), 'Build readers first')
class SerialTests(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.port = os.ttyname(self.slave)
        os.set_blocking(self.master, False)
        self.worker = None

    def tearDown(self):
        if self.worker:
            self.worker.close()
        os.close(self.master)
        os.close(self.slave)

    def start(self, mode):
        args = [str(BIN/f'l1_{mode}_reader'), '--port', self.port]
        if mode == 'sdk':
            args += ['--scans', '18', '--points']
        self.worker = Worker(args)
        deadline = time.monotonic()+4
        while time.monotonic() < deadline:
            if any(e['type'] == 'ready' for e in self.worker.poll(.02)):
                return
            if self.worker.process.poll() is not None:
                self.fail(self.worker.log_tail())
        self.fail('Worker did not initialize')

    def test_wire_valid_and_corrupt_frames(self):
        self.start('wire')
        bad = bytearray(frame(19, struct.pack('<10fH', 0, 0, 0, 1, 0, 0, 0, 0, 0, 9.81, 99)))
        bad[-1] ^= 0x55
        os.write(self.master, packets(1)+packets(2)+bad)
        deadline = time.monotonic()+2
        result = None
        while time.monotonic() < deadline and result is None:
            for e in self.worker.poll(.05):
                if e['type'] == 'wire':
                    result = e
        self.assertIsNotNone(result)
        self.assertEqual(result['frames'], 6)
        self.assertEqual(result['imu_frames'], 2)
        self.assertEqual(result['bad_crc'], 1)

    def test_sdk_decodes_cloud_and_imu_from_fixture(self):
        self.start('sdk')
        kinds = set()
        for n in range(1, 100):
            os.write(self.master, packets(n))
            for e in self.worker.poll(.005):
                kinds.add(e['type'])
                if e['type'] == 'cloud':
                    self.assertGreater(e['valid'], 0)
                    self.assertEqual(len(e['points']), e['valid'])
                if e['type'] == 'imu':
                    self.assertEqual(e['q'], [0, 0, 0, 1])
            if {'cloud', 'imu'} <= kinds:
                break
        self.assertTrue({'cloud', 'imu'} <= kinds, f'Observed only {kinds}')

    def test_silent_real_sdk_times_out_and_saves_failure(self):
        with tempfile.TemporaryDirectory() as temp:
            report = Path(temp)/'report.json'
            result = subprocess.run([sys.executable, str(ROOT/'scripts/l1_check.py'),
                '--port', self.port, '--warmup', '.1', '--seconds', '1',
                '--stall-timeout', '.2', '--startup-timeout', '1', '--report', str(report)],
                capture_output=True, timeout=4)
            self.assertEqual(result.returncode, 1, result.stderr.decode())
            r = json.loads(report.read_text())
            self.assertFalse(r['passed'])
            self.assertFalse(r['checks']['full_duration'])

    def test_full_sdk_cli_with_live_synthetic_packets(self):
        stop = threading.Event()
        def feed():
            n = 0
            while not stop.is_set():
                n += 1
                try:
                    os.write(self.master, packets(n))
                    # Drain commands/initial PTY echo so fixture IO cannot fill.
                    try:
                        os.read(self.master, 8192)
                    except BlockingIOError:
                        pass
                except BlockingIOError:
                    pass
                stop.wait(.005)
        thread = threading.Thread(target=feed, daemon=True)
        thread.start()
        try:
            with tempfile.TemporaryDirectory() as temp:
                report = Path(temp)/'report.json'
                result = subprocess.run([sys.executable, str(ROOT/'scripts/l1_check.py'),
                    '--port', self.port, '--warmup', '.3', '--seconds', '1.5',
                    '--report', str(report)], capture_output=True, timeout=5)
                r = json.loads(report.read_text())
                self.assertEqual(result.returncode, 0, json.dumps(r, indent=2))
                self.assertTrue(r['passed'])
                self.assertFalse(r['power_verified'])
                self.assertEqual(r['imu']['repeated_stamps'], 0)
        finally:
            stop.set()
            thread.join(timeout=1)

    def test_two_package_readers_cannot_open_same_port(self):
        self.start('wire')
        result = subprocess.run([str(BIN/'l1_sdk_reader'), '--port', self.port],
                                capture_output=True, timeout=2)
        self.assertEqual(result.returncode, 2)
        self.assertIn(b'Port in use', result.stderr)


if __name__ == '__main__':
    unittest.main()
