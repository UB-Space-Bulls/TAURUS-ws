"""ROS-independent supervision and stream health metrics (Python stdlib only)."""
import json
import math
import os
from pathlib import Path
import selectors
import subprocess
import sys
import tempfile
import time


def finite(x):
    return isinstance(x, (int, float)) and math.isfinite(x)


def valid_imu(m):
    values = [m.get(k, []) for k in ('q', 'gyro', 'accel')]
    if [len(v) for v in values] != [4, 3, 3]:
        return False
    if not all(finite(x) for v in values for x in v):
        return False
    return 0.8 <= math.sqrt(sum(x*x for x in values[0])) <= 1.2


def worker_path(mode):
    name = 'l1_sdk_reader' if mode == 'sdk' else 'l1_wire_reader'
    here = Path(__file__).resolve().parent
    candidates = []
    if os.environ.get('L1_READER_DIR'):
        candidates.append(Path(os.environ['L1_READER_DIR'])/name)
    candidates += [Path(sys.argv[0]).absolute().parent/name, here/name, here.parent/'build'/name]
    try:
        from ament_index_python.packages import get_package_prefix
        candidates.append(Path(get_package_prefix('unitree_l1_driver'))/'lib/unitree_l1_driver'/name)
    except (ImportError, LookupError):
        pass
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    raise FileNotFoundError(f'{name} not found. Run ./build_standalone.sh or colcon build.')


class Worker:
    """A separate process makes even a blocked vendor SDK safely terminable."""
    def __init__(self, command):
        self.log = tempfile.TemporaryFile(mode='w+b')
        try:
            self.process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                            stderr=self.log, start_new_session=True)
        except Exception:
            self.log.close()
            raise
        os.set_blocking(self.process.stdout.fileno(), False)
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.process.stdout, selectors.EVENT_READ)
        self.buffer = b''
        self.malformed = 0
        self.eof = False

    def poll(self, timeout=0.05):
        events = []
        for key, _ in self.selector.select(timeout):
            try:
                chunk = os.read(key.fd, 262144)
            except BlockingIOError:
                continue
            if not chunk:
                self.selector.unregister(key.fileobj)
                self.eof = True
                continue
            self.buffer += chunk
            if len(self.buffer) > 16_000_000:
                raise RuntimeError('Worker exceeded maximum buffered record size')
            while b'\n' in self.buffer:
                line, self.buffer = self.buffer.split(b'\n', 1)
                if not line.startswith(b'L1JSON '):
                    continue  # The vendor library may also write human-readable text.
                try:
                    event = json.loads(line[7:])
                    if not isinstance(event, dict) or 'type' not in event:
                        raise ValueError('Missing type')
                    events.append(event)
                except (ValueError, UnicodeError):
                    self.malformed += 1
        return events

    def log_tail(self):
        self.log.seek(0, os.SEEK_END)
        size = self.log.tell()
        self.log.seek(max(0, size-8192))
        return self.log.read().decode(errors='replace')

    def stop(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1)

    def close(self):
        self.stop()
        self.selector.close()
        self.process.stdout.close()
        self.log.close()


class Stream:
    def __init__(self, start, bits):
        self.start = start
        self.bits = bits
        self.count = 0
        self.last = None
        self.max_gap = 0.0
        self.last_id = None
        self.last_stamp = None
        self.duplicate_ids = 0
        self.id_discontinuities = 0
        self.estimated_missing_ids = 0
        self.stamp_regressions = 0
        self.repeated_stamps = 0
        self.invalid_stamps = 0

    def add(self, event, now):
        self.max_gap = max(self.max_gap, now-(self.last if self.last is not None else self.start))
        self.last = now
        self.count += 1
        seq = event.get('id')
        if isinstance(seq, int):
            if self.last_id is not None:
                delta = (seq-self.last_id) % (1 << self.bits)
                if delta == 0:
                    self.duplicate_ids += 1
                elif delta != 1:
                    self.id_discontinuities += 1
                    if delta < (1 << (self.bits-1)):
                        self.estimated_missing_ids += delta-1
            self.last_id = seq
        stamp = event.get('stamp')
        if not finite(stamp) or stamp <= 0:
            self.invalid_stamps += 1
        else:
            if self.last_stamp is not None:
                self.stamp_regressions += stamp < self.last_stamp
                self.repeated_stamps += stamp == self.last_stamp
            self.last_stamp = stamp

    def report(self, now):
        elapsed = max(now-self.start, 1e-9)
        return {
            'messages': self.count, 'rate_hz': self.count/elapsed,
            'max_gap_s': max(self.max_gap, now-(self.last if self.last is not None else self.start)),
            'age_s': now-(self.last if self.last is not None else self.start),
            'duplicate_ids': self.duplicate_ids,
            'id_discontinuities': self.id_discontinuities,
            'estimated_missing_ids': self.estimated_missing_ids,
            'stamp_regressions': self.stamp_regressions,
            'repeated_stamps': self.repeated_stamps, 'invalid_stamps': self.invalid_stamps,
        }


class Health:
    def __init__(self, start):
        self.start = start
        self.cloud = Stream(start, 32)
        self.imu = Stream(start, 16)  # Wire IMU packet_id is uint16_t.
        self.points = 0
        self.invalid_points = 0
        self.empty_clouds = 0
        self.invalid_imus = 0
        self.last_imu = None
        self.range_min = None
        self.range_max = None
        self.dirty = None

    def add(self, event, now):
        if now < self.start:
            return
        kind = event['type']
        if kind == 'cloud':
            self.cloud.add(event, now)
            self.points += event['valid']
            self.invalid_points += event['invalid']
            self.empty_clouds += event['valid'] == 0
            self.range_min, self.range_max = event.get('range_min'), event.get('range_max')
        elif kind == 'imu':
            self.imu.add(event, now)
            self.invalid_imus += not valid_imu(event)
            self.last_imu = event
        elif kind == 'aux':
            self.dirty = event.get('dirty_percent')

    def report(self, now, min_cloud=5, min_imu=100, max_gap=0.5):
        c, i = self.cloud.report(now), self.imu.report(now)
        checks = {
            'cloud_rate': c['rate_hz'] >= min_cloud,
            'imu_rate': i['rate_hz'] >= min_imu,
            'cloud_continuity': c['messages'] > 0 and c['max_gap_s'] <= max_gap,
            'imu_continuity': i['messages'] > 0 and i['max_gap_s'] <= max_gap,
            'nonempty_clouds': self.points > 0 and self.empty_clouds == 0,
            'finite_clouds': self.invalid_points == 0,
            'valid_imu_payload': i['messages'] > 0 and self.invalid_imus == 0,
            'imu_ids_continuous': i['messages'] > 0 and i['duplicate_ids'] == 0 and i['id_discontinuities'] == 0,
            'advancing_timestamps': all(s['messages'] > 1 and not any(s[k] for k in
                ('stamp_regressions', 'repeated_stamps', 'invalid_stamps')) for s in (c, i)),
        }
        return {'passed': all(checks.values()), 'checks': checks,
                'elapsed_s': max(0, now-self.start), 'cloud': c, 'imu': i,
                'valid_points': self.points, 'invalid_points': self.invalid_points,
                'empty_clouds': self.empty_clouds, 'invalid_imus': self.invalid_imus,
                'last_range_m': [self.range_min, self.range_max], 'last_imu': self.last_imu,
                'dirty_percent': self.dirty,
                'power_verified': False,
                'scope': 'Received data integrity only; supply voltage/current require an external meter.'}


def save_report(path, report):
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name+'.tmp')
    temp.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    temp.replace(path)
