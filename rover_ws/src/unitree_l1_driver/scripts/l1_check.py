#!/usr/bin/env python3
"""Bounded direct SDK or passive wire test. No ROS dependency."""
import argparse
from datetime import datetime, timezone
import sys
import time
from l1_common import Health, Worker, save_report, worker_path


def positive(value):
    v = float(value)
    if not 0 < v < 86401:
        raise argparse.ArgumentTypeError('must be finite, > 0 and <= 86400')
    return v


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='/dev/ttyUSB0')
    p.add_argument('--mode', choices=['sdk', 'wire'], default='sdk')
    p.add_argument('--seconds', type=positive, default=60)
    p.add_argument('--warmup', type=positive, default=10)
    p.add_argument('--startup-timeout', type=positive, default=10)
    p.add_argument('--stall-timeout', type=positive, default=3)
    p.add_argument('--scans', type=int, choices=range(1, 181), default=18, metavar='1..180')
    p.add_argument('--min-cloud-hz', type=positive, default=5)
    p.add_argument('--min-imu-hz', type=positive, default=100)
    p.add_argument('--max-gap', type=positive, default=0.5)
    p.add_argument('--passive', action='store_true', help='SDK: do not request NORMAL mode')
    p.add_argument('--report', default='l1_report.json')
    args = p.parse_args()
    worker = None
    health = None
    wire = None
    ready = None
    completed = False
    failure = None
    report = {'passed': False, 'mode': args.mode, 'port': args.port,
              'created_utc': datetime.now(timezone.utc).isoformat(),
              'settings': vars(args), 'power_verified': False}
    try:
        command = [worker_path(args.mode), '--port', args.port]
        if args.mode == 'sdk':
            command += ['--scans', str(args.scans)]
            if args.passive:
                command += ['--passive']
        worker = Worker(command)
        opened = time.monotonic()
        last_print = opened
        print(f'{args.mode.upper()} test on {args.port}; waiting for port initialization.', flush=True)
        while True:
            events = worker.poll(0.02)
            now = time.monotonic()
            for event in events:
                if event['type'] == 'ready' and ready is None:
                    ready = now
                    # Wire metrics cover the entire run, including initialization boundary.
                    health = Health(ready+args.warmup)
                    print(f'Port initialized. Warmup {args.warmup:g}s, measurement {args.seconds:g}s.', flush=True)
                if health:
                    health.add(event, now)
                if event['type'] == 'wire':
                    wire = event
            if worker.process.poll() is not None:
                raise RuntimeError(f'Reader exited with code {worker.process.returncode}')
            if ready is None:
                if now-opened > args.startup_timeout:
                    raise RuntimeError('Timed out opening/initializing the reader')
                continue
            start = ready+args.warmup
            if now >= start+args.seconds:
                completed = True
                break
            if args.mode == 'sdk' and now >= start:
                for name, stream in [('point cloud', health.cloud), ('IMU', health.imu)]:
                    if now-(stream.last if stream.last is not None else start) > args.stall_timeout:
                        raise RuntimeError(f'{name} stalled/missing for > {args.stall_timeout:g}s')
            if now-last_print >= 1:
                last_print = now
                if now < start:
                    print(f'Warming up: {start-now:.0f}s remaining', flush=True)
                elif args.mode == 'sdk':
                    r = health.report(now, args.min_cloud_hz, args.min_imu_hz, args.max_gap)
                    print(f"cloud {r['cloud']['rate_hz']:.1f} Hz | IMU {r['imu']['rate_hz']:.1f} Hz | "
                          f"points {r['valid_points']} | max gaps {r['cloud']['max_gap_s']:.3f}/"
                          f"{r['imu']['max_gap_s']:.3f}s", flush=True)
                elif wire:
                    print(f"bytes {wire['bytes']} | valid frames {wire['frames']} | bad CRC {wire['bad_crc']}", flush=True)
    except KeyboardInterrupt:
        failure = 'Interrupted before the requested observation completed'
    except Exception as exc:
        failure = str(exc)
    finally:
        end = time.monotonic()
        if args.mode == 'sdk' and health:
            report.update(health.report(end, args.min_cloud_hz, args.min_imu_hz, args.max_gap))
        elif args.mode == 'wire':
            checks = {
                'bytes_received': bool(wire and wire['bytes'] > 0),
                'range_packets': bool(wire and wire['range_frames'] > 0),
                'imu_packets': bool(wire and wire['imu_frames'] > 0),
                'auxiliary_packets': bool(wire and wire['aux_frames'] > 0),
                'no_observed_bad_crc': bool(wire and wire['bad_crc'] == 0),
                'byte_continuity': bool(wire and wire['max_byte_gap'] <= args.max_gap),
            }
            report.update(checks=checks, wire=wire, passed=all(checks.values()),
                          scope='Passive wire evidence only; run SDK test for usable cloud/IMU validation.')
        report.setdefault('checks', {})['full_duration'] = completed
        report['failure'] = failure
        if worker:
            worker.stop()
            report['worker_log'] = worker.log_tail()
            report['malformed_records'] = worker.malformed
            report['checks']['worker_records_valid'] = worker.malformed == 0
            worker.close()
        report['passed'] = bool(report.get('passed') and completed and not failure
                                and all(report['checks'].values()))
        try:
            save_report(args.report, report)
        except OSError as exc:
            print(f'Cannot save report: {exc}', file=sys.stderr)
            return 2
    print(('PASS' if report['passed'] else 'FAIL')+f' — {args.report}')
    if failure:
        print(failure, file=sys.stderr)
    if not report['passed'] and report.get('worker_log'):
        print(report['worker_log'], file=sys.stderr)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
