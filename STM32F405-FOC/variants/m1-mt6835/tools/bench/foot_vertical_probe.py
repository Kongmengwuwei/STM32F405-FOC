"""Supported x=0 vertical-path commissioning; default reads only, --drive moves.

Requires the operator's physical-path confirmation. Never resets, homes, clears
faults or changes calibration. Reads the geometry from the running controller.
"""
import argparse
import csv
import json
import math
import statistics
import struct
import time
from pathlib import Path

import serial
from fivebar_point_probe import Probe
from fivebar_tool import Frames, Solver, decode_geometry, geometry_from_dict, read_group


class VerticalProbe(Probe):
    def pump(self):
        now = time.monotonic()
        if self.target and now - self.sent >= .05:
            self.send(self.target)
            self.sent = now
        for row in self.frames.feed(self.link.read(8192)):
            self.last = row
            self.rows.append((time.monotonic(), row))
            if row[20] != self.sequence:
                self.sequence = row[20]
                self.progress = time.monotonic()
            if self.driving:
                assert row[13] == 0, f"fault {row[13]}"
                assert int(row[14]) & 257 == 257, "position/acquisition lost"
                assert all(-1 <= row[i] <= 111 for i in (0, 1)), "joint excursion"
                # Calibrated x=0 is not q0=q1. Supervise each planned reference.
                assert max(abs(row[i] - row[i+2]) for i in (0, 1)) < 8, "tracking deviation"
                assert max(abs(row[i]) for i in (8, 9)) < 11, "current"
                assert 10 <= row[10] <= 14, "bus voltage"
                assert row[15] == 0, f"command rejected {row[15]}"
                if row[11] == row[12] == 4:
                    self.started = True
                if self.started:
                    assert row[11] in (1, 4) and row[12] in (1, 4), "external stop"
        if self.driving:
            assert time.monotonic() - self.progress < .25, "stale acquisition"


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    return value


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', default='COM8')
    ap.add_argument('--drive', action='store_true')
    ap.add_argument('--max-rpm', type=int, choices=(10, 45, 60, 90, 120), default=120)
    ap.add_argument('--output', type=Path, default=Path('build/dual-leg/wheel-leg-ui/vertical-probe-20261016.json'))
    args = ap.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    result = dict(success=False, drive_requested=args.drive, segments=[], stop_verified=False)
    with serial.Serial(args.port, 1000000, timeout=.015) as link:
        link.dtr = True
        geometry = decode_geometry(read_group(link, Frames(), 4))
        result['geometry'] = geometry
        solver = Solver('C:/mingw64/bin/gcc.exe')
        solver.geometry = geometry_from_dict(geometry['geometry'])
        p = VerticalProbe(link)
        p.send('send 5')
        p.wait(1)
        try:
            assert p.last and p.last[11:14] == (0, 0, 0), 'Not idle/fault-free'
            assert int(p.last[14]) & 287 == 287, 'Position/calibration/acquisition not ready'
            assert geometry['calibrated'], 'Missing geometry calibration'
            assert max(abs(p.last[21+i] - solver.geometry.zero_ccw[i]) for i in (0, 1)) < .01, 'Geometry changed'
            result['initial'] = p.last
            if not args.drive:
                result['success'] = True
                return
            assert all(0 <= p.last[i] <= 108 for i in (0, 1)), 'Start inside supported legal workspace'
            assert max(abs(row[i]) for _, row in p.rows for i in (8, 9)) < .6, 'Gate-off current bias: recheck offsets before drive'
            p.command('limit 10')
            p.command('leg rate 10')
            p.command('foot speed 20')

            def segment(line, joints, seconds, rate, settle=True):
                start = len(p.rows)
                began = time.monotonic()
                p.command(line)
                p.driving = True
                p.wait(seconds)
                def stability():
                    tail = [r for stamp, r in p.rows[start:] if stamp > time.monotonic()-.5]
                    return ([max(abs(r[i]-joints[i]) for r in tail) for i in (0, 1)],
                            [statistics.pstdev(r[i] for r in tail) for i in (0, 1)])
                error, deviation = stability()
                extra = 0
                if settle and (max(error) >= 1 or max(deviation) >= .15):
                    p.wait(2)
                    extra = 2
                    error, deviation = stability()
                entry = dict(command=line, motor_rate_rpm=rate, settle=settle,
                             start_host_time=began, first_row=start, end_row=len(p.rows),
                             elapsed_s=time.monotonic()-began, extra_settle_s=extra, joint_goal=joints,
                             stable_joint_error=error, stable_joint_std=deviation,
                             peak_iq=[max(abs(r[i]) for _, r in p.rows[start:]) for i in (8, 9)],
                             final=p.last)
                result['segments'].append(entry)
                print(json.dumps(clean({k: v for k, v in entry.items() if k != 'final'})), flush=True)
                if settle:
                    assert max(error) < 1 and max(deviation) < .15, 'Not settled'

            def foot(y, seconds, rate, settle=True):
                status, pose = solver.solve([0, y])
                assert status == 'ok', f'Illegal endpoint {y}: {status}'
                segment(f'foot pos 0 {y}', list(pose.q), seconds, rate, settle)

            def configure(rate):
                # Joint endpoint ownership allows rate changes while continuing
                # to hold both motors. Never release the linkage between stages.
                joints = list(p.last[:2])
                segment(f'leg pos {joints[0]:.2f} {joints[1]:.2f}', joints, 1, rate)
                p.command(f'leg rate {rate}')
                p.command(f'foot speed {20 if rate == 10 else 400}')

            initial = list(p.last[:2])
            for delta in (1, 3):
                joints = [v+delta for v in initial]
                if max(joints) > 108:
                    joints = [v-delta for v in initial]
                segment(f'leg pos {joints[0]:.2f} {joints[1]:.2f}', joints, 3, 10)
            foot(100, 7, 10)
            foot(110, 4, 10)
            foot(100, 4, 10)
            if args.max_rpm >= 45:
                configure(45)
                for y in (125, 150, 175, 200, 100):
                    foot(y, 4, 45)
            for rate in (60, 90, 120):
                if rate > args.max_rpm:
                    break
                configure(rate)
                foot(200, 3, rate)
                foot(100, 3, rate)
                # Reverse before arrival to exercise the reported trigger.
                foot(200, .55, rate, False)
                foot(100, 3, rate)
            foot(200 if args.max_rpm >= 45 else 110, 4, args.max_rpm)
            result['success'] = True
        except Exception as exc:
            result['error'] = str(exc)
            print('STOP: ' + str(exc), flush=True)
        finally:
            if args.drive:
                p.driving = False
                p.target = None
                p.send('stop')
                p.wait(.4)
                result['stop_verified'] = p.last[11] == p.last[12] == 0
                # Fault snapshot survives stop. Query only after outputs off.
                p.send('send 7')
                pending = bytearray()
                end = time.monotonic()+.35
                while time.monotonic() < end:
                    pending.extend(link.read(8192))
                    while len(pending) >= 100:
                        if pending[96:100] != b'\0\0\x80\x7f':
                            del pending[0]
                            continue
                        row = struct.unpack('<24f', pending[:96])
                        del pending[:100]
                        if row[23] == 5007:
                            result['fault_snapshot'] = row
                # Fault state (5) does not imply PWM remains active. Verify
                # the two hardware gate modes from group 2 after stopping.
                p.send('send 2')
                pending = bytearray()
                gate_rows = []
                end = time.monotonic()+.35
                while time.monotonic() < end:
                    pending.extend(link.read(8192))
                    while len(pending) >= 100:
                        if pending[96:100] != b'\0\0\x80\x7f':
                            del pending[0]
                            continue
                        row = struct.unpack('<24f', pending[:96])
                        del pending[:100]
                        if row[23] == 2:
                            gate_rows.append(row)
                result['stop_verified'] = bool(gate_rows) and all(
                    row[9] == row[10] == 0 for row in gate_rows)
                if gate_rows:
                    result['gates_after_stop'] = list(gate_rows[-1][9:11])
                p.send('send 5')
                p.wait(.2)
            result['final'] = p.last
            result['frames'] = len(p.rows)
            args.output.write_text(json.dumps(clean(result), indent=2)+'\n', encoding='utf-8')
            with args.output.with_suffix('.csv').open('w', newline='', encoding='utf-8') as stream:
                writer = csv.writer(stream)
                writer.writerow(['host_time']+[f'ch{i}' for i in range(24)])
                writer.writerows((stamp, *row) for stamp, row in p.rows)
            print(json.dumps(clean({k: v for k, v in result.items() if k not in ('segments', 'initial', 'final', 'geometry')})), flush=True)
    if not result['success']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
