"""Bounded commissioning runs for the 12 V M1/TLE5012B Longqiu branch.

Raw ADC-scaled currents are provisional. Each stage stops in a finally block;
an independent host timer also sends stop at the stage deadline.
"""
import argparse
import json
import math
import threading
import time
from pathlib import Path

import numpy as np
import serial
from benchlib import FrameReader, continuity


class Probe:
    def __init__(self, port, out):
        self.link = serial.Serial(port, 1000000, timeout=0.003, write_timeout=0.2)
        self.link.dtr = True
        self.out = out
        self.offsets = None

    def command(self, command):
        self.link.write((command + '\n').encode())
        self.link.flush()

    def capture(self, name, group, seconds, command=None, check=None):
        self.command(f'send {group}')
        until = time.monotonic() + .08
        while time.monotonic() < until:
            self.link.read(max(1, min(self.link.in_waiting, 65536)))
        reader, records = FrameReader(), []
        start = last = time.monotonic()
        guard = threading.Timer(seconds + .3, self.command, args=('stop',)) if command else None
        if guard:
            guard.start()
        try:
            if command:
                self.command(command)
            while time.monotonic() - start < seconds:
                for frame in reader.feed(self.link.read(max(1, min(self.link.in_waiting, 65536)))):
                    if frame.group != group:
                        continue
                    records.append(frame.body)
                    last = time.monotonic()
                    values = np.frombuffer(frame.body, dtype='<f4')
                    if frame.fault or not np.isfinite(values[2:]).all():
                        raise RuntimeError(f'Invalid frame: {frame}')
                    bus_index = {0: 7, 3: 11, 4: 11, 5: 11}.get(group)
                    if bus_index is not None and not 10.8 <= values[bus_index] <= 13.5:
                        raise RuntimeError(f'Motor supply outside initial 12 V region: {values[bus_index]}')
                    if check:
                        check(frame, values)
                # Saving calibration erases sector 11 with gates off. The
                # single-bank F405 pauses USB until the flash operation ends.
                stall_limit = 3.0 if command == 'cal' and time.monotonic() - start > 9.8 else .4
                if time.monotonic() - last > stall_limit:
                    raise RuntimeError('Telemetry stalled')
        finally:
            if command:
                try:
                    self.command('stop')
                finally:
                    guard.cancel()
            raw = b''.join(records)
            (self.out / (name + '.payload')).write_bytes(raw)
        if len(records) < 100:
            raise RuntimeError('Insufficient telemetry')
        words = np.frombuffer(raw, dtype='<u4').reshape(-1, 12)
        values = words.view('<f4').copy()
        np.save(self.out / (name + '.npy'), values)
        stats = {'frames': len(values), 'group': group,
                 'states': np.unique((words[:, 0] >> 24) & 7).tolist(),
                 'pwm_frames': int(np.count_nonzero(words[:, 0] >> 31)),
                 'continuity': continuity(words, group, 10000, 2)}
        print(name, json.dumps(stats), flush=True)
        return values, stats

    def idle(self, name):
        self.command('stop')
        raw, stats = self.capture(name + '_raw', 0, .5)
        if stats['states'] != [0] or stats['pwm_frames'] != 0:
            raise RuntimeError(f'Not idle: {stats}')
        phase, _ = self.capture(name + '_phase', 1, .3)
        self.offsets = (float(np.mean(phase[:, 10])), float(np.mean(phase[:, 11])))
        diag, _ = self.capture(name + '_diag', 4, .3)
        stats.update(bus_mean_v=float(raw[:, 7].mean()), angle_min_deg=float(raw[:, 8].min()),
                     angle_max_deg=float(raw[:, 8].max()), b_offset_v=self.offsets[0],
                     c_offset_v=self.offsets[1], warnings=int(diag[-1, 4]),
                     rejected=int(diag[-1, 8]))
        self.save(name, stats)
        return stats

    def save(self, name, stats):
        (self.out / (name + '.json')).write_text(json.dumps(stats, indent=2), encoding='utf-8')
        print(json.dumps(stats), flush=True)

    def calibration(self):
        self.idle('before_cal')
        self.command('clear')
        travel = [None, 0., 0., 0.]
        peaks = [0., 0.]

        def check(frame, x):
            if travel[0] is not None:
                delta = (float(x[8]) - travel[0] + 180.) % 360. - 180.
                travel[1] += delta
                travel[2] = min(travel[2], travel[1])
                travel[3] = max(travel[3], travel[1])
            travel[0] = float(x[8])
            for i in range(2):
                current = abs((float(x[5 + i]) - self.offsets[i]) * 50.)
                peaks[i] = max(peaks[i], current)
                if current > 4.:
                    raise RuntimeError(f'Calibration phase-current bound exceeded: {current}')
            ia = abs(((float(x[5]) - self.offsets[0]) +
                      (float(x[6]) - self.offsets[1])) * 50.)
            if ia > 4.:
                raise RuntimeError(f'Calibration reconstructed A-current bound exceeded: {ia}')
            if abs(travel[1]) > 90.:
                raise RuntimeError('Calibration exceeded expected mechanical excursion')

        x, stats = self.capture('cal', 0, 14.0, 'cal', check)
        after = self.idle('after_cal')
        stats.update(travel_min_deg=travel[2], travel_max_deg=travel[3],
                     final_travel_deg=travel[1], phase_peak_a=peaks, after=after)
        self.save('cal', stats)
        if 2 not in stats['states'] or after['warnings'] & ((1 << 5) | (1 << 6)):
            raise RuntimeError('Calibration did not complete cleanly; inspect record before motion')

    def current(self):
        self.idle('before_current')
        for target in (.10, -.10):
            self.command('clear')

            def check(frame, x):
                if abs(x[7]) > 150. or abs(x[9]) > 1. or abs(x[4]) > 1.:
                    raise RuntimeError('Small-current test left commissioning region')
                if math.hypot(float(x[5]), float(x[6])) > float(x[11]) * .101:
                    raise RuntimeError('Unexpected voltage ceiling')

            name = 'iq_positive' if target > 0 else 'iq_negative'
            x, stats = self.capture(name, 5, .4, f'Iq {target:.2f}', check)
            run = x[((x.view('<u4')[:, 0] >> 24) & 7) == 4]
            if len(run) < 100:
                raise RuntimeError('Current command did not enter RUN; calibration may be invalid')
            stable = run[-min(1000, len(run)):]
            stats.update(target_a=target, iq_mean_a=float(stable[:, 9].mean()),
                         id_mean_a=float(stable[:, 4].mean()), rpm_peak=float(abs(run[:, 7]).max()),
                         ud_mean_v=float(stable[:, 5].mean()), uq_mean_v=float(stable[:, 6].mean()),
                         voltage_scale_min=float(run[:, 8].min()))
            self.save(name, stats)
            self.idle(name + '_stop')

    def speed(self, rpm):
        self.idle('before_speed')
        for target in (rpm, -rpm):
            self.command('clear')

            def check(frame, x):
                if abs(x[6]) > 80. or abs(x[2]) > 2.5 or abs(x[9]) > 3.:
                    raise RuntimeError('Low-speed test left commissioning region')

            name = 'rpm_positive' if target > 0 else 'rpm_negative'
            x, stats = self.capture(name, 3, 2.5, f'rpm {target:.0f}', check)
            run = x[((x.view('<u4')[:, 0] >> 24) & 7) == 4]
            if len(run) < 100:
                raise RuntimeError('Speed command did not enter RUN')
            final = run[-min(2500, len(run)):]
            stats.update(target_rpm=target, rpm_final_mean=float(final[:, 6].mean()),
                         rpm_final_std=float(final[:, 6].std()), rpm_peak=float(abs(run[:, 6]).max()),
                         travel_deg=float(run[-1, 4] - run[0, 4]),
                         iq_target_peak_a=float(abs(run[:, 2]).max()),
                         iq_actual_peak_a=float(abs(run[:, 9]).max()))
            self.save(name, stats)
            self.idle(name + '_stop')

    def close(self):
        try:
            self.command('stop')
        finally:
            self.link.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM8')
    parser.add_argument('--out', required=True)
    parser.add_argument('--stage', required=True, choices=('idle', 'cal', 'current', 'speed'))
    parser.add_argument('--rpm', type=float, default=10.0, choices=(10., 30.))
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    probe = Probe(args.port, out)
    try:
        if args.stage == 'idle':
            probe.idle('preflight')
        elif args.stage == 'cal':
            probe.calibration()
        elif args.stage == 'current':
            probe.current()
        else:
            probe.speed(args.rpm)
    finally:
        probe.close()


if __name__ == '__main__':
    main()
