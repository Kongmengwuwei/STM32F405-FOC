"""Bounded speed/position comparison. Always sends stop on exit.

Bounds belong to this experiment, not firmware faults. Currents are internal
ADC-scaled values, not independently calibrated amperes.
"""
import argparse
import json
import struct
import time
from pathlib import Path

import numpy as np
import serial
from benchlib import FrameReader, continuity
from current_probe import collect, summary


def record(link, seconds, events, max_iq_ref=1.2, max_position=1000.0):
    link.write(b"send 3\n")
    reader = FrameReader()
    frames, sent = [], []
    first = None
    last = time.monotonic()
    deadline = last + seconds + 2
    index = 0
    while first is None or time.monotonic() - first < seconds:
        now = time.monotonic()
        if now > deadline or now - last > .5:
            raise RuntimeError("Telemetry stalled")
        if first is not None:
            while index < len(events) and now - first >= events[index][0]:
                at, command = events[index]
                link.write((command + "\n").encode())
                sent.append(dict(requested_s=at, sent_s=now-first, command=command))
                index += 1
        for frame in reader.feed(link.read(max(1, min(link.in_waiting, 65536)))):
            if frame.group != 3:
                continue
            if first is None:
                first = time.monotonic()
            last = time.monotonic()
            v = np.frombuffer(frame.body, dtype="<f4")
            frames.append(frame.body + struct.pack("<f", float("inf")))
            if frame.fault or not np.all(np.isfinite(v[2:])):
                raise RuntimeError(f"Invalid telemetry: fault {frame.fault}")
            if not 8 <= v[11] <= 12.5 or abs(v[7]) > 200 or abs(v[2]) > max_iq_ref or abs(v[4]) > max_position:
                error = RuntimeError(f"Outside comparison region: rpm={v[7]}, Iq={v[2]}, position={v[4]}, bus={v[11]}")
                error.raw, error.events = b''.join(frames), sent
                raise error
    return b"".join(frames), sent


def stats(raw, sent):
    w = np.frombuffer(raw, dtype="<u4").reshape(-1, 13)
    v = w.view("<f4")
    t = np.r_[0, np.cumsum(np.diff(w[:, 0] & 0xffffff).astype(np.int64) & 0xffffff)] / 1e6
    run = ((w[:, 0] >> 24) & 7) == 4
    result = dict(frames=len(w), continuity=continuity(w[:, :12], 3, 10000, 2), events=sent,
                  max_abs_rpm=float(abs(v[:, 7]).max()), max_abs_iq_ref=float(abs(v[:, 2]).max()))
    segments = []
    for i, e in enumerate(sent):
        if not e['command'].startswith(('rpm ', 'pos ', 'hold')):
            continue
        end = sent[i+1]['sent_s'] if i+1 < len(sent) else t[-1]
        m = run & (t >= max(e['sent_s'] + .4, end - .7)) & (t < end)
        if m.sum() < 50:
            continue
        x = v[m]
        speed_error = x[:, 6] - x[:, 8]
        position_error = x[:, 4] - x[:, 5]
        segments.append(dict(command=e['command'], start_s=e['sent_s'], end_s=end,
            speed_mean=float(x[:, 6].mean()), speed_std=float(x[:, 6].std()),
            speed_rmse=float(np.sqrt(np.mean(speed_error**2))),
            position_mean=float(x[:, 4].mean()), position_error=float(position_error.mean()),
            position_std=float(x[:, 4].std()), position_pp=float(np.ptp(x[:, 4])),
            iq_mean=float(x[:, 2].mean()), iq_peak=float(abs(x[:, 2]).max())))
    result['segments'] = segments
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--label', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--kind', choices=('speed', 'position', 'both'), default='both')
    p.add_argument('--rpm', type=float, default=20)
    p.add_argument('--deg', type=float, default=15)
    p.add_argument('--dwell', type=float, default=2)
    args = p.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    results = []
    with serial.Serial('COM8', 1000000, timeout=.003, write_timeout=.2) as link:
        try:
            link.write(b'stop\n')
            link.dtr = False
            time.sleep(.15)
            link.reset_input_buffer()
            link.dtr = True
            raw, _, _ = collect(link, 0, .5)
            pre = summary(raw, 0, 2)
            if pre['states'] != {'0': pre['frames']} or not 8 <= pre['bus_mean'] <= 12.5:
                raise RuntimeError('Not a valid powered idle baseline')
            print('Preflight', pre['bus_mean'], flush=True)
            kinds = ('speed', 'position') if args.kind == 'both' else (args.kind,)
            for kind in kinds:
                link.write(b'stop\nzero\nclear\n')
                time.sleep(.1)
                if kind == 'speed':
                    commands = ['rpm 5', f'rpm {args.rpm:.2f}', f'rpm {-args.rpm:.2f}', 'rpm 0', 'stop']
                else:
                    commands = [f'pos {args.deg:.2f}', 'pos 0', f'pos {-args.deg:.2f}', 'pos 0', 'stop']
                events = [(.2 + i * args.dwell, command) for i, command in enumerate(commands)]
                try:
                    raw, sent = record(link, .5 + 4 * args.dwell, events)
                except RuntimeError as error:
                    if hasattr(error, 'raw'):
                        (out / f'{args.label}_{kind}_aborted.f32').write_bytes(error.raw)
                        aborted = stats(error.raw, error.events)
                        aborted['abort'] = str(error)
                        (out / f'{args.label}_aborted.json').write_text(json.dumps(aborted, indent=2))
                    raise
                (out / f'{args.label}_{kind}.f32').write_bytes(raw)
                result = stats(raw, sent)
                result['kind'] = kind
                results.append(result)
                (out / f'{args.label}.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
                print(json.dumps(result), flush=True)
                if result['continuity']['gaps']:
                    raise RuntimeError('Capture gaps')
            link.write(b'stop\nsend 3\n')
        finally:
            link.write(b'stop\n')
            link.flush()
            time.sleep(.05)
            link.dtr = False


if __name__ == '__main__':
    main()
