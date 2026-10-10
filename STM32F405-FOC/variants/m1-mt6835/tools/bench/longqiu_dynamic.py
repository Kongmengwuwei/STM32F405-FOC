"""Continuous speed/position reversals for the 12 V free-shaft Longqiu.

Uses overview target and device timestamps for response timing. An independent
host deadline sends stop even if parsing stalls. Limits bound the experiment.
"""
import argparse
import json
import struct
import threading
import time
from pathlib import Path
import numpy as np
import serial


def analyse(x, kind, targets):
    t = np.r_[0, np.cumsum(np.diff(x[:, 22]).astype(np.int64) & 0xffffff)] * 1e-6
    if np.any(np.diff(t) > .000405):
        raise RuntimeError('Telemetry gap invalidates response timing')
    target_ch, mode = (0, 1) if kind == 'speed' else (2, 2)
    # 20 ms trailing speed average, identical before/after. Also derive speed
    # directly from the unfiltered position change over that window.
    n = 50
    speed = np.convolve(x[:, 1], np.ones(n) / n, mode='full')[:len(x)]
    velocity = np.r_[np.full(n, np.nan), (x[n:, 3] - x[:-n, 3]) / .12]
    signal = speed if kind == 'speed' else x[:, 3]
    starts, after = [], 0
    for target in targets:
        found = np.flatnonzero((x[after:, 11] == mode) & np.isclose(x[after:, target_ch], target, atol=1e-4, rtol=0))
        if not len(found):
            raise RuntimeError(f'Target missing: {target}')
        after += int(found[0])
        starts.append(after)
        after += 1
    results = []
    for i, target in enumerate(targets):
        start = starts[i]
        stop = starts[i+1] if i+1 < len(starts) else len(x)
        idx = np.arange(start, stop)
        initial = float(signal[max(0, start-1)])
        direction = np.sign(target-initial)
        band = max(3., abs(target)*.05) if kind == 'speed' else .5
        good = abs(signal[idx] - target) <= band
        residence = 250  # 100 ms at 2.5 kHz
        windows = np.convolve(good.astype(int), np.ones(residence, dtype=int), mode='valid') if len(idx)>=residence else []
        hits = np.flatnonzero(np.asarray(windows) == residence)
        def crossed(fraction):
            h = np.flatnonzero(direction*(signal[idx] - (initial + fraction*(target-initial))) >= 0)
            return float(t[idx[h[0]]]-t[start]) if len(h) else None
        a, b = crossed(.1), crossed(.9)
        tail = idx[-min(750, len(idx)//4):]
        reverse = np.flatnonzero(direction * signal[idx] >= 0) if initial*target<0 and abs(initial)>band else []
        results.append(dict(target=target, initial=initial, start_s=float(t[start]),
                            t90_s=b, rise_10_90_s=b-a if a is not None and b is not None else None,
                            reversal_zero_s=float(t[idx[reverse[0]]]-t[start]) if len(reverse) else None,
                            arrival_s=float(t[idx[hits[0]]]-t[start]) if len(hits) else None,
                            band=band, residence_s=.1,
                            overshoot=float(max(0, np.max(direction*(signal[idx]-target)))),
                            final_mean=float(signal[tail].mean()), final_std=float(signal[tail].std()),
                            final_error=float(signal[tail].mean()-target),
                            final_peak_to_peak=float(np.ptp(signal[tail])),
                            position_velocity_20ms_mean=float(np.nanmean(velocity[tail])),
                            position_velocity_20ms_std=float(np.nanstd(velocity[tail])),
                            iq_target_peak_a=float(abs(x[idx, 4]).max()),
                            iq_peak_a=float(abs(x[idx, 5]).max()), id_peak_a=float(abs(x[idx, 7]).max()),
                            dq_volts_peak=float(np.hypot(x[idx, 9], x[idx, 10]).max()),
                            voltage_scale_min=float(x[idx, 17].min())))
    return dict(response=results, frames=len(x), warnings=int(x[-1, 14]),
                rejected=int(x[-1, 16]), fault_peak=int(x[:, 13].max()),
                bus_min=float(x[:, 8].min()), bus_max=float(x[:, 8].max()),
                speed_peak=float(abs(x[:, 1]).max()),
                seq_gaps=int(np.count_nonzero((np.diff(x[:, 23]).astype(np.int64)&0xffffff)!=4)))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    ap.add_argument('--port', default='COM8')
    ap.add_argument('--kind', choices=['speed', 'position'], default='speed')
    ap.add_argument('--targets', nargs='+', type=float, required=True)
    ap.add_argument('--dwell', type=float, default=1.2)
    ap.add_argument('--max-rpm', type=float, default=180)
    ap.add_argument('--max-current', type=float, default=3.5)
    args = ap.parse_args()
    if not .5 <= args.dwell <= 2 or len(args.targets)>8 or not args.targets:
        ap.error('Use 0.5..2 s dwell and 1..8 targets')
    if any(abs(v)>600 for v in args.targets) or not 0 < args.max_current <= 10 or not 0 < args.max_rpm <= 1000:
        ap.error('Trial limits exceed the supported short-bench range')
    out = Path(args.out); out.parent.mkdir(parents=True, exist_ok=True)
    rows = []; pending = bytearray(); prefix = 'rpm' if args.kind == 'speed' else 'pos'
    # Native CDC briefly disappears after an SWD reset. Retry only absence;
    # an occupied port requires closing its owner, not repeated open attempts.
    deadline = time.monotonic() + 5
    while True:
        try:
            link = serial.Serial(args.port, 1000000, timeout=.003, write_timeout=.2)
            break
        except serial.SerialException as exc:
            if not isinstance(exc.__context__, FileNotFoundError) or time.monotonic() >= deadline:
                raise
            time.sleep(.2)
    with link as s:
        lock = threading.Lock()
        def send(line):
            with lock:
                s.write((line+'\r\n').encode('ascii')); s.flush()
        guard = None
        try:
            s.dtr = True; send('stop'); send('clear'); send('zero'); send('send 6')
            # Discard the prior session and pre-zero snapshots before imposing
            # bounds on position relative to this experiment's new zero.
            drain_end = time.monotonic() + .12
            while time.monotonic() < drain_end:
                s.read(max(1, min(s.in_waiting, 65536)))
            s.reset_input_buffer()
            start=time.monotonic(); last=start; next_event=0; valid_idle=False
            total=.2+len(args.targets)*args.dwell
            guard=threading.Timer(total+.3, send, args=('stop',)); guard.start()
            while time.monotonic()-start<total:
                elapsed=time.monotonic()-start
                if next_event<len(args.targets) and elapsed>=.2+next_event*args.dwell:
                    if not valid_idle and next_event==0: raise RuntimeError('No powered idle before motion')
                    send(f'{prefix} {args.targets[next_event]:g}'); next_event+=1
                pending.extend(s.read(max(1, min(s.in_waiting, 65536))))
                while len(pending)>=100:
                    if pending[96:100]!=b'\x00\x00\x80\x7f':
                        del pending[:1]; continue
                    row=struct.unpack('<24f', pending[:96]); del pending[:100]
                    rows.append(row); last=time.monotonic()
                    if not all(np.isfinite(row)): raise RuntimeError('Nonfinite telemetry')
                    if not 10.8<=row[8]<=13.5 or row[13]!=0: raise RuntimeError('Bus/fault bound exceeded')
                    if next_event==0 and row[12]==0: valid_idle=True
                    if abs(row[1])>args.max_rpm or max(abs(row[4]),np.hypot(row[5],row[7]))>args.max_current:
                        raise RuntimeError(f'Motion bound exceeded: rpm={row[1]:.2f}, ref={row[4]:.2f}, Iq={row[5]:.2f}, Id={row[7]:.2f}')
                    if args.kind=='position' and abs(row[3])>max(abs(v) for v in args.targets)+25:
                        raise RuntimeError('Position excursion exceeded bound')
                if time.monotonic()-last>.4: raise RuntimeError('Telemetry stalled')
        finally:
            send('stop')
            if guard: guard.cancel(); guard.join()
            x=np.array(rows, dtype='<f4'); np.save(out.with_suffix('.npy'),x)
            # Leave the normal 24-channel VOFA stream selected.
            send('send 6')
    stat=analyse(x,args.kind,args.targets)
    stat.update(kind=args.kind, targets=args.targets, dwell_s=args.dwell)
    out.with_suffix('.json').write_text(json.dumps(stat,indent=2),encoding='utf-8')
    print(json.dumps(dict(kind=args.kind, warnings=stat['warnings'], fault=stat['fault_peak'],
                          response=[{k:r[k] for k in ['target','t90_s','arrival_s','overshoot','final_mean','iq_target_peak_a','iq_peak_a']} for r in stat['response']])) ,flush=True)


if __name__=='__main__': main()
