"""12 V/free-shaft anticogging: bidirectional stationary learning and A/B trials.

The firmware only accepts a complete, bounded table while stopped. Learning
uses the existing position loop; the current-loop gains are not changed.
"""
import argparse
import json
import struct
import threading
import time
from pathlib import Path

import numpy as np
import serial

POINTS = 720
MAX_MAP = 1.5


class Link:
    def __init__(self, port):
        self.s = serial.Serial(port, 1000000, timeout=.005, write_timeout=.2)
        self.s.dtr = True
        self.lock = threading.Lock()
        self.pending = bytearray()
        self.group = 6
        self.last_sequence = None
        self.guard = None

    def send(self, text):
        with self.lock:
            self.s.write((text + '\r\n').encode('ascii'))
            self.s.flush()

    def select(self, group):
        self.send(f'send {group}')
        self.group = group
        self.last_sequence = None
        end = time.monotonic() + .12
        while time.monotonic() < end:
            self.s.read(max(1, min(self.s.in_waiting, 65536)))
        self.pending.clear()
        self.s.reset_input_buffer()

    def rows(self, duration):
        result = []
        end = time.monotonic() + duration
        last = time.monotonic()
        size = 100 if self.group == 6 else 52
        while time.monotonic() < end:
            self.pending.extend(self.s.read(max(1, min(self.s.in_waiting, 65536))))
            while len(self.pending) >= size:
                if self.pending[size-4:size] != b'\x00\x00\x80\x7f':
                    del self.pending[:1]
                    continue
                frame = bytes(self.pending[:size]); del self.pending[:size]
                if self.group == 6:
                    row = struct.unpack('<24f', frame[:-4])
                    if not all(np.isfinite(row)):
                        raise RuntimeError('Nonfinite overview')
                    if not 10.8 <= row[8] <= 13.5 or row[13] or row[14]:
                        raise RuntimeError(f'Bus/fault/warning bound: {row[8]}, {row[13:15]}')
                    if abs(row[1]) > 100 or max(abs(row[4]), np.hypot(row[5], row[7])) > 3:
                        raise RuntimeError(f'Motion bound: rpm {row[1]:.2f}, Iq_ref {row[4]:.2f}, Iq {row[5]:.2f}')
                else:
                    tw, sw = struct.unpack('<II', frame[:8])
                    if sw >> 24 != self.group:
                        continue
                    values = struct.unpack('<10f', frame[8:48])
                    if not all(np.isfinite(values)) or tw >> 24 & 0x87:
                        raise RuntimeError('Nonfinite/still running during stopped operation')
                    row = values
                result.append(row); last = time.monotonic()
            if time.monotonic() - last > .4:
                raise RuntimeError('Telemetry stalled')
        if len(result) < 20:
            raise RuntimeError('Insufficient telemetry')
        x = np.asarray(result)
        if self.group == 6 and len(x) > 1:
            if self.last_sequence is not None and (int(x[0,23])-self.last_sequence) & 0xffffff != 4:
                raise RuntimeError('Overview discontinuity between reads')
            if np.any((np.diff(x[:, 23]).astype(np.int64) & 0xffffff) != 4):
                raise RuntimeError('Overview sequence discontinuity')
            self.last_sequence=int(x[-1,23])
        return x

    def status(self):
        self.select(9)
        return self.rows(.03)[-1]

    def deadline(self, seconds):
        if self.guard:
            self.guard.cancel(); self.guard.join()
        self.guard = threading.Timer(seconds, self.send, args=('stop',))
        self.guard.start()

    def close(self):
        try:
            self.send('stop'); self.send('cog off'); self.send('send 6')
            if self.guard:
                self.guard.cancel(); self.guard.join()
        finally:
            self.s.close()


def binding(status):
    return dict(zero_rad=float(status[6]), direction=int(status[7]),
                pole_pairs=int(status[8]), installation_identity=int(status[9]))


def build_map(passes):
    grid = np.arange(POINTS) * .5
    maps = []
    for samples in passes:
        samples = np.asarray(samples)
        if samples.shape != (POINTS, 4) or not np.all(np.isfinite(samples)):
            raise RuntimeError('Incomplete bidirectional capture')
        angle, current = samples[:, 0] % 360, samples[:, 1]
        order = np.argsort(angle)
        angle, current = angle[order], current[order]
        if np.max(np.diff(np.r_[angle, angle[0]+360])) > .75:
            raise RuntimeError('Uneven angle coverage')
        maps.append(np.interp(grid, np.r_[angle[-1]-360, angle, angle[0]+360],
                              np.r_[current[-1], current, current[0]]))
    forward, reverse = maps
    f, r = forward-forward.mean(), reverse-reverse.mean()
    rms = float(np.sqrt(np.mean(((f+r)/2)**2)))
    repeat = float(np.sqrt(np.mean((f-r)**2)))
    correlation = float(np.corrcoef(f, r)[0, 1])
    if rms < .015 or not np.isfinite(correlation) or correlation < .65 or repeat > max(.08, rms*.8):
        raise RuntimeError(f'Not repeatable: rms={rms:.3f} A, difference={repeat:.3f} A, correlation={correlation:.3f}')
    table = (f+r)/2
    if abs(table).max() > MAX_MAP:
        raise RuntimeError('Learned map exceeds 1.5 A bound')
    return table, dict(rms_a=rms, repeat_rms_a=repeat, correlation=correlation,
                      peak_a=float(abs(table).max()), friction_bias_a=float((forward.mean()-reverse.mean())/2))


def learn(link, out):
    link.send('stop'); link.send('cog off'); link.send('clear'); link.send('zero')
    status = link.status()
    offset = status[2] % 360
    identity = binding(status)
    start_index = int(np.ceil(offset*2)) % POINTS
    start_target = (start_index*.5-offset) % 360
    if start_target > 1:
        raise RuntimeError('Unexpected initial angle')
    link.select(6)
    idle = link.rows(.04)
    if idle[-1, 12] != 0:
        raise RuntimeError('Not idle')
    rejected = idle[-1, 16]
    link.send('cog learn')
    link.deadline(POINTS*2*3.1+5)
    passes = [[], []]
    try:
        # A full clockwise turn, then the same locations in reverse. Revisit
        # the reversal endpoint after a dwell; interpolate at measured angles,
        # not the requested positions, before averaging away friction bias.
        for direction, k in [(1, 0), (-1, 1)]:
            for j in range(POINTS):
                index = j if direction == 1 else POINTS-1-j
                target = start_target + index*.5
                link.send(f'pos {target:.2f}')
                begin = time.monotonic(); window = np.empty((0, 24))
                while True:
                    # Refresh is required in TRIP firmware (200 ms watchdog).
                    link.send(f'pos {target:.2f}')
                    x = link.rows(.05)
                    if np.any(x[:, 16] != rejected) or np.any(abs(x[:, 3]-target) > 2):
                        np.save(out.with_suffix('.failed.npy'),x)
                        print(json.dumps(dict(target=target, rejection_old=float(rejected), rejection_new=float(x[-1,16]),
                            position_min=float(x[:,3].min()),position_max=float(x[:,3].max()),mode=float(x[-1,11]))),flush=True)
                        raise RuntimeError(f'Rejected command/mode/excursion at point {j}')
                    if np.any(x[:, 11] != 2) and time.monotonic()-begin > .15:
                        raise RuntimeError(f'Position mode lost at point {j}')
                    x=x[x[:, 11] == 2]  # Discard queued idle frames at first arm.
                    window = np.vstack([window, x])[-500:]  # 200 ms at 2.5 kHz
                    if len(window) >= 450 and time.monotonic()-begin > .22:
                        # Static torque, without learning velocity quantisation
                        # or an unsettled positional acceleration as cogging.
                        error = float(window[:, 3].mean()-target)
                        drift = float(window[-100:, 3].mean()-window[:100, 3].mean())
                        iq_std = float(window[:, 5].std())
                        # TLE quantisation and small servo jitter are averaged
                        # in a bounded neighborhood (sigma <= 0.07 deg). At
                        # 84 cycles/turn this is <0.11 rad RMS spatial phase.
                        if abs(error) < .10 and np.ptp(window[:, 3]) < .35 and window[:,3].std() < .07 and abs(drift) < .035 and iq_std < .15 and abs((window[:, 4]-window[:, 5]).mean()) < .06:
                            passes[k].append([float((offset+window[:, 3].mean()) % 360),
                                              float(window[:, 5].mean()), iq_std, error])
                            break
                    if time.monotonic()-begin > 3:
                        np.save(out.with_suffix('.failed.npy'),window)
                        print(json.dumps(dict(target=target, error_deg=float(window[:,3].mean()-target),
                            position_ptp_deg=float(np.ptp(window[:,3])), drift_deg=drift, iq_std_a=iq_std,
                            iq_mean=float(window[:,5].mean()), ref_mean=float(window[:,4].mean()))),flush=True)
                        raise RuntimeError(f'Point {j}/{POINTS}, direction {direction} did not settle')
                if j % 60 == 0:
                    print(f'pass {k+1}/2, {j+1}/{POINTS}, Iq={passes[k][-1][1]:.3f} A', flush=True)
                    out.with_suffix('.partial.json').write_text(json.dumps(dict(binding=identity, passes=passes)), encoding='utf-8')
        table, quality = build_map(passes)
        doc = dict(version=1, binding=identity, points=POINTS, map=table.tolist(), quality=quality, passes=passes)
        out.write_text(json.dumps(doc, indent=2), encoding='utf-8')
        print(json.dumps(quality), flush=True)
        return doc
    finally:
        link.send('stop')
        out.with_suffix('.partial.json').write_text(json.dumps(dict(binding=identity, passes=passes)), encoding='utf-8')


def upload(link, doc, gain):
    link.send('stop'); link.send('cog off')
    status = link.status()
    if binding(status) != doc['binding']:
        raise RuntimeError('Table belongs to a different magnet/phase calibration or installation')
    table = np.asarray(doc['map'])
    if doc.get('version') != 1 or table.shape != (POINTS,) or not np.all(np.isfinite(table)) or abs(table).max() > MAX_MAP:
        raise RuntimeError('Malformed table')
    link.send('cog begin')
    for index, value in enumerate(table):
        link.send(f'cog point {index} {value:.2f}')
        if index % 24 == 23:
            # Each batch must be accepted before continuing. This also drains
            # the status stream and avoids overflowing the USB RX queue.
            deadline=time.monotonic()+.3
            while True:
                row = link.rows(.02)[-1]
                if int(row[1]) == index+1 and int(row[0]) & 4:
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError(f'Upload rejected at {index}')
    link.send('cog commit'); link.send(f'cog gain {gain:.2f}')
    row = link.rows(.025)[-1]
    if not int(row[0]) & 1 or abs(row[4]-gain) > 1e-5:
        raise RuntimeError('Commit/gain rejected')


def trial(link, enabled, rpm, seconds):
    link.send('stop'); link.send('zero'); link.send('clear')
    link.send('cog on' if enabled else 'cog off')
    link.select(6)
    idle = link.rows(.04); rejected = idle[-1, 16]
    link.deadline(seconds+1)
    link.send(f'rpm {rpm:.2f}')
    batches=[]; start=time.monotonic()
    try:
        while time.monotonic()-start < seconds:
            link.send(f'rpm {rpm:.2f}')
            x=link.rows(.06)
            if np.any(x[:, 16] != rejected) or (np.any(x[:, 11] != 1) and time.monotonic()-start > .15):
                raise RuntimeError('Trial command rejected')
            x=x[x[:, 11] == 1]
            batches.append(x)
    finally:
        link.send('stop')
    x=np.vstack(batches)
    y=x[2500:]  # Discard the first second, include several full turns.
    n=50
    vel=(y[n:, 3]-y[:-n, 3])/.12  # Encoder displacement, 20 ms window.
    return x, dict(rpm=rpm, mean_rpm=float(vel.mean()), ripple_std_rpm=float(vel.std()),
                   error_rms_rpm=float(np.sqrt(np.mean((vel-rpm)**2))), peak_to_peak_rpm=float(np.ptp(vel)))


def compare(link, out, seconds):
    status=link.status()
    if not int(status[0]) & 1:
        raise RuntimeError('No valid table')
    report=[]
    # Paired repeats reduce the chance that a favorable starting angle alone
    # gets credited to the map. Both mechanical directions must improve.
    for rpm in [10, -10, 10, -10]:
        pair=[]
        for enabled in [False, True]:
            x, stat=trial(link,enabled,rpm,seconds)
            np.save(out.with_name(f'{out.stem}_{len(report)}_{int(enabled)}.npy'),x)
            pair.append(stat)
            print(json.dumps(dict(enabled=enabled, **stat)),flush=True)
        report.append(dict(off=pair[0], on=pair[1], ratio=pair[1]['error_rms_rpm']/pair[0]['error_rms_rpm']))
    accepted=all(p['ratio']<.85 and abs(p['on']['mean_rpm']-p['on']['rpm'])<1 for p in report)
    doc=dict(accepted=accepted, gain=float(status[4]), binding=binding(status), seconds=seconds, pairs=report)
    out.with_suffix('.comparison.json').write_text(json.dumps(doc,indent=2),encoding='utf-8')
    return accepted


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',default='COM8')
    ap.add_argument('--map',required=True)
    ap.add_argument('--learn',action='store_true')
    ap.add_argument('--load',action='store_true')
    ap.add_argument('--compare',action='store_true')
    ap.add_argument('--save',action='store_true',help='Save only after this run passes A/B acceptance')
    ap.add_argument('--gain',type=float,default=.5)
    ap.add_argument('--seconds',type=float,default=14)
    args=ap.parse_args()
    if not 0 < args.gain <= 1 or not 8 <= args.seconds <= 30 or (args.save and not args.compare):
        ap.error('gain (0,1], trial 8..30 seconds; --save requires --compare')
    path=Path(args.map);path.parent.mkdir(parents=True,exist_ok=True)
    link=Link(args.port)
    try:
        if args.learn:
            doc=learn(link,path); upload(link,doc,args.gain)
        elif args.load:
            upload(link,json.loads(path.read_text(encoding='utf-8')),args.gain)
        if args.compare:
            accepted=compare(link,path,args.seconds)
            if not accepted:
                raise RuntimeError('A/B acceptance failed; compensation left off, Flash unchanged')
            if args.save:
                # Sector 11 erase stalls Flash fetch/USB for longer than a
                # normal receive timeout. Finish the motion guard, then allow
                # the stopped save to complete before reading its status.
                link.deadline(5)
                link.send('cog off'); link.send('cog save'); time.sleep(2); link.select(7)
                row=link.rows(.08)[-1]
                if (int(row[9]) >> 2) & 3 != 2:
                    raise RuntimeError('Flash save not verified')
    finally:
        link.close()


if __name__=='__main__':
    main()
