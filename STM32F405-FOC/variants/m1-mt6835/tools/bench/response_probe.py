"""Repeatable short speed and position response tests; stops on every exit.

Speed arrival is measured on a trailing 20 ms average, not on one noisy point.
Position arrival requires a 100 ms residence inside +/-0.5 mechanical degree.
Bounds apply to the host experiment and never latch the firmware.
"""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import serial
from current_probe import collect, summary
from outer_probe import record, stats


def response(raw, events):
    w = np.frombuffer(raw, dtype='<u4').reshape(-1, 13)
    v = w.view('<f4')
    t = np.r_[0, np.cumsum(np.diff(w[:, 0] & 0xffffff).astype(np.int64) & 0xffffff)] / 1e6
    # 1 ms bins; average all five USB points, which repeat the outer update.
    n = len(v) // 5
    x = v[:n*5].reshape(n, 5, 13).mean(axis=1)
    time_ms = t[:n*5].reshape(n, 5).mean(axis=1)
    speed_avg = np.convolve(x[:, 6], np.ones(20)/20, mode='full')[:n]
    # Serial receive buffering can drift relative to host wall time. Locate
    # each command in the firmware target/mode channels and use ADC timestamps.
    observed=[]
    after=0
    for event in events:
        command=event['command']
        if command.startswith(('rpm ', 'pos ')):
            speed_mode=command.startswith('rpm ')
            channel=8 if speed_mode else 5
            mode=1 if speed_mode else 2
            target=float(command.split()[1])
            match=(v[:,10]==mode)&np.isclose(v[:,channel],target,rtol=0,atol=1e-5)
        elif command=='stop':
            match=(v[:,10]==0)&(((w[:,0]>>24)&7)==0)
        else:
            raise ValueError('Response timestamps require explicit rpm/pos targets')
        found=np.flatnonzero(match[after:])
        if not len(found):raise RuntimeError('Command absent from telemetry: '+command)
        after+=int(found[0])
        observed.append(float(t[after]));after+=1
    results = []
    for i, e in enumerate(events[:-1]):
        if not e['command'].startswith(('rpm ', 'pos ')):
            continue
        target = float(e['command'].split()[1])
        start = observed[i]
        end = observed[i+1]
        idx = np.flatnonzero((time_ms >= start) & (time_ms < end))
        speed_mode = e['command'].startswith('rpm ')
        signal = speed_avg if speed_mode else x[:, 4]
        band = 3.0 if speed_mode else .5
        good = abs(signal[idx] - target) <= band
        residence = np.convolve(good.astype(int), np.ones(100, dtype=int), mode='valid')
        success = np.flatnonzero(residence == 100)
        first = float(time_ms[idx[success[0]]] - start) if len(success) else None
        # Short rapid moves must not include most of their travel in a
        # statistic labelled final. Keep the last quarter, at most 300 ms.
        last = idx[-min(300, max(1, len(idx)//4)):]
        initial = signal[max(0, idx[0]-1)]
        direction = np.sign(target-initial)
        crossings=[]
        for fraction in (.1,.9):
            threshold=initial+fraction*(target-initial)
            crossed=np.flatnonzero(direction*(signal[idx]-threshold)>=0)
            crossings.append(float(time_ms[idx[crossed[0]]]-start) if len(crossed) else None)
        overshoot = float(max(0, np.max(direction*(signal[idx]-target))))
        results.append(dict(command=e['command'], arrival_s=first, band=band,
            observed_start_s=start, timestamp_source='firmware_target',
            t90_s=crossings[1], rise_10_90_s=(crossings[1]-crossings[0]) if all(c is not None for c in crossings) else None,
            residence_s=.1, overshoot=overshoot, final_mean=float(signal[last].mean()),
            final_error=float(signal[last].mean()-target), iq_peak=float(abs(x[idx,2]).max())))
    return results


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--label', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--repeat', type=int, default=2)
    p.add_argument('--dwell', type=float, default=1.5)
    p.add_argument('--kind', choices=('speed','position','both'), default='both')
    p.add_argument('--deg', type=float, default=15)
    p.add_argument('--rpm', type=float, default=30)
    p.add_argument('--max-iq-ref', type=float, default=1.8)
    p.add_argument('--max-position', type=float, default=1000.0,
                   help='Host experiment bound in degrees for continuous rotation')
    args = p.parse_args()
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    results=[]
    with serial.Serial('COM8',1000000,timeout=.003,write_timeout=.2) as s:
        try:
            s.write(b'stop\n'); s.dtr=False; time.sleep(.15); s.reset_input_buffer(); s.dtr=True
            pre=summary(collect(s,0,.5)[0],0,2)
            if pre['states']!={'0':pre['frames']} or not 8<=pre['bus_mean']<=12.5:
                raise RuntimeError('Not a valid powered idle baseline')
            print('Preflight bus',pre['bus_mean'],flush=True)
            kinds=('speed','position') if args.kind=='both' else (args.kind,)
            for repeat in range(args.repeat):
                for kind in kinds:
                    s.write(b'stop\nzero\nclear\n'); time.sleep(.1)
                    if kind=='speed':
                        lead = 5 if abs(args.rpm - 10) < 1e-9 else 10
                        commands=[f'rpm {lead}',f'rpm {args.rpm:.2f}',
                                  f'rpm {-args.rpm:.2f}','rpm 0','stop']
                    else:
                        commands=[f'pos {args.deg:.2f}',f'pos {-args.deg:.2f}',f'pos {args.deg*2:.2f}','pos 0','stop']
                    events=[(.2+i*args.dwell,c) for i,c in enumerate(commands)]
                    name=f'{args.label}_{repeat}_{kind}'
                    try:
                        raw,sent=record(s,.5+4*args.dwell,events,
                                        max_iq_ref=args.max_iq_ref,
                                        max_position=args.max_position)
                    except RuntimeError as e:
                        if hasattr(e,'raw'):
                            (out/f'{name}_aborted.f32').write_bytes(e.raw)
                            (out/f'{name}_aborted.json').write_text(json.dumps(dict(error=str(e),events=e.events),indent=2))
                        raise
                    (out/f'{name}.f32').write_bytes(raw)
                    result=stats(raw,sent); result.update(kind=kind,repeat=repeat,response=response(raw,sent))
                    results.append(result)
                    (out/f'{args.label}.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
                    print(json.dumps(dict(name=name,response=result['response'],iq_peak=result['max_abs_iq_ref'])),flush=True)
                    if result['continuity']['gaps']:raise RuntimeError('Capture gaps')
        finally:
            s.write(b'stop\n');s.flush();time.sleep(.05);s.dtr=False


if __name__=='__main__':main()
