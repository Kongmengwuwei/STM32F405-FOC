"""Short unloaded group-6 acceptance run. Always stop and drain USB on exit.

Currents are firmware ADC-scaled values, not independently measured amperes.
Close VOFA's serial connection before running this tool.
"""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import serial

TAIL = b'\x00\x00\x80\x7f'


def capture(link, seconds):
    buf = bytearray()
    frames = []
    end = time.monotonic() + seconds
    last = time.monotonic()
    while time.monotonic() < end:
        buf.extend(link.read(max(1, min(link.in_waiting, 65536))))
        while True:
            at = buf.find(TAIL)
            if at < 0:
                break
            packet = bytes(buf[:at]); del buf[:at+4]
            if len(packet) != 96:
                continue  # Initial partial frame / former logging group.
            x = np.frombuffer(packet, dtype='<f4').copy()
            if not np.isfinite(x).all() or x[13] != 0:
                raise RuntimeError('Invalid data or firmware fault')
            if not 8 <= x[8] <= 12.5 or abs(x[1]) > 200 or abs(x[4]) > 1.8:
                raise RuntimeError('Outside unloaded acceptance region')
            frames.append(x); last = time.monotonic()
        if time.monotonic() - last > .5:
            raise RuntimeError('Telemetry stalled')
    if len(frames) < 100:
        raise RuntimeError('Insufficient frames')
    return np.array(frames)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='COM8')
    p.add_argument('--out', required=True)
    args = p.parse_args()
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    all_data, results = [], []
    with serial.Serial(args.port, 1000000, timeout=.003, write_timeout=.2) as link:
        try:
            link.write(b'stop\nsend 6\n')
            pre = capture(link, .5)
            if not np.all(pre[:,12] == 0):
                raise RuntimeError('Board is not idle')
            link.write(b'zero\nclear\n')
            for command, duration, target, actual, mode in [
                ('rpm 30', 1.4, 0, 1, 1), ('rpm -30', 1.4, 0, 1, 1),
                ('pos 15', 1.4, 2, 3, 2), ('pos -15', 1.4, 2, 3, 2),
                ('Iq 0.10', .4, 4, 5, 0), ('stop', .5, None, None, 0)]:
                link.write((command+'\n').encode())
                data = capture(link, duration); all_data.append(data)
                row = dict(command=command, frames=len(data),
                    bus_mean=float(data[:,8].mean()), faults=np.unique(data[:,13]).tolist(),
                    rejected_max=float(data[:,16].max()))
                if target is not None:
                    value = float(command.split()[1])
                    run = data[(data[:,12] == 4) & (data[:,11] == mode) & np.isclose(data[:,target],value)]
                    if len(run) < 100:
                        raise RuntimeError('Command not reflected: '+command)
                    final = run[-min(len(run), 500):]
                    row.update(target=value, actual_mean=float(final[:,actual].mean()),
                        error_mean=float((final[:,target]-final[:,actual]).mean()),
                        actual_std=float(final[:,actual].std()),
                        iq_target_peak=float(abs(data[:,4]).max()))
                elif not np.all(data[-100:,12] == 0):
                    raise RuntimeError('Stop did not reach idle')
                results.append(row); print(json.dumps(row), flush=True)
        finally:
            link.write(b'stop\nsend 6\n'); link.flush()
            until = time.monotonic()+.3
            while time.monotonic() < until:
                link.read(max(1,min(link.in_waiting,65536)))
            link.dtr = False
    data = np.concatenate(all_data)
    dt = (np.diff(data[:,22].astype(np.int64)) & 0xffffff)
    ds = (np.diff(data[:,23].astype(np.int64)) & 0xffffff)
    # Segment boundaries discard a partial frame; count continuity inside each capture.
    gaps = sum(int(np.sum((np.diff(a[:,23].astype(np.int64)) & 0xffffff) != 4)) for a in all_data)
    result = dict(segments=results, frames=len(data),
        sample_hz=float(1e6/np.median(dt)), nominal_interval_us=int(np.median(dt)),
        nominal_sequence_step=int(np.median(ds)), gaps_within_segments=gaps,
        mapping_error_max=[float(abs(data[:,19]-(data[:,18]-data[:,1])).max()),
            float(abs(data[:,20]-(data[:,2]-data[:,3])).max()),
            float(abs(data[:,21]-(data[:,4]-data[:,5])).max())],
        final_state=int(data[-1,12]), final_fault=int(data[-1,13]))
    np.save(out/'overview.npy', data)
    (out/'overview.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
