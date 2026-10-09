"""Short free-shaft speed/position trials; always stop after each target.

Use with 12 V, the M1/TLE5012B/L4225 profile and a saved valid calibration.
JSON metrics are calculated from device samples, not host receive timing.
"""
import argparse
from pathlib import Path
import numpy as np
from longqiu_probe import Probe


def metrics(x, stage, target):
    run = x[((x.view('<u4')[:, 0] >> 24) & 7) == 4]
    if len(run) < 500:
        raise RuntimeError('Target did not enter RUN')
    t = np.arange(len(run)) * .0002
    tail = run[-2500:]
    result = dict(target=target, rpm_mean=float(tail[:, 6].mean()),
                  rpm_std=float(tail[:, 6].std()), rpm_peak=float(abs(run[:, 6]).max()),
                  iq_target_peak_a=float(abs(run[:, 2]).max()),
                  iq_actual_peak_a=float(abs(run[:, 9]).max()),
                  travel_deg=float(run[-1, 4] - run[0, 4]),
                  position_final_deg=float(run[-1, 4]))
    # Derive a separate 20 ms velocity from the position samples, so changing
    # the firmware speed filter cannot by itself improve the reported ripple.
    n = len(tail) // 100
    blocks = tail[:n * 100, 4].reshape(n, 100).mean(axis=1)
    velocity = np.diff(blocks) / .12
    result.update(position_velocity_20ms_mean=float(velocity.mean()),
                  position_velocity_20ms_std=float(velocity.std()))
    if stage == 'position':
        error = run[:, 4] - target
        outside = np.flatnonzero(abs(error) > .5)
        settle_index = outside[-1] + 1 if len(outside) else 0
        settle = float(t[settle_index]) if settle_index < len(t) - 1250 else None
        direction = np.sign(target - run[0, 4])
        result.update(error_final_deg=float(error[-1]),
                      error_tail_mean_deg=float((tail[:, 4] - target).mean()),
                      hold_peak_to_peak_deg=float(np.ptp(tail[:, 4])),
                      overshoot_deg=float(max(0., np.max(direction * error))),
                      settle_0_5_deg_s=settle, settle_min_hold_s=.25)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', default='COM8')
    ap.add_argument('--out', required=True)
    ap.add_argument('--stage', choices=('speed', 'position'), required=True)
    ap.add_argument('--targets', type=float, nargs='+', required=True)
    ap.add_argument('--seconds', type=float, default=3.)
    args = ap.parse_args()
    if not .5 <= args.seconds <= 5. or any(abs(t) > 60 for t in args.targets):
        ap.error('Initial free-shaft trials allow up to 5 s and +/-60 rpm/deg')
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    p = Probe(args.port, out)
    try:
        p.idle('before')
        p.command('clear')
        if args.stage == 'position':
            p.command('zero')
            p.capture('zero_check', 3, .15)
        for i, target in enumerate(args.targets):
            name = f'{args.stage}_{i}_{target:+g}'
            p.command('clear')
            def check(frame, x):
                if abs(x[6]) > 120 or abs(x[2]) > 2.05 or abs(x[9]) > 3.5:
                    raise RuntimeError('Motion left the free-shaft trial limits')
                if args.stage == 'position' and abs(x[4]) > 110:
                    raise RuntimeError('Position excursion exceeded trial limit')
            command = 'rpm' if args.stage == 'speed' else 'pos'
            x, stat = p.capture(name, 3, args.seconds, f'{command} {target:g}', check)
            stat.update(metrics(x, args.stage, target))
            p.save(name, stat)
            p.idle(name + '_stop')
        p.command('send 6')
    finally:
        p.close()


if __name__ == '__main__':
    main()
