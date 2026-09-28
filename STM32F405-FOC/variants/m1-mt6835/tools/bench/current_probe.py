"""Record standby or bounded, small current steps on the calibrated M0 bench.

Never issues cal/rpm/pos or changes firmware limits. Stops before and after
every record; --steps explicitly enables motion. Serial must be free of VOFA.
"""
import argparse
import json
from pathlib import Path
import struct
import time

import numpy as np
import serial

from benchlib import FRAME_BYTES, FrameReader, continuity


def collect(link, group, seconds, events=()):
    link.write(f"send {group}\n".encode())
    # Drain the previous group before decoding 52-byte frames. Group 6 has a
    # different length; its trailing words can otherwise mimic an old header.
    drain_until = time.monotonic() + 0.05
    while time.monotonic() < drain_until:
        link.read(max(1, min(link.in_waiting, 65536)))
    reader = FrameReader()
    records = []
    host_events = []
    first = None
    deadline = time.monotonic() + seconds + 2
    event_index = 0
    last = time.monotonic()
    while first is None or time.monotonic() - first < seconds:
        now = time.monotonic()
        if now > deadline or now - last > 0.5:
            raise RuntimeError("Telemetry stalled or requested group missing")
        if first is not None:
            while event_index < len(events) and now - first >= events[event_index][0]:
                at, command = events[event_index]
                link.write((command + "\n").encode())
                host_events.append({"requested_s": at, "sent_s": now - first, "command": command})
                event_index += 1
        for frame in reader.feed(link.read(max(1, min(link.in_waiting, 65536)))):
            if frame.group != group:
                if first is not None:
                    raise RuntimeError("Group changed during record")
                continue
            if first is None:
                first = time.monotonic()
            last = time.monotonic()
            values = np.frombuffer(frame.body, dtype="<f4")
            if not np.all(np.isfinite(values[2:])) or frame.fault:
                raise RuntimeError(f"Invalid telemetry / fault {frame.fault}")
            if group == 5 and frame.state == 4:
                # Abort the measurement if it leaves the low-speed comparison
                # region. This is a host experiment bound, not a firmware limit.
                if abs(values[7]) > 300 or not 8 <= values[11] <= 12.5:
                    raise RuntimeError("Measurement left low-speed / supply comparison region")
            records.append(frame.body + struct.pack("<f", float("inf")))
    return b"".join(records), host_events, reader.skipped


def summary(raw, group, usb_divider):
    words = np.frombuffer(raw, dtype="<u4").reshape(-1, 13)
    values = words.view("<f4")
    status = words[:, 0] >> 24
    dt = np.diff(words[:, 0] & 0xFFFFFF).astype(np.int64) & 0xFFFFFF
    t = np.r_[0, np.cumsum(dt)] / 1e6
    result = {"frames": len(words), "duration_s": float(t[-1]),
              "sample_hz": 10000, "usb_divider": usb_divider,
              "continuity": continuity(words[:, :12], group, 10000, usb_divider),
              "states": {str(int(k)): int(v) for k, v in zip(*np.unique(status & 7, return_counts=True))}}
    if group == 0:
        result.update(bus_mean=float(values[:, 7].mean()),
                      angle_min=float(values[:, 8].min()), angle_max=float(values[:, 8].max()),
                      b_std_v=float(values[:, 5].std()), c_std_v=float(values[:, 6].std()))
    if group == 1:
        theta = np.deg2rad(values[:, 4])
        ia = -values[:, 2] - values[:, 3]
        beta = (values[:, 2] - values[:, 3]) / np.sqrt(3)
        for name, x in {"ib": values[:, 2], "ic": values[:, 3],
                        "id": ia*np.cos(theta)+beta*np.sin(theta),
                        "iq": -ia*np.sin(theta)+beta*np.cos(theta)}.items():
            result[name] = {"mean": float(x.mean()), "std": float(x.std()), "peak_abs": float(np.abs(x).max())}
    if group == 5:
        run = (status & 7) == 4
        result.update(max_abs_rpm=float(np.abs(values[:, 7]).max()),
                      min_bus_v=float(values[:, 11].min()), warnings=int(values[-1, 10]),
                      saturated_samples=int(np.sum(run & (values[:, 8] < 0.999) &
                                            (np.hypot(values[:, 5], values[:, 6]) > 1e-6))))
        edges = np.r_[0, np.flatnonzero(np.diff(values[:, 2]) != 0) + 1, len(values)]
        segments = []
        for start, end in zip(edges[:-1], edges[1:]):
            mask = run[start:end]
            if mask.sum() < 30:
                continue
            # Exclude first 30 ms; count and preserve raw samples separately.
            stable = np.arange(start, end)[mask & (t[start:end] >= t[start] + 0.030)]
            if len(stable) < 20:
                continue
            error = values[stable, 9] - values[stable, 2]
            block = 10 // usb_divider
            n = len(error) // block * block
            avg_error = error[:n].reshape(-1, block).mean(axis=1)
            segments.append({"start_s": float(t[start]), "target_a": float(values[start, 2]),
                "mean_iq_a": float(values[stable, 9].mean()), "mean_id_a": float(values[stable, 4].mean()),
                "bias_a": float(error.mean()), "raw_rms_a": float(np.sqrt(np.mean(error**2))),
                "rms_1ms_a": float(np.sqrt(np.mean(avg_error**2))),
                "uq_mean_v": float(values[stable, 6].mean()), "raw_samples": len(stable)})
        result["segments"] = segments
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("port")
    p.add_argument("--out", required=True)
    p.add_argument("--steps", action="store_true")
    p.add_argument("--label", default="baseline")
    p.add_argument("--usb-divider", type=int, default=2, choices=(1, 2))
    p.add_argument("--repeat", type=int, default=2, choices=(1, 2))
    p.add_argument("--kp", type=float, help="record the gain already flashed; does not change it")
    p.add_argument("--ki", type=float, help="record the Ki/s already flashed; does not change it")
    args = p.parse_args()
    if args.steps and (args.kp is None or args.ki is None):
        p.error("--steps requires --kp and --ki describing the flashed firmware")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    results = []
    with serial.Serial(args.port, 1000000, timeout=0.003, write_timeout=0.2) as link:
        try:
            link.write(b"stop\n")
            link.dtr = False
            end = time.monotonic() + 0.3
            while time.monotonic() < end:
                link.read(max(1, link.in_waiting))
            link.dtr = True

            def record(name, group, seconds, events=()):
                raw, sent, skipped = collect(link, group, seconds, events)
                (out / f"{args.label}_{name}.f32").write_bytes(raw)
                stats = summary(raw, group, args.usb_divider)
                stats.update(name=name, group=group, events=sent, resync_skipped_bytes=skipped)
                stats.update(current_kp=args.kp, current_ki_per_s=args.ki)
                results.append(stats)
                (out / f"{args.label}_summary.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
                print(json.dumps(stats), flush=True)
                if stats["continuity"]["gaps"]:
                    raise RuntimeError("Discontinuous capture; do not compare controller gains")
                return stats

            preflight = record("standby_raw", 0, 2)
            if not 8 <= preflight["bus_mean"] <= 12.5 or preflight["states"] != {"0": preflight["frames"]}:
                raise RuntimeError("Not a powered, valid idle baseline")
            record("standby_current", 1, 2)
            if args.steps:
                for repeat in range(args.repeat):
                    for sign in (1, -1):
                        link.write(b"stop\nclear\n")
                        record(f"settle_{repeat}_{sign}", 5, 0.4)
                        events = [(0.1, f"Iq {sign*0.05:.2f}"), (0.22, f"Iq {sign*0.10:.2f}"),
                                  (0.34, f"Iq {sign*0.20:.2f}"), (0.46, "Iq 0"), (0.58, "stop")]
                        record(f"step_{repeat}_{sign}", 5, 0.85, events)
                        link.write(b"stop\n")
            record("final_idle", 4, 0.5)
        finally:
            try:
                link.write(b"stop\n")
                time.sleep(0.05)
                link.dtr = False
            except serial.SerialException:
                print("STOP transmission failed; disconnect motor supply", flush=True)
                raise


if __name__ == "__main__":
    main()
