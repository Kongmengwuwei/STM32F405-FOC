"""Bounded dual zero-hold recording for a supervised, gentle load test.

Run only after the operator agrees to apply small external loads. Both axes
hold zero; motion beyond the selected local window ends the test. This does not measure
external torque. Firmware travel, current and speed trips remain active.
"""
import argparse
import csv
import json
import math
import time
from pathlib import Path
from dual_smoke import Frames


def main():
    p = argparse.ArgumentParser()
    p.add_argument("port")
    p.add_argument("--seconds", type=float, default=45)
    p.add_argument("--speed-limit-rpm", type=float, default=75)
    p.add_argument("--travel-window-deg", type=float, default=5)
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args()
    if not 5 <= a.seconds <= 60 or not 5 <= a.speed_limit_rpm <= 100:
        p.error("duration/speed threshold outside the bounded test range")
    if not 2 <= a.travel_window_deg <= 15:
        p.error("local travel window must be 2..15 degrees")
    a.out.mkdir(parents=True, exist_ok=True)
    link = Frames(a.port)
    start = time.monotonic()
    rows, high = [], [0, 0]
    result = {"passed": False, "stop_confirmed": False, "events": [[], []],
              "local_travel_window_deg": a.travel_window_deg}
    stage = "idle"
    try:
        with (a.out / "hold.csv").open("w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["host_seconds", "stage"] + [f"m{i}_{n}" for i in range(2) for n in
                ("rpm_target", "rpm", "pos_target", "pos", "iq_target", "iq", "id", "bus", "mode", "state", "fault", "warnings")])

            def capture(seconds, powered=False):
                batch = []
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    f = link.read()
                    if f is None:
                        if time.monotonic() - link.last_frame > .5:
                            raise RuntimeError("telemetry lost")
                        continue
                    t = time.monotonic() - start
                    writer.writerow([t, stage] + list(f[:24]))
                    rows.append((t, f)); batch.append((t, f))
                    if not all(math.isfinite(x) for x in f[:24]) or not 8 <= f[7] <= 12.4:
                        raise RuntimeError("feedback/bus invalid")
                    for i in range(2):
                        b = 12 * i
                        if f[b+10] or int(f[b+9]) == 5:
                            raise RuntimeError(f"M{i} fault {f[b+10]}")
                        if abs(f[b+3]) > a.travel_window_deg:
                            raise RuntimeError(f"M{i} escaped local travel window")
                        if int(f[b+9]) in (1, 4, 7) and abs(f[b+1]) >= a.speed_limit_rpm:
                            raise RuntimeError(f"M{i} overspeed")
                        magnitude = math.hypot(f[b+5], f[b+6])
                        high[i] = high[i] + 1 if magnitude > 8.8 else 0
                        if magnitude > 9.5 or high[i] >= 20:
                            raise RuntimeError(f"M{i} excessive current")
                        if powered and (int(f[b+9]) != 4 or int(f[b+8]) != 2 or f[b+2] != 0):
                            raise RuntimeError(f"M{i} zero hold lost")
                file.flush()
                if not batch:
                    raise RuntimeError("no frames")
                return batch

            def command(text):
                link.serial.reset_input_buffer(); link.buffer.clear()
                link.command(text)

            command("stop")
            idle = capture(.7)[-1][1]
            if idle[9] != 0 or idle[21] != 0 or max(abs(idle[1]), abs(idle[13])) >= 5:
                raise RuntimeError("not stationary/idle")
            stage = "settle"
            command("gimbal pos 0 0")
            settled = capture(2)[-1][1]
            if settled[9] != 4 or settled[21] != 4 or max(abs(settled[3]), abs(settled[15])) > .2:
                raise RuntimeError("zero hold did not establish")
            stage = "external-load"
            print("READY: both axes holding zero; apply gentle loads, then release.", flush=True)
            samples = capture(a.seconds, powered=True)
            unstable = []
            for i in range(2):
                b = i * 12
                event = None
                stable_start = None
                for t, f in samples:
                    error = abs(f[b+3])
                    if error > .2:
                        stable_start = None
                        if event is None:
                            event = {"start_s": t, "last_outside_s": t, "peak_deg": error}
                        event["last_outside_s"] = t
                        event["peak_deg"] = max(event["peak_deg"], error)
                    elif event is not None:
                        if stable_start is None:
                            stable_start = t
                        if t - stable_start >= .2:
                            event["recovered_s"] = stable_start
                            event["time_outside_0_2_deg_s"] = stable_start - event["start_s"]
                            result["events"][i].append(event); event = None
                if event:
                    result["events"][i].append(event)
                end = samples[-1][0]
                steady = [f[b+3] for t, f in samples if t > end - 1]
                result[f"m{i}"] = {"peak_error_deg": max(abs(f[b+3]) for _, f in samples),
                    "final_error_deg": samples[-1][1][b+3],
                    "last_second_span_deg": max(steady) - min(steady),
                    "iq_target_range": [min(f[b+4] for _, f in samples), max(f[b+4] for _, f in samples)],
                    "observed_angle_disturbance": bool(result["events"][i])}
                if abs(steady[-1]) > .2 or max(steady) - min(steady) > .2:
                    unstable.append(f"M{i}")
            if unstable:
                raise RuntimeError(f"{', '.join(unstable)} final hold unstable")
            result["passed"] = True
    except Exception as error:
        result["error"] = str(error)
        raise
    finally:
        try:
            link.command("stop")
            link.serial.reset_input_buffer(); link.buffer.clear()
            end = time.monotonic() + .7
            while time.monotonic() < end:
                f = link.read()
                if f and int(f[9]) in (0, 5, 6) and int(f[21]) in (0, 5, 6):
                    result["stop_confirmed"] = True
                    break
        except Exception as error:
            result["stop_error"] = str(error)
        finally:
            link.serial.close()
        result["frames"] = len(rows)
        if not result["stop_confirmed"]:
            result["passed"] = False
        (a.out / "hold.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result), flush=True)
    if not result["passed"]:
        raise RuntimeError("hold/shutdown verification failed")


if __name__ == "__main__":
    main()
