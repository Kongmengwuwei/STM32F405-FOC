"""Record separately timed current, speed and position tests; always stop.

itest/stest terminate in firmware even if USB disappears. Position tests use
host timeout plus the firmware's physical travel/current/speed protections.
"""
import argparse
import csv
import json
import math
import statistics
import time
from pathlib import Path
from dual_smoke import Frames


def main():
    p = argparse.ArgumentParser()
    p.add_argument("port")
    p.add_argument("axis", type=int, choices=(0, 1))
    p.add_argument("loop", choices=("current", "speed", "position", "field"))
    p.add_argument("values", type=float, nargs="+")
    p.add_argument("--seconds", type=float, default=4.0)
    p.add_argument("--hold-other-zero", action="store_true",
                   help="position tests only: hold the other axis at zero and verify its settling")
    p.add_argument("--speed-limit-rpm", type=float, default=75.0,
                   help="host stop threshold; set to the tested firmware's independent hard trip")
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args()
    if a.hold_other_zero and a.loop != "position":
        p.error("--hold-other-zero requires a position test")
    if not 0.1 <= a.seconds <= 10 or not all(math.isfinite(v) for v in a.values):
        p.error("invalid duration or target")
    if not math.isfinite(a.speed_limit_rpm) or not 5 <= a.speed_limit_rpm <= 100:
        p.error("speed limit must be between 5 and 100 RPM")
    ceiling = {"current": .30, "speed": 2, "position": 85, "field": 270}[a.loop]
    if any(abs(v) > ceiling for v in a.values):
        p.error(f"test target exceeds {ceiling}")
    a.out.mkdir(parents=True, exist_ok=True)
    name = f"m{a.axis}-{a.loop}"
    link = Frames(a.port)
    origin = time.monotonic()
    results = {"axis": a.axis, "loop": a.loop, "values": a.values, "stages": [], "passed": False}
    results["hold_other_zero"] = a.hold_other_zero
    results["host_speed_limit_rpm"] = a.speed_limit_rpm
    data = []
    stage = "idle"
    initial = None
    high_current_frames = [0, 0]
    coast_rpm_peak = [0.0, 0.0]
    at = a.axis * 12
    try:
        with (a.out / (name + ".csv")).open("w", newline="") as out:
            writer = csv.writer(out)
            writer.writerow(["host_seconds", "stage"] + [f"m{i}_{n}" for i in range(2) for n in
                ("rpm_target", "rpm", "pos_target", "pos", "iq_target", "iq", "id", "bus", "mode", "state", "fault", "warnings")])

            def capture(seconds, allow_fault=False):
                batch = []
                until = time.monotonic() + seconds
                while time.monotonic() < until:
                    f = link.read()
                    if f is None:
                        if time.monotonic() - link.last_frame > .5:
                            raise RuntimeError("USB telemetry lost")
                        continue
                    t = time.monotonic() - origin
                    writer.writerow([t, stage] + list(f[:24]))
                    data.append((t, f))
                    batch.append((t, f))
                    if not all(math.isfinite(v) for v in f[:24]) or not 8 <= f[7] <= 12.4:
                        raise RuntimeError("invalid feedback or bus voltage")
                    for i in range(2):
                        b = 12 * i
                        if f[b + 10] or f[b + 9] == 5:
                            if allow_fault:
                                continue
                            raise RuntimeError(f"M{i} fault={f[b + 10]}")
                        speed_ceiling = 60 if a.loop == "field" and i == a.axis == 0 else a.speed_limit_rpm
                        drive_active = int(f[b+9]) in (1, 2, 4, 7)
                        if not drive_active:
                            coast_rpm_peak[i] = max(coast_rpm_peak[i], abs(f[b+1]))
                        current_vector = math.hypot(f[b+5], f[b+6])
                        # A 8.0 A reference is not an instantaneous current
                        # ceiling. Keep an absolute guard below the 10 A
                        # firmware phase trip, and reject sustained excess.
                        high_current_frames[i] = high_current_frames[i] + 1 if current_vector > 8.8 else 0
                        if current_vector > 9.5 or high_current_frames[i] >= 20 or (drive_active and abs(f[b+1]) > speed_ceiling):
                            raise RuntimeError(f"M{i} excessive current/speed: {current_vector:.3f} A, {f[b+1]:.2f} RPM, state={int(f[b+9])}")
                    if abs(f[15]) >= 88:
                        raise RuntimeError("M1 approaching travel boundary")
                    travel_window = max(abs(v - initial[at+3]) for v in a.values) + 5 if initial and a.loop == "position" else 25
                    if initial and stage != "idle" and abs(f[at+3] - initial[at+3]) > travel_window:
                        raise RuntimeError("escaped local test window")
                if not batch:
                    raise RuntimeError("no feedback")
                return batch

            def command(s):
                link.serial.reset_input_buffer()
                link.buffer.clear()
                link.command(s)

            command("stop")
            initial = capture(.7, allow_fault=True)[-1][1]
            if initial[10] or initial[22]:
                command("clear")
                initial = capture(.7, allow_fault=True)[-1][1]
            if initial[9] != 0 or initial[21] != 0:
                raise RuntimeError("axes not idle")
            if a.hold_other_zero:
                stage = "other-hold"
                command(f"m{1-a.axis} pos 0")
                held = capture(3.0)
                other_at = (1-a.axis) * 12
                if held[-1][1][other_at+9] != 4 or abs(held[-1][1][other_at+3]) > .2:
                    raise RuntimeError("other axis did not establish zero hold")
            for v in a.values:
                if abs(data[-1][1][at+1]) >= 5:
                    raise RuntimeError("shaft must settle below 5 RPM before another command")
                stage = f"{a.loop}:{v:+.2f}"
                cmd = {"current": "itest", "speed": "stest", "position": "pos", "field": "field"}[a.loop]
                command(f"m{a.axis} {cmd} {v:.2f}")
                samples = capture({"current": .6, "speed": 1.8, "position": a.seconds, "field": .45}[a.loop])
                active = [(t, f) for t, f in samples if f[at+9] == (2 if a.loop == "field" else 4)]
                if not active:
                    raise RuntimeError("test command rejected")
                tail = [f for t, f in active if t >= active[-1][0] - .10]
                report = {"target": v, "frames": len(active),
                    "iq_mean": statistics.mean(f[at+5] for f in tail),
                    "id_mean": statistics.mean(f[at+6] for f in tail),
                    "iq_error_rms": math.sqrt(statistics.mean((f[at+5]-f[at+4])**2 for f in tail)),
                    "id_rms": math.sqrt(statistics.mean(f[at+6]**2 for f in tail)),
                    "rpm_mean": statistics.mean(f[at+1] for f in tail),
                    "rpm_range": [min(f[at+1] for _, f in active), max(f[at+1] for _, f in active)],
                    "pos_final": active[-1][1][at+3],
                    "pos_range": [min(f[at+3] for _, f in active), max(f[at+3] for _, f in active)],
                    "iq_target_range": [min(f[at+4] for _, f in active), max(f[at+4] for _, f in active)]}
                if a.loop == "field":
                    # Telemetry projects onto the rotor's calibrated axes,
                    # while field() controls a fixed stator direction. Its
                    # iq_ref channel is not the 0.20 A field reference.
                    report["current_magnitude_mean"] = statistics.mean(math.hypot(f[at+5], f[at+6]) for f in tail)
                    report["current_magnitude_error_rms"] = math.sqrt(statistics.mean(
                        (math.hypot(f[at+5], f[at+6]) - .20)**2 for f in tail))
                    report["field_magnitude_tracking_passed"] = (
                        abs(report["current_magnitude_mean"] - .20) <= .04 and
                        report["current_magnitude_error_rms"] <= .05)
                    report["rotor_iq_mean"] = report.pop("iq_mean")
                    report["rotor_id_mean"] = report.pop("id_mean")
                    report.pop("iq_error_rms")
                    report.pop("iq_target_range")
                if a.loop == "current":
                    report["current_error_mean"] = report["iq_mean"] - v
                    report["current_tracking_passed"] = abs(report["current_error_mean"]) <= max(.02, .2 * abs(v)) and report["iq_error_rms"] <= max(.05, .2 * abs(v))
                if a.loop == "speed":
                    report["speed_error"] = report["rpm_mean"] - v
                    report["speed_tracking_passed"] = abs(report["speed_error"]) <= max(.1, .2 * abs(v))
                if a.loop == "position":
                    report["position_error"] = report["pos_final"] - v
                    end_t = active[-1][0]
                    steady = [f[at+3] for t, f in active if t >= end_t - .5]
                    report["last_half_second_range"] = [min(steady), max(steady)]
                    report["settle_0_2_deg_s"] = None
                    last_outside = max((t for t, f in active if abs(f[at+3]-v) > .2), default=active[0][0])
                    if abs(report["position_error"]) <= .2:
                        report["settle_0_2_deg_s"] = last_outside - active[0][0]
                    if a.hold_other_zero:
                        other_at = (1-a.axis) * 12
                        other_steady = [f[other_at+3] for t, f in active if t >= end_t - .5]
                        report["other_position_error"] = active[-1][1][other_at+3]
                        report["other_last_half_second_range"] = [min(other_steady), max(other_steady)]
                        report["other_hold_passed"] = (active[-1][1][other_at+9] == 4 and
                            abs(report["other_position_error"]) <= .2 and
                            max(other_steady) - min(other_steady) <= .2)
                results["stages"].append(report)
                print(json.dumps(report), flush=True)
                if a.loop == "position" and (abs(report["position_error"]) > .2 or
                        report["last_half_second_range"][1] - report["last_half_second_range"][0] > .2):
                    raise RuntimeError("position did not settle inside 0.2 degrees")
                if a.hold_other_zero and not report["other_hold_passed"]:
                    raise RuntimeError("other axis did not maintain a stable zero hold")
                if a.loop != "position":
                    if samples[-1][1][at+9] != 0:
                        raise RuntimeError("firmware diagnostic timeout did not stop")
                    stage = "idle"
                    command("stop")
                    capture(.8)
            results["diagnostic_completed"] = True
            if a.loop == "field" and not all(r["field_magnitude_tracking_passed"] for r in results["stages"]):
                raise RuntimeError("field current magnitude did not meet the 0.20 A mean/RMS criteria")
            if a.loop == "current" and not all(r["current_tracking_passed"] for r in results["stages"]):
                raise RuntimeError("current feedback did not meet the mean/RMS error criteria")
            if a.loop == "speed" and not all(r["speed_tracking_passed"] for r in results["stages"]):
                raise RuntimeError("speed tail mean did not settle within 20 percent or 0.1 RPM")
            results["passed"] = True
    except Exception as e:
        results["error"] = str(e)
        raise
    finally:
        results["stop_confirmed"] = False
        cleanup_errors = []
        try:
            link.command("stop")
            until = time.monotonic() + .5
            while time.monotonic() < until:
                f = link.read()
                if f and all(int(f[i]) in (0, 5, 6) for i in (9, 21)):
                    results["stop_confirmed"] = True
                    break
        except Exception as e:
            cleanup_errors.append(f"stop: {e}")
        try:
            link.close()
        except Exception as e:
            cleanup_errors.append(f"close: {e}")
        results["frames"] = len(data)
        results["coast_rpm_peak"] = coast_rpm_peak
        if cleanup_errors:
            results["cleanup_errors"] = cleanup_errors
        stop_failed = results["passed"] and not results["stop_confirmed"]
        if stop_failed:
            results["passed"] = False
            results["error"] = "drive shutdown not confirmed by telemetry"
        (a.out / (name + ".json")).write_text(json.dumps(results, indent=2), encoding="utf-8")
        print(json.dumps(results), flush=True)
        if stop_failed:
            raise RuntimeError(results["error"])


if __name__ == "__main__":
    main()
