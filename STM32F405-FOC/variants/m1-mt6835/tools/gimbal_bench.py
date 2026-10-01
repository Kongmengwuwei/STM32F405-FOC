"""Bounded USB checks for the installed yaw/pitch gimbal; always sends stop.

No automatic retries, resetting of position zero, or enlargement of limits.
Use a current-limited supply. Position checks require valid calibration.
"""
import argparse
import csv
import json
import math
import time
from pathlib import Path

from dual_smoke import Frames


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("action", choices=("idle", "clear", "cal-m0", "cal-m1", "pos-m0", "pos-m1", "pos-m1-positive", "pos-both", "return-m1"))
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--settle-seconds", type=float, default=8.0)
    args = parser.parse_args()
    if not 4.0 <= args.settle_seconds <= 10.0:
        parser.error("settle-seconds must be within 4..10")
    args.out.mkdir(parents=True, exist_ok=True)
    link = Frames(args.port)
    start = time.monotonic()
    rows = 0
    peak = [0.0, 0.0]
    bus_range = [math.inf, -math.inf]
    positions = [[math.inf, -math.inf], [math.inf, -math.inf]]
    last = None
    result = {"action": args.action, "port": args.port, "passed": False}
    stage = "idle"
    try:
        with (args.out / (args.action + ".csv")).open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["host_seconds", "stage"] + [f"m{i}_{n}" for i in range(2) for n in
                ("rpm_target", "rpm", "pos_target", "pos", "iq_target", "iq", "id", "bus", "mode", "state", "fault", "warnings")])

            def receive(duration, idle=False, expected=None):
                nonlocal rows, last
                started = time.monotonic()
                deadline = started + duration
                states_seen = [set(), set()]
                accepted = expected is None
                errors = []
                while time.monotonic() < deadline:
                    frame = link.read()
                    if frame is None:
                        # Sector 11 erase stalls this MCU's Flash execution.
                        # Calibration gates are already off before saving;
                        # permit only the bounded expected completion gap.
                        saving = stage.startswith("cal-") and time.monotonic() - started >= 11.5
                        timeout = 2.0 if saving else 0.5
                        if time.monotonic() - link.last_frame > timeout:
                            raise RuntimeError("USB telemetry lost for 0.5 seconds")
                        continue
                    last = frame
                    rows += 1
                    writer.writerow([time.monotonic() - start, stage] + list(frame[:24]))
                    if not all(math.isfinite(x) for x in frame[:24]):
                        raise RuntimeError("non-finite telemetry")
                    bus_range[0] = min(bus_range[0], frame[7])
                    bus_range[1] = max(bus_range[1], frame[7])
                    if not 8.0 <= frame[7] <= 12.4:
                        raise RuntimeError(f"bus outside 8.0..12.4 V: {frame[7]:.3f}")
                    for i, boundary in enumerate((math.inf, 89.0)):
                        at = i * 12
                        current = math.hypot(frame[at + 5], frame[at + 6])
                        peak[i] = max(peak[i], current)
                        positions[i][0] = min(positions[i][0], frame[at + 3])
                        positions[i][1] = max(positions[i][1], frame[at + 3])
                        states_seen[i].add(int(frame[at + 9]))
                        speed_ceiling = 60.0 if i == 0 and stage == "cal-m0" else 10.0
                        if i == 1 and stage == "cal-m1" and int(frame[at + 9]) == 2:
                            speed_ceiling = 30.0
                        if current > 1.2 or abs(frame[at + 1]) > speed_ceiling:
                            raise RuntimeError(f"M{i} current/speed: {current:.3f} A, {frame[at + 1]:.2f} RPM")
                        if abs(frame[at + 3]) > boundary:
                            raise RuntimeError(f"M{i} position near physical boundary: {frame[at + 3]:.2f}")
                        if frame[at + 10] or int(frame[at + 9]) == 5:
                            errors.append(f"M{i} state={int(frame[at + 9])}, fault={int(frame[at + 10])}, warnings={int(frame[at + 11])}")
                        if idle and int(frame[at + 9]) not in (0, 6, 5):
                            raise RuntimeError(f"M{i} did not stop")
                    if errors and args.action != "idle" and not (args.action == "clear" and stage == "idle"):
                        raise RuntimeError("; ".join(errors))
                    if expected is not None:
                        accepted |= all(int(frame[i * 12 + 8]) == 2 and int(frame[i * 12 + 9]) == 4 and
                            abs(frame[i * 12 + 2] - target) < 0.05 for i, target in expected.items())
                        for i, target in expected.items():
                            if abs(frame[i * 12 + 3] - target) > 10.0:
                                raise RuntimeError(f"M{i} escaped the small-angle test window")
                    if stage.startswith("cal"):
                        selected = 0 if stage == "cal-m0" else 1
                        other = 1 - selected
                        if int(frame[other * 12 + 9]) != 0:
                            raise RuntimeError("other motor is not idle during calibration")
                        if 2 in states_seen[selected] and int(frame[selected * 12 + 9]) == 0:
                            return states_seen
                    if rows % 2000 == 0:
                        print(f"{stage}: pos={frame[3]:.2f}/{frame[15]:.2f} deg, iq={frame[5]:.3f}/{frame[17]:.3f} A, states={int(frame[9])}/{int(frame[21])}", flush=True)
                if not accepted:
                    raise RuntimeError("position command was not accepted into RUN")
                if expected is not None:
                    for i, target in expected.items():
                        if abs(last[i * 12 + 3] - target) > 1.0:
                            raise RuntimeError(f"M{i} position did not settle within 1 degree: {last[i * 12 + 3]:.2f}, target={target:.2f}")
                return states_seen

            def fresh_command(command):
                link.serial.reset_input_buffer()
                link.buffer.clear()
                link.command(command)

            fresh_command("stop")
            if args.action == "clear":
                link.command("clear")
            receive(1.0, idle=True)
            result["initial"] = list(last[:24])
            print("initial " + json.dumps(result["initial"]), flush=True)
            if args.action == "idle":
                result["passed"] = last[10] == last[22] == 0 and int(last[9]) == int(last[21]) == 0
                return
            if int(last[9]) != 0 or int(last[21]) != 0:
                raise RuntimeError("both axes must be idle before movement")
            if args.action.startswith("cal-"):
                stage = args.action
                selected = 0 if stage == "cal-m0" else 1
                if abs(last[selected * 12 + 3]) > 5.0:
                    raise RuntimeError("calibrate only near the original neutral position")
                fresh_command(f"m{selected} cal")
                seen = receive(15.0)
                if 2 not in seen[selected] or int(last[selected * 12 + 9]) != 0:
                    raise RuntimeError("calibration did not complete")
            elif args.action.startswith("pos-") or args.action == "return-m1":
                returning = args.action == "return-m1"
                selected = (0, 1) if args.action == "pos-both" else (0,) if args.action == "pos-m0" else (1,)
                start_window = 10.0 if returning else 5.0
                if any(abs(last[i * 12 + 3]) > start_window for i in selected):
                    raise RuntimeError(f"start position must be within {start_window:.0f} degrees of neutral")
                angles = (0.0,) if returning else (3.0, 0.0) if args.action == "pos-m1-positive" else (3.0, -3.0, 0.0)
                for angle in angles:
                    stage = f"{args.action}:{angle:+.0f}deg"
                    link.serial.reset_input_buffer()
                    link.buffer.clear()
                    for i in selected:
                        link.command(f"m{i} pos {angle:.0f}")
                    receive(args.settle_seconds, expected={i: angle for i in selected})
            result["passed"] = True
    except Exception as error:
        result["error"] = str(error)
        raise
    finally:
        try:
            try:
                # Wait for the firmware to report gates off before closing USB.
                # Serial flush alone only confirms host-side transmission.
                link.command("stop")
                stopped = False
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    frame = link.read()
                    if frame is not None and all(int(frame[i]) in (0, 5, 6) for i in (9, 21)):
                        stopped = True
                        break
                result["stop_confirmed"] = stopped
            finally:
                link.close()
        finally:
            result.update(frames=rows, elapsed=time.monotonic() - start, peak_current=peak,
                          bus_range=bus_range, position_range=positions, last=None if last is None else list(last[:24]))
            (args.out / (args.action + ".json")).write_text(json.dumps(result, indent=2), encoding="utf-8")
            print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
