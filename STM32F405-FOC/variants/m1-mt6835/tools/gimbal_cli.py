"""Local gimbal tool: JSON status, paired position commands and verified stop."""
import argparse
import json
import math
import sys
import time

from serial.tools import list_ports
from dual_smoke import Frames

BOARD_SERIAL = "385C38793335"
FIELDS = ("rpm_target", "rpm", "angle_target_deg", "angle_deg", "iq_target_a",
          "iq_a", "id_a", "bus_v", "mode", "state", "fault", "warnings")


def select_port(explicit=None):
    boards = [p for p in list_ports.comports() if (p.vid, p.pid) == (0x0483, 0x5740)]
    if explicit:
        matches = [p for p in boards if p.device.lower() == explicit.lower()]
    else:
        matches = [p for p in boards if p.serial_number == BOARD_SERIAL]
    if len(matches) != 1:
        raise RuntimeError("Cannot uniquely identify the gimbal USB CDC board; check USB or specify --port.")
    return matches[0].device


def angle(value):
    number = float(value)
    if not math.isfinite(number) or abs(number) > 85:
        raise ValueError("Angles must be finite and within -85..85 mechanical degrees.")
    if abs(number - round(number, 2)) > 1e-8:
        raise ValueError("Use at most two decimal places for angles.")
    return round(number, 2)


def unpack(frame):
    if len(frame) < 24 or not all(math.isfinite(v) for v in frame[:24]):
        raise RuntimeError("Invalid telemetry frame.")
    axes = []
    for axis in (0, 1):
        data = dict(zip(FIELDS, frame[axis * 12:axis * 12 + 12]))
        for name in ("mode", "state", "fault", "warnings"):
            if data[name] != int(data[name]):
                raise RuntimeError("Invalid telemetry state.")
            data[name] = int(data[name])
        data.update(motor=f"M{axis}", role="pitch" if axis == 0 else "yaw")
        axes.append(data)
    return {"axes": axes, "outputs_idle": all(a["state"] in (0, 5, 6) for a in axes)}


class Gimbal:
    """Own one serial session; close() stops unless explicitly told otherwise."""
    def __init__(self, link):
        self.link = link
        self.stop_confirmed = False
        self.high_current_frames = [0, 0]

    def fresh(self):
        self.link.serial.reset_input_buffer()
        self.link.buffer.clear()

    def next_frame(self, deadline):
        while time.monotonic() < deadline:
            frame = self.link.read()
            if frame is not None:
                return unpack(frame)
            if time.monotonic() - self.link.last_frame > .5:
                raise RuntimeError("USB telemetry lost for 0.5 seconds.")
        raise TimeoutError("Timed out waiting for telemetry.")

    def status(self, timeout=2):
        self.fresh()
        return self.next_frame(time.monotonic() + timeout)

    def check(self, data):
        for i, a in enumerate(data["axes"]):
            if a["fault"] or a["state"] == 5:
                raise RuntimeError(f'{a["motor"]} fault={a["fault"]}; do not automatically clear or reset.')
            if not 8 <= a["bus_v"] <= 12.4:
                raise RuntimeError("Bus voltage outside tested 8..12.4 V range.")
            if not 0 <= a["state"] <= 7 or not 0 <= a["mode"] <= 2:
                raise RuntimeError("Unexpected firmware telemetry layout.")
            if abs(a["angle_deg"]) >= 88:
                raise RuntimeError(f'{a["motor"]} approaching travel boundary.')
            current = math.hypot(a["iq_a"], a["id_a"])
            self.high_current_frames[i] = self.high_current_frames[i] + 1 if current > 8.8 else 0
            if current > 9.5 or self.high_current_frames[i] >= 20:
                raise RuntimeError(f'{a["motor"]} excessive measured current.')
            if a["state"] in (1, 2, 4, 7) and abs(a["rpm"]) >= 75:
                raise RuntimeError(f'{a["motor"]} excessive speed.')

    @staticmethod
    def matches(data, targets):
        return all(a["mode"] == 2 and a["state"] == 4 and
                   abs(a["angle_target_deg"] - target) < .01
                   for a, target in zip(data["axes"], targets))

    def move(self, pitch, yaw, timeout=8, tolerance=.2, settle=.3):
        targets = (angle(pitch), angle(yaw))
        if not .05 <= tolerance <= 1 or not .1 <= settle <= 2 or not 1 <= timeout <= 15:
            raise ValueError("Invalid movement timeout/tolerance/settle interval.")
        initial = self.status()
        self.check(initial)
        if any(a["state"] not in (0, 4) or (a["state"] == 4 and a["mode"] != 2)
               for a in initial["axes"]):
            raise RuntimeError("Axes are not ready for position commands.")
        if any(abs(a["rpm"]) >= 5 for a in initial["axes"]):
            raise RuntimeError("Wait until both shafts are stationary before commanding movement.")
        self.fresh()
        self.link.command(f"gimbal pos {targets[0]:.2f} {targets[1]:.2f}")
        self.stop_confirmed = False
        start = time.monotonic()
        stable_since = None
        accepted = False
        while time.monotonic() - start < timeout:
            data = self.next_frame(start + timeout)
            self.check(data)
            now = time.monotonic()
            if self.matches(data, targets):
                accepted = True
            elif accepted:
                raise RuntimeError("Position target or run state changed unexpectedly.")
            elif now - start > .5:
                raise RuntimeError("Position command was not accepted; verify calibration and coordinate limits.")
            inside = accepted and all(abs(a["angle_deg"] - t) <= tolerance
                                      for a, t in zip(data["axes"], targets))
            if inside:
                stable_since = now if stable_since is None else stable_since
                if now - stable_since >= settle:
                    return {"settled": True, "elapsed_s": now - start,
                            "tolerance_deg": tolerance, "telemetry": data}
            else:
                stable_since = None
        raise TimeoutError("Both axes did not settle before the movement timeout.")

    def watch(self, seconds, targets):
        if not .1 <= seconds <= 60:
            raise ValueError("Holding interval must be .1..60 seconds.")
        end = time.monotonic() + seconds
        last = None
        peak_error = [0.0, 0.0]
        while time.monotonic() < end:
            last = self.next_frame(end + .5)
            self.check(last)
            if not self.matches(last, targets):
                raise RuntimeError("Position hold target or run state changed.")
            for i, (a, target) in enumerate(zip(last["axes"], targets)):
                error = abs(a["angle_deg"] - target)
                peak_error[i] = max(peak_error[i], error)
                if error > 5:
                    raise RuntimeError("Axis escaped the local holding window.")
        return {"telemetry": last, "max_hold_error_deg": peak_error}

    def stop(self, timeout=2):
        self.stop_confirmed = False
        self.fresh()
        self.link.command("stop")
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            data = self.next_frame(end)
            if data["outputs_idle"]:
                self.stop_confirmed = True
                return data
        raise TimeoutError("Drive shutdown was not confirmed.")

    def close(self, stop=True):
        try:
            if stop:
                self.stop()
        finally:
            # Frames.close() always sends stop; status and explicit continued
            # holding must close only the transport.
            self.link.serial.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Normally auto-detect the known board; currently COM8")
    actions = parser.add_subparsers(dest="action", required=True)
    actions.add_parser("status", help="Read live status without sending motor commands")
    actions.add_parser("stop", help="Stop both axes and verify telemetry")
    move = actions.add_parser("move", help="Command M0 pitch and M1 yaw together")
    move.add_argument("--pitch", type=angle, required=True)
    move.add_argument("--yaw", type=angle, required=True)
    move.add_argument("--timeout", type=float, default=8)
    move.add_argument("--tolerance", type=float, default=.2)
    move.add_argument("--hold-seconds", type=float, default=1)
    move.add_argument("--keep-holding", action="store_true",
                      help="Leave BOTH axes enabled after success; later issue stop")
    args = parser.parse_args(argv)
    if args.action == "move" and ((args.hold_seconds != 0 and not .1 <= args.hold_seconds <= 60) or
                                  not 1 <= args.timeout <= 15 or not .05 <= args.tolerance <= 1):
        parser.error("timeout 1..15, tolerance .05..1, hold-seconds 0 or .1..60")
    controller = None
    result = {"ok": False, "action": args.action}
    if args.action != "status":
        result["stop_confirmed"] = False
    leave_running = False
    try:
        port = select_port(args.port)
        result["port"] = port
        controller = Gimbal(Frames(port))
        if args.action == "status":
            result["telemetry"] = controller.status()
        elif args.action == "stop":
            result["telemetry"] = controller.stop()
            result["stop_confirmed"] = controller.stop_confirmed
        else:
            result.update(controller.move(args.pitch, args.yaw, args.timeout, args.tolerance))
            if args.hold_seconds:
                result.update(controller.watch(args.hold_seconds, (args.pitch, args.yaw)))
                if any(abs(a["angle_deg"] - target) > args.tolerance
                       for a, target in zip(result["telemetry"]["axes"], (args.pitch, args.yaw))):
                    raise RuntimeError("Position drift exceeded tolerance at the end of holding.")
            leave_running = args.keep_holding
            result["holding_after_exit"] = leave_running
        result["ok"] = True
    except (Exception, KeyboardInterrupt) as error:
        result["error"] = "Interrupted by operator." if isinstance(error, KeyboardInterrupt) else str(error)
    finally:
        if controller:
            must_stop = args.action == "move" and (not result["ok"] or not leave_running)
            try:
                controller.close(stop=must_stop)
                if must_stop:
                    result["stop_confirmed"] = controller.stop_confirmed
            except Exception as error:
                result["ok"] = False
                result["holding_after_exit"] = False if controller.stop_confirmed else None
                result["cleanup_error"] = str(error)
                result["stop_confirmed"] = controller.stop_confirmed
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    sys.exit(main())
