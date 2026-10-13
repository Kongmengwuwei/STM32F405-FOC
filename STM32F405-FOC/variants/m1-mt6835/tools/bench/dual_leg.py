"""12 V raised wheel-leg bench, bounded calibration and position trials."""

import argparse, json, struct, time, threading
from pathlib import Path
import numpy as np
import serial


class Link:
    def __init__(self, port="COM8"):
        self.s = serial.Serial(port, 1000000, timeout=0.005, write_timeout=0.2)
        self.s.dtr = True
        self.buffer = bytearray()
        self.group = 0
        self.last = None
        self.guard = None
        self.send("stop")
        self.select(1)

    def send(self, c):
        self.s.write((c + "\r\n").encode())
        self.s.flush()

    def select(self, g):
        self.send("send " + str(g))
        self.group = g
        end = time.monotonic() + 0.2
        while time.monotonic() < end:
            self.s.read(max(1, min(self.s.in_waiting, 65536)))
        self.buffer.clear()
        self.last = None

    def rows(self, seconds=0.06, allow_silence=False):
        rows = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.buffer.extend(self.s.read(max(1, min(self.s.in_waiting, 65536))))
            while len(self.buffer) >= 100:
                if self.buffer[96:100] != b"\x00\x00\x80\x7f":
                    del self.buffer[0]
                    continue
                x = np.array(struct.unpack("<24f", self.buffer[:96]))
                del self.buffer[:100]
                if not np.isfinite(x).all():
                    raise RuntimeError("Non-finite telemetry")
                if bool(x[23] == 1) != bool(self.group):
                    continue
                bus = x[3] if self.group else x[7]
                if not 10.8 <= bus <= 13.6:
                    raise RuntimeError(f"Bus {bus:.3f} V out of experiment range")
                if self.group:
                    if abs(x[0]) > 3.7 or abs(x[1]) > 3.7:
                        raise RuntimeError(
                            "Mechanical displacement reached bench boundary"
                        )
                    if x[20] != 0:
                        raise RuntimeError(f"Firmware fault {x[20]}")
                    if (
                        self.last is not None
                        and (int(x[7]) - self.last) & 0xFFFFFF != 10
                    ):
                        raise RuntimeError("USB sequence gap")
                    self.last = int(x[7])
                else:
                    if x[10] != 0 or x[22] != 0:
                        raise RuntimeError(f"Firmware faults {x[10]}/{x[22]}")
                    if (
                        max(abs(x[1]), abs(x[13])) > 8.5
                        or max(
                            abs(x[4]),
                            abs(x[5]),
                            abs(x[6]),
                            abs(x[16]),
                            abs(x[17]),
                            abs(x[18]),
                        )
                        >= 9.5
                        or max(np.hypot(x[5], x[6]), np.hypot(x[17], x[18])) >= 9.5
                    ):
                        raise RuntimeError(
                            f"Current or speed outside experiment boundary: rpm={x[[1,13]].tolist()}, iq/id={x[[5,6,17,18]].tolist()}"
                        )
                rows.append(x)
        if not rows and not allow_silence:
            raise RuntimeError("No telemetry")
        return np.array(rows)

    def raw_rows(self, seconds=0.2):
        """Read fault/stop diagnostic data without suppressing a recorded fault."""
        data = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.buffer.extend(self.s.read(max(1, min(self.s.in_waiting, 65536))))
            while len(self.buffer) >= 100:
                if self.buffer[96:100] != b"\x00\x00\x80\x7f":
                    del self.buffer[0]
                    continue
                x = np.array(struct.unpack("<24f", self.buffer[:96]))
                del self.buffer[:100]
                if np.isfinite(x).all() and bool(x[23] == 1) == bool(self.group):
                    data.append(x)
        return np.array(data)

    def deadline(self, seconds):
        if self.guard:
            self.guard.cancel()
            self.guard.join()
        self.guard = threading.Timer(seconds, self.send, args=("stop",))
        self.guard.start()

    def close(self):
        try:
            self.send("stop")
            self.send("send 0")
            if self.guard:
                self.guard.cancel()
                self.guard.join()
        finally:
            self.s.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--out", type=Path, required=True)
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--cal", type=int, choices=[0, 1])
    parser.add_argument("--current", type=float, default=1)
    action.add_argument("--positions", nargs=2, type=float)
    action.add_argument(
        "--sequence",
        action="store_true",
        help="Continuous paired and individual joint trials; both bridges hold throughout",
    )
    action.add_argument(
        "--watchdog",
        action="store_true",
        help="Verify command silence shuts both bridges off, then clear fault",
    )
    parser.add_argument("--seconds", type=float, default=3)
    parser.add_argument("--limit", type=float, default=5)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    if (
        not 0.2 <= args.current <= 9
        or not 0.2 <= args.limit <= 9
        or not 0.3 <= args.seconds <= 3
    ):
        parser.error("Invalid current/duration")
    if args.positions and max(map(abs, args.positions)) > 2:
        parser.error("Bench targets limited to +/-2 joint degrees")
    link = Link(args.port)
    batches = []
    report = {}
    try:
        x = link.rows(0.3)
        np.save(args.out / "preflight.npy", x)
        if np.any(x[:, 9:11]):
            raise RuntimeError("Power output must be off before trial")
        report["preflight"] = x[-1].tolist()
        if args.cal is not None:
            if np.any(x[:, 11 + args.cal]):
                raise RuntimeError(
                    "This axis is already calibrated; use deliberate reset/recalibration workflow"
                )
            link.deadline(16)
            link.select(0)
            link.send(f"m{args.cal} cal {args.current:.2f}")
            end = time.monotonic() + 14.5
            active = False
            while time.monotonic() < end:
                x = link.rows(0.05, allow_silence=True)
                if not len(x):
                    continue
                batches.append(x)
                axis = x[:, args.cal * 12 : args.cal * 12 + 12]
                active = active or np.any((axis[:, 9] == 1) | (axis[:, 9] == 2))
                if active and time.monotonic() > end - 8 and axis[-1, 9] == 0:
                    break
            link.send("stop")
            link.select(1)
            status = link.rows(0.2)[-1]
            if not active or status[11 + args.cal] != 1:
                raise RuntimeError("Calibration did not produce a valid record")
            report["calibration"] = dict(
                axis=args.cal, current=args.current, status=status.tolist()
            )
        elif args.positions:
            if x[-1, 11] != 1 or x[-1, 12] != 1:
                raise RuntimeError("Both axes require validated calibration")
            link.send(f"limit {args.limit:.2f}")
            check = link.rows(0.05)[-1]
            if abs(check[2] - args.limit) > 0.001:
                raise RuntimeError("Current limit command rejected")
            link.select(0)
            link.deadline(args.seconds + 0.5)
            command = f"leg pos {args.positions[0]:.2f} {args.positions[1]:.2f}"
            end = time.monotonic() + args.seconds
            while time.monotonic() < end:
                link.send(command)
                x = link.rows(0.04)
                batches.append(x)
            x = np.concatenate(batches)
            tail = x[-200:]
            if not np.allclose(tail[:, [2, 14]], args.positions, atol=0.001):
                raise RuntimeError("Position target was rejected")
            report["position"] = dict(
                target=args.positions,
                limit=args.limit,
                mean=tail[:, [3, 15]].mean(axis=0).tolist(),
                error=(tail[:, [3, 15]] - args.positions).mean(axis=0).tolist(),
                rms=(tail[:, [3, 15]].std(axis=0)).tolist(),
                iq_peak=np.abs(x[:, [5, 17]]).max(axis=0).tolist(),
                warning=np.max(x[:, [11, 23]], axis=0).tolist(),
            )
            if (
                np.max(np.abs(report["position"]["error"])) > 0.25
                or np.max(report["position"]["rms"]) > 0.1
            ):
                raise RuntimeError(
                    "Position tracking did not meet 0.25 degree error / 0.1 degree stability requirement"
                )
        elif args.sequence:
            if x[-1, 11] != 1 or x[-1, 12] != 1:
                raise RuntimeError("Both axes require validated calibration")
            link.send(f"limit {args.limit:.2f}")
            check = link.rows(0.05)[-1]
            if abs(check[2] - args.limit) > 0.001:
                raise RuntimeError("Current limit command rejected")
            targets = [
                (0, 0),
                (1, 0),
                (0, 1),
                (-1, 0),
                (0, -1),
                (2, -2),
                (-2, 2),
                (0, 0),
            ]
            link.select(0)
            link.deadline(len(targets) * args.seconds + 1)
            report["sequence"] = []
            for target in targets:
                stage = []
                command = f"leg pos {target[0]:.2f} {target[1]:.2f}"
                end = time.monotonic() + args.seconds
                while time.monotonic() < end:
                    link.send(command)
                    data = link.rows(0.04)
                    stage.append(data)
                    batches.append(data)
                data = np.concatenate(stage)
                tail = data[-200:]
                result = dict(
                    target=target,
                    error=(tail[:, [3, 15]] - target).mean(axis=0).tolist(),
                    std=tail[:, [3, 15]].std(axis=0).tolist(),
                    iq_peak=np.abs(data[:, [5, 17]]).max(axis=0).tolist(),
                )
                report["sequence"].append(result)
                print(json.dumps(result), flush=True)
                if not np.allclose(tail[:, [2, 14]], target, atol=0.001):
                    raise RuntimeError("Paired target rejected")
                if (
                    np.max(np.abs(result["error"])) > 0.25
                    or np.max(result["std"]) > 0.1
                ):
                    raise RuntimeError("Continuous sequence tracking acceptance failed")
        elif args.watchdog:
            if x[-1, 11] != 1 or x[-1, 12] != 1:
                raise RuntimeError("Both axes require validated calibration")
            link.select(0)
            link.deadline(3)
            end = time.monotonic() + 1
            while time.monotonic() < end:
                link.send("leg pos 0 0")
                last_command = time.monotonic()
                batches.append(link.rows(0.04))
            observed = None
            end = time.monotonic() + 0.6
            while time.monotonic() < end:
                data = link.raw_rows(0.02)
                if len(data):
                    batches.append(data)
                    if np.any(data[:, 10] == 7) and np.any(data[:, 22] == 7):
                        observed = time.monotonic() - last_command
                        break
            if observed is None:
                raise RuntimeError("Both axes did not enter UART timeout fault")
            link.select(1)
            diagnosis = link.raw_rows(0.1)[-1]
            if diagnosis[20] != 7 or np.any(diagnosis[9:11]):
                raise RuntimeError("Timeout did not disable both PWM outputs")
            report["watchdog"] = dict(
                observed_after_last_command_s=observed, status=diagnosis.tolist()
            )
            link.send("clear")
            link.select(1)
            cleared = link.rows(0.7)[-1]
            if cleared[20] != 0 or cleared[11] != 1 or cleared[12] != 1:
                raise RuntimeError("Clear lost validated calibration")
        link.send("stop")
        link.select(1)
        report["stopped"] = link.rows(0.2)[-1].tolist()
        if np.any(np.array(report["stopped"])[9:11]):
            raise RuntimeError("PWM remains enabled")
        print(json.dumps(report), flush=True)
    except Exception as error:
        report["error"] = str(error)
        link.send("stop")
        link.select(1)
        diagnosis = link.raw_rows(0.2)
        if len(diagnosis):
            report["failure_status"] = diagnosis[-1].tolist()
        raise
    finally:
        try:
            link.send("stop")
            if batches:
                np.save(args.out / "motion.npy", np.concatenate(batches))
            (args.out / "report.json").write_text(json.dumps(report, indent=2))
        finally:
            link.close()


if __name__ == "__main__":
    main()
