"""Supported, synchronous two-leg tests in the explicitly homed 0..110 degree workspace."""

import argparse, json, time
from pathlib import Path
import numpy as np
from serial import SerialException
from dual_leg import Link

HOME = 4194304


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--port", default="COM8")
    p.add_argument(
        "--home",
        action="store_true",
        help="Operator has physically placed both axes at the reliable stop",
    )
    p.add_argument("--clear", action="store_true", help="Operator restored supported pose after a UART/speed trip")
    p.add_argument("--converge-start", action="store_true", help="Begin at 10 RPM toward the live mean of both joints")
    p.add_argument("--targets", nargs="+", type=float, default=[2, 5, 10])
    p.add_argument("--limit", type=float, default=10)
    p.add_argument("--seconds", type=float, default=4)
    p.add_argument("--trajectory-rpm", type=float, default=10,
                   help="Current firmware trajectory ceiling in motor RPM")
    p.add_argument("--rates", nargs="+", type=float, help="Per-target motor RPM ceilings, 10..60")
    a = p.parse_args()
    if (
        not 0.2 <= a.limit <= 10
        or not 2 <= a.seconds <= 10
        or not 10 <= a.trajectory_rpm <= 60
        or any(not 2 <= q <= 108 for q in a.targets)
        or (a.rates is not None and (len(a.rates)!=len(a.targets) or any(not 10<=r<=60 for r in a.rates)))
    ):
        p.error("Native experiment targets 2..108, duration 2..10, current .2..10, trajectory 10..60 RPM")
    a.out.mkdir(parents=True, exist_ok=True)
    for retry in range(20):
        try:
            link = Link(a.port)
            break
        except SerialException as error:
            if "FileNotFoundError" not in str(error) or retry == 19:
                raise
            time.sleep(0.1)  # MCU reset briefly re-enumerates USB CDC.
    batches = []
    report = {"targets": [], "limit_A": a.limit, "trajectory_motor_rpm": a.trajectory_rpm}
    rows = []
    q = None
    try:
        pre = link.raw_rows(0.4)
        if a.clear and len(pre) and pre[-1,20]!=0:
            if pre[-1,20] not in (7,11) or pre[-1,22]!=3 or np.any(pre[:,9:11]):
                raise RuntimeError("Only supported live-reference UART/speed trips may be explicitly cleared")
            report["before_explicit_clear"] = pre[-1].tolist()
            link.send("clear")
            link.select(1)
            pre=link.raw_rows(.4)
        if (
            len(pre) < 100
            or np.any(pre[:, 20])
            or np.any(pre[:, 9:11])
            or np.any(pre[:, 11:13] != 1)
        ):
            raise RuntimeError("Need fault-free, calibrated, gate-off axes")
        if np.any(np.mod(np.diff(pre[:, 7]), 2**24) <= 0):
            raise RuntimeError("Encoder acquisition is stale")
        if a.home:
            if pre[-1, 22] not in (0, 3):
                raise RuntimeError(
                    "Cannot home from invalid teaching/acquisition state"
                )
            if pre[-1, 22] == 3 and np.max(abs(pre[:, :2])) > 2:
                raise RuntimeError(
                    "Tracked pose is not the reliable stop; do not redefine turns"
                )
            link.send("leg home")
            link.select(1)
            pre = link.raw_rows(0.3)
        if not len(pre) or np.any(pre[:, 22] != 3):
            raise RuntimeError("Explicit known-stop home not accepted")
        if np.any((pre[:, :2]<-1)|(pre[:, :2]>111)):
            raise RuntimeError("Restore a supported pose inside the live workspace")
        if a.converge_start:
            warm=float(np.clip(round(float(pre[-1,:2].mean()),2),2,108))
            if a.rates is None:a.rates=[a.trajectory_rpm]*len(a.targets)
            a.targets.insert(0,warm)
            a.rates.insert(0,10)
        initial_skew=float(abs(pre[-1,0]-pre[-1,1]))
        rates=a.rates if a.rates is not None else [a.trajectory_rpm]*len(a.targets)
        if initial_skew>5 and (initial_skew>7 or rates[0]!=10 or
                not min(pre[-1,:2])-.25<=a.targets[0]<=max(pre[-1,:2])+.25):
            raise RuntimeError("Initial unequal pose requires a slow converging target")
        report["preflight"] = pre[-1].tolist()
        np.save(a.out / "preflight.npy", pre)
        link.send(f"limit {a.limit:.2f}")
        time.sleep(0.03)
        check = link.raw_rows(0.1)
        if not len(check) or abs(check[-1, 2] - a.limit) > 0.01:
            raise RuntimeError("Limit command not accepted")
        link.select(0)
        previous = float(pre[-1, :2].mean())
        for stage, q in enumerate(a.targets):
            rate = a.rates[stage] if a.rates is not None else a.trajectory_rpm
            link.send(f"leg rate {rate:.2f}")
            duration = max(a.seconds, abs(q - previous) / (rate * 6 / 7) + 2.5)
            link.deadline(duration + 2)
            end = time.monotonic() + duration
            next_command = 0
            rows = []
            while time.monotonic() < end:
                if time.monotonic() >= next_command:
                    link.send(f"leg pos {q:.2f} {q:.2f}")
                    next_command = time.monotonic() + 0.04
                x = link.raw_rows(0.04)
                if not len(x):
                    raise RuntimeError("No main telemetry")
                rows.append(x)
                if np.any(x[:, [10, 22]]):
                    raise RuntimeError(f"Firmware fault {x[-1,[10,22]].tolist()}")
                if np.any((x[:, [11, 23]].astype(np.int64) & HOME) == 0):
                    raise RuntimeError("Lost multi-turn reference")
                if np.any((x[:, [3, 15]] < -1) | (x[:, [3, 15]] > 111)):
                    raise RuntimeError("Joint excursion")
                # Only the initial low-speed convergence may start >5 deg
                # apart; reject any further separation, then use normal 5 deg.
                skew_limit=max(5,initial_skew+.25) if stage==0 else 5
                if np.any(abs(x[:, 3] - x[:, 15]) > skew_limit):
                    raise RuntimeError("Synchronous joints separated by over 5 degrees")
                if np.any((x[:, [7, 19]] < 10.8) | (x[:, [7, 19]] > 13.6)):
                    raise RuntimeError("Supply out of native 12 V trial range")
                if (
                    np.max(np.hypot(x[:, 5], x[:, 6])) >= 11.0
                    or np.max(np.hypot(x[:, 17], x[:, 18])) >= 11.0
                ):
                    raise RuntimeError("Hard current threshold")
            x = np.concatenate(rows)
            batches.append(x)
            rows = []
            tail = x[-500:]
            mean = tail[:, [3, 15]].mean(axis=0)
            std = tail[:, [3, 15]].std(axis=0)
            result = {
                "goal_deg": q,
                "trajectory_motor_rpm": rate,
                "step_deg": abs(q-previous),
                "settled_time_0_5deg_s": (float(np.flatnonzero(np.any(abs(x[:,[3,15]]-q)>.5,axis=1))[-1]+1)/1000 if np.any(abs(x[:,[3,15]]-q)>.5) else 0),
                "duration_s": duration,
                "mean_deg": mean.tolist(),
                "error_deg": (mean - q).tolist(),
                "std_deg": std.tolist(),
                "max_abs_iq_A": abs(x[:, [5, 17]]).max(axis=0).tolist(),
                "max_abs_joint_rpm": abs(x[:, [1, 13]]).max(axis=0).tolist(),
                "max_pair_difference_deg": float(abs(x[:, 3] - x[:, 15]).max()),
                "bus_min_V": float(x[:, [7, 19]].min()),
                "frames": len(x),
            }
            report["targets"].append(result)
            np.save(a.out / "motion.npy", np.concatenate(batches))
            (a.out / "report.json").write_text(json.dumps(report, indent=2))
            print(json.dumps(result), flush=True)
            if np.max(abs(mean - q)) > 0.5 or np.max(std) > 0.15:
                raise RuntimeError("Settled tracking error/ripple outside acceptance")
            previous = q
        link.send("stop")
        link.select(1)
        final = link.raw_rows(0.3)
        if len(final):
            report["final"] = final[-1].tolist()
        np.save(a.out / "final.npy", final)
        if (
            not len(final)
            or np.any(final[:, 9:11])
            or np.any(final[:, 20])
            or np.any(final[:, 22] != 3)
        ):
            raise RuntimeError("Final stop/homing invalid")
        report["final"] = final[-1].tolist()
        report["passed"] = True
    except Exception as e:
        report["passed"] = False
        report["error"] = str(e)
        # Stop before diagnostic I/O; preserve the failing transition as well.
        link.send("stop")
        if rows:
            failed = np.concatenate(rows)
            np.save(a.out / "failed-stage.npy", failed)
            report["failed_goal_deg"] = q
            report["failed_frames"] = len(failed)
        link.select(1)
        diagnosis = link.raw_rows(0.3)
        np.save(a.out / "failure-diag.npy", diagnosis)
        if len(diagnosis):
            report["failure_status"] = diagnosis[-1].tolist()
        if batches:
            np.save(a.out / "motion.npy", np.concatenate(batches))
        raise
    finally:
        (a.out / "report.json").write_text(json.dumps(report, indent=2))
        link.close()


if __name__ == "__main__":
    main()
