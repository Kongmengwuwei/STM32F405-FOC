"""Capture supported, gate-off five-bar endpoints; never commands motor motion."""

import argparse
from datetime import datetime
import json
from pathlib import Path
import struct
import time
import numpy as np
import serial


def read(s, seconds=1.0):
    s.write(b"send 2\r\n")
    s.flush()
    time.sleep(0.15)
    s.reset_input_buffer()
    buf = bytearray()
    rows = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        buf.extend(s.read(max(1, min(s.in_waiting, 65536))))
        while len(buf) >= 100:
            if buf[96:100] != b"\x00\x00\x80\x7f":
                del buf[0]
                continue
            x = struct.unpack("<24f", buf[:96])
            del buf[:100]
            if x[23] == 2:
                rows.append(x)
    a = np.asarray(rows)
    if len(a) < 100 or not np.isfinite(a).all():
        raise RuntimeError("Missing fresh teach protocol data")
    if np.any(a[:, 20]) or np.any(a[:, 9:11]) or np.any(a[:, 22] != 1):
        raise RuntimeError("Teach invalid, faulted, or bridge not off")
    if np.any(a[:, 15] != 7) or np.any(a[:, 16] != 1):
        raise RuntimeError("Unknown gear ratio/protocol")
    # A USB loss does not lose encoder turns: the MCU tracks at 10 kHz.
    # Frozen acquisition must never qualify as a present endpoint.
    sequence = np.mod(np.diff(a[:, 7]), 2**24)
    if np.any(sequence <= 0) or np.any(sequence > 100):
        raise RuntimeError("Stale acquisition or excessive telemetry gap")
    if np.ptp(a[:, :2], axis=0).max() > 0.05:
        raise RuntimeError("Support the rods and hold the endpoint still")
    return a


def raw_mean(a):
    radians = np.deg2rad(a[:, 4:6])
    return np.mod(
        np.rad2deg(
            np.arctan2(np.sin(radians).mean(axis=0), np.cos(radians).mean(axis=0))
        ),
        360,
    )


def complete(record, a, margin):
    anchor = np.asarray(record["teach"]["motor_anchor_deg"])
    if np.max(np.abs(a[:, 13:15] - anchor)) > 0.0001:
        raise RuntimeError("Teach anchors changed; do not infer missing turns")
    delta = a[:, :2].mean(axis=0)
    if np.min(np.abs(delta)) < 1 or np.max(np.abs(delta)) > 120:
        raise RuntimeError("Endpoint travel outside 1..120 joint degrees")
    if np.min(np.abs(delta)) <= 2 * margin:
        raise RuntimeError("Safety margins consume the full travel")
    signs = np.sign(delta).astype(int)
    review = bool(np.max(np.abs(delta)) > 110)
    record["status"] = (
        "both_boundaries_recorded_requires_range_confirmation"
        if review
        else "both_boundaries_recorded_not_enabled_for_drive"
    )
    record["boundary_B"] = {
        "timestamp": datetime.now().astimezone().isoformat(),
        "signed_joint_deg_from_A": delta.tolist(),
        "motor_unwrapped_deg": (anchor + 7 * delta).tolist(),
        "encoder_deg": raw_mean(a).tolist(),
        "joint_travel_deg": np.abs(delta).tolist(),
        "encoder_sign_toward_B": signs.tolist(),
        "candidate_joint_interval_positive_toward_B_deg": [
            [margin, float(abs(d) - margin)] for d in delta
        ],
        "margin_deg_each_end": margin,
        "requires_range_confirmation": review,
        "limits_enabled_for_drive": False,
        "angles_assume_confirmed_gear_ratio": 7,
        "physical_clockwise_direction": "not defined until viewing side is specified",
        "frames": len(a),
    }
    record["joint_coordinate"] = (
        "q_i = encoder_sign_toward_B[i] * (unwrapped_motor_deg_i - motor_anchor_deg_i) / 7; q=0 at downward A"
    )
    return record


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default="COM8")
    p.add_argument("--record", type=Path, required=True)
    action = p.add_mutually_exclusive_group(required=True)
    action.add_argument("--begin", action="store_true")
    action.add_argument("--finish", action="store_true")
    action.add_argument("--read", action="store_true")
    p.add_argument("--margin", type=float, default=2)
    args = p.parse_args()
    if not 0 <= args.margin <= 10:
        p.error("Margin must be between 0 and 10 degrees")
    record = json.loads(args.record.read_text(encoding="utf-8"))
    if args.begin and record.get("teach"):
        raise RuntimeError("A is already taught; do not silently reset the session")
    if args.finish and (not record.get("teach") or record.get("boundary_B")):
        raise RuntimeError("Need a pending A teach session")
    s = serial.Serial(args.port, 1000000, timeout=0.005, write_timeout=0.2)
    s.dtr = True
    try:
        s.write(b"stop\r\n")
        if args.begin:
            s.write(b"leg teach\r\n")
        a = read(s)
        if args.begin:
            # Confirm user is still at the endpoint captured before flashing.
            difference = (a[-1, 4:6] - record["encoder_deg_mean"] + 180) % 360 - 180
            if np.max(np.abs(difference)) > 1:
                raise RuntimeError(
                    "Pose moved since initial A; restore vertical reference"
                )
            record["teach"] = {
                "timestamp": datetime.now().astimezone().isoformat(),
                "motor_anchor_deg": a[-1, 13:15].tolist(),
                "raw_motor_deg_mean": raw_mean(a).tolist(),
                "joint_delta_deg_mean": a[:, :2].mean(axis=0).tolist(),
                "frames": len(a),
                "power_cycle_invalidates_multi_turn_session": True,
                "drive_interlock": "latched gate-off; no exit except MCU restart",
            }
        elif args.finish:
            record = complete(record, a, args.margin)
        if args.begin or args.finish:
            args.record.write_text(
                json.dumps(record, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            np.save(args.record.with_suffix(".A.npy" if args.begin else ".B.npy"), a)
        print(
            json.dumps(
                {
                    "joint_delta_deg": a[-1, :2].tolist(),
                    "motor_raw_deg": a[-1, 4:6].tolist(),
                    "motor_anchor_deg": a[-1, 13:15].tolist(),
                    "teach_state": int(a[-1, 22]),
                    "frames": len(a),
                    "status": record["status"],
                }
            )
        )
    finally:
        # Stop is deliberately not a teach exit; count tracking remains active.
        s.write(b"stop\r\nsend 1\r\n")
        s.flush()
        s.close()


if __name__ == "__main__":
    main()
