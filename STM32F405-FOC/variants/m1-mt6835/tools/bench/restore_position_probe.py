"""Verify near-zero position recovery without ever enabling either motor.
Default only reads. --restore asserts the operator placed both joints within
0..40 degrees inward from the original reference and supports the linkage.
"""
import argparse
import json
import math
import struct
import time
from pathlib import Path
import serial

class Frames:
    def __init__(self):
        self.pending = bytearray()
    def feed(self, data):
        self.pending.extend(data)
        rows = []
        while len(self.pending) >= 100:
            if self.pending[96:100] != b"\x00\x00\x80\x7f":
                del self.pending[0]
                continue
            row = struct.unpack("<24f", self.pending[:96])
            del self.pending[:100]
            if row[23] in (2, 5005):
                rows.append(row)
        return rows

def expected(raw, reference, direction):
    phase = direction * ((raw - reference + 180) % 360 - 180) / 7
    candidates = [phase + turn * 360 / 7 for turn in (-1, 0, 1)
                  if -2 <= phase + turn * 360 / 7 <= 40]
    assert len(candidates) == 1, f"No unique near-zero candidate: {candidates}"
    return candidates[0]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--restore", action="store_true")
    parser.add_argument("--output", type=Path, default=Path("build/dual-leg/wheel-leg-ui/restore-probe-20261016.json"))
    args = parser.parse_args()
    result = {"restore_requested": args.restore, "motion_requested": False, "success": False}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    try:
        with serial.Serial(args.port, 1000000, timeout=.015) as link:
            link.dtr = True
            frames = Frames()
            def send(line):
                # Deliberately no API for motion, limits, clearing faults or redefining zero.
                assert line in ("send 5", "send 2", "leg restore")
                link.write((line + "\r\n").encode()); link.flush()
            def collect(tag, seconds):
                begin = len(rows); end = time.monotonic() + seconds
                while time.monotonic() < end:
                    for row in frames.feed(link.read(8192)):
                        rows.append(row)
                        if row[23] == 5005:
                            assert row[11] == row[12] == 0, "Motor not IDLE"
                            assert row[13] == 0, f"Fault {row[13]}"
                        else:
                            assert row[9] == row[10] == 0, "PWM bridge output active"
                            assert row[20] == 0, f"Fault {row[20]}"
                selected = [row for row in rows[begin:] if row[23] == tag]
                assert len(selected) >= 5, f"Insufficient telemetry: {len(selected)}"
                return selected[-1]
            send("send 5"); before = collect(5005, 1)
            result["before"] = before
            assert int(before[14]) & 1024, "Firmware lacks near-zero restore"
            send("send 2"); raw_before = collect(2, .5)
            result["raw_before"] = raw_before
            send("send 5"); state = collect(5005, .3)
            if args.restore:
                predicted = [expected(raw_before[4], 11.220408074, 1),
                             expected(raw_before[5], 302.910885201, -1)]
                result["expected_joint_deg"] = predicted
                if not int(state[14]) & 1:
                    send("leg restore"); state = collect(5005, .5)
                    assert state[16] != before[16], "No command ACK"
                    assert state[15] == 0, f"Rejected: {state[15]}"
                    assert int(state[14]) & 1, "Position not restored"
                    assert max(abs(state[i] - predicted[i]) for i in (0, 1)) < .05, "Wrong recovered branch"
                result["restored"] = state
                q = state[:2]
                for _ in range(2):
                    ack = state[16]
                    send("leg restore"); state = collect(5005, .3)
                    assert state[16] != ack and state[15] == 0, "Repeat restore rejected"
                    assert max(abs(state[i] - q[i]) for i in (0, 1)) < .03, "Repeat restore changed origin"
                result["repeat_preserved_joint_deg"] = list(state[:2])
                first_seq = state[20]; state = collect(5005, 1)
                assert state[20] != first_seq and int(state[14]) & 257 == 257, "Tracking stalled"
                result["final"] = state
                send("send 2"); result["raw_after"] = collect(2, .5)
                send("send 5"); collect(5005, .2)
            result["success"] = True
    except Exception as exc:
        result["error"] = str(exc)
    finally:
        result["frames"] = len(rows)
        result["all_outputs_off"] = all((r[11] == r[12] == 0 if r[23] == 5005 else r[9] == r[10] == 0) for r in rows)
        # Strict JSON: absent FK before recovery is represented as null.
        def clean(value):
            if isinstance(value, float) and not math.isfinite(value): return None
            if isinstance(value, (list, tuple)): return [clean(v) for v in value]
            if isinstance(value, dict): return {k: clean(v) for k, v in value.items()}
            return value
        args.output.write_text(json.dumps(clean(result), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(clean(result), ensure_ascii=False, indent=2))
    if not result["success"]: raise SystemExit(1)

if __name__ == "__main__": main()
