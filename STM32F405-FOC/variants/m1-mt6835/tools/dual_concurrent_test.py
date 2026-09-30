"""Bounded, current-monitored open-loop sweep of both motor outputs."""

import argparse
import math
import time

from dual_smoke import Frames


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    args = parser.parse_args()
    link = Frames(args.port)
    try:
        link.command("stop")
        link.command("clear")
        deadline = time.monotonic() + 2.0
        idle = None
        while time.monotonic() < deadline:
            frame = link.read()
            if frame is not None and int(frame[9]) == int(frame[21]) == 0 and \
                    frame[10] == frame[22] == 0 and \
                    8.0 <= frame[7] <= 12.4:
                idle = frame
                break
        if idle is None:
            raise RuntimeError("both motors must be idle with a valid bus")
        start_position = [idle[3], idle[15]]
        link.serial.reset_input_buffer()
        link.buffer.clear()
        link.command("all test")
        overlap = 0
        seen = [False, False]
        peak_current = [0.0, 0.0]
        last = idle
        deadline = time.monotonic() + 7.0
        while time.monotonic() < deadline:
            frame = link.read()
            if frame is None:
                if time.monotonic() - link.last_frame > 0.5:
                    raise RuntimeError("USB telemetry stopped")
                continue
            last = frame
            if not math.isfinite(frame[7]) or not 8.0 <= frame[7] <= 12.4:
                raise RuntimeError(f"bus out of range: {frame[7]}")
            states = [int(frame[9]), int(frame[21])]
            faults = [int(frame[10]), int(frame[22])]
            warnings = [int(frame[11]), int(frame[23])]
            if any(faults) or any(state == 5 for state in states):
                raise RuntimeError(f"motor fault: states={states} faults={faults}")
            if any(warning & ((1 << 1) | (1 << 10)) for warning in warnings):
                raise RuntimeError(f"sensor/current warning: {warnings}")
            for i in range(2):
                current = math.hypot(frame[5 + 12 * i], frame[6 + 12 * i])
                if not math.isfinite(current) or current > 2.0:
                    raise RuntimeError(f"M{i} current out of range: {current}")
                peak_current[i] = max(peak_current[i], current)
                seen[i] |= states[i] == 2
            if states == [2, 2]:
                overlap += 1
            if all(seen) and states == [0, 0]:
                break
        delta = [last[3] - start_position[0], last[15] - start_position[1]]
        print(f"simultaneous frames={overlap}, peak current="
              f"{peak_current[0]:.3f}/{peak_current[1]:.3f} A, "
              f"position delta={delta[0]:.1f}/{delta[1]:.1f} deg, "
              f"final states={int(last[9])}/{int(last[21])}", flush=True)
        if overlap < 500 or min(peak_current) < 0.05 or \
                int(last[9]) != 0 or int(last[21]) != 0:
            raise RuntimeError("both outputs did not complete the concurrent sweep")
    finally:
        link.close()


if __name__ == "__main__":
    main()
