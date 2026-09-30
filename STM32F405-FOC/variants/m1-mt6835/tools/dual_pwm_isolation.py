"""Short selected-motor PWM-zero test for isolating phase/bridge faults.

Use only with free shafts and a current-limited 12 V supply. The test stops
at the first invalid sensor/current/bus sample or after the zero-vector stage.
"""

import argparse
import math
import time

from dual_smoke import Frames


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("motor", choices=("m0", "m1"), nargs="?", default="m0")
    parser.add_argument("--zero-frames", type=int, default=51)
    args = parser.parse_args()
    selected = 0 if args.motor == "m0" else 1
    other = 1 - selected
    if not 1 <= args.zero_frames <= 51:
        raise SystemExit("--zero-frames must be 1..51")
    link = Frames(args.port)
    try:
        link.command("stop")
        link.command("clear")
        ready = False
        until = time.monotonic() + 2.0
        while time.monotonic() < until:
            frame = link.read()
            if frame is not None and int(frame[9]) == int(frame[21]) == 0 and \
                    frame[10] == frame[22] == 0 and \
                    int(frame[11]) == int(frame[23]) == 0 and \
                    8.0 <= frame[7] <= 13.0:
                ready = True
                break
        if not ready:
            raise RuntimeError("both motors must be idle with a valid 12 V bus")
        link.serial.reset_input_buffer()
        link.buffer.clear()
        link.command(f"{args.motor} cal")
        until = time.monotonic() + 0.40
        zero_frames = 0
        peak_iq = 0.0
        bus_min = math.inf
        bus_max = -math.inf
        reason = None
        last_state = -1
        frame_count = 0
        while time.monotonic() < until:
            frame = link.read()
            if frame is None:
                continue
            frame_count += 1
            bus_min = min(bus_min, frame[7])
            bus_max = max(bus_max, frame[7])
            peak_iq = max(peak_iq, abs(frame[5 + 12 * selected]))
            state = int(frame[9 + 12 * selected])
            last_state = state
            if state == 7:
                zero_frames += 1
                if zero_frames >= args.zero_frames:
                    break
            if not 8.0 <= frame[7] <= 13.0 or \
                    abs(frame[5 + 12 * selected]) > 2.0 or \
                    frame[10] or frame[22] or int(frame[9 + 12 * other]) != 0 or \
                    (int(frame[11]) | int(frame[23])) & (2 | 32):
                reason = (f"state={state}, warnings={int(frame[11])}/"
                          f"{int(frame[23])}, faults={int(frame[10])}/"
                          f"{int(frame[22])}")
                break
            if state == 2 and zero_frames:
                break
        print(f"frames={frame_count}, PWM-zero frames={zero_frames}, "
              f"last state={last_state}, peak Iq={peak_iq:.3f} A, "
              f"bus={bus_min:.2f}..{bus_max:.2f} V, reason={reason}")
        if reason:
            raise RuntimeError(f"isolation test stopped on {reason}")
        if not zero_frames:
            raise RuntimeError(f"{args.motor} PWM-zero stage was not observed")
    finally:
        link.close()


if __name__ == "__main__":
    main()
