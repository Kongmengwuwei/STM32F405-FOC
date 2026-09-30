"""Bounded USB smoke test for the dual-motor firmware.

Run only with a current-limited supply and free motor shafts. The selected
motor calibrates; the other motor must remain idle throughout the test.
"""

import argparse
import math
import struct
import time

import serial


MARKER = b"\x00\x00\x80\x7f"


class Frames:
    def __init__(self, port):
        self.serial = serial.Serial(port, 1_000_000, timeout=0.05, write_timeout=0.2)
        self.serial.dtr = True
        self.buffer = bytearray()
        self.last_frame = time.monotonic()

    def command(self, value):
        self.serial.write(value.encode("ascii") + b"\r\n")
        self.serial.flush()

    def read(self):
        marker = self.buffer.find(MARKER, 96)
        if marker >= 0:
            frame = struct.unpack("<25f", self.buffer[marker - 96:marker + 4])
            del self.buffer[:marker + 4]
            self.last_frame = time.monotonic()
            return frame
        if len(self.buffer) > 500:
            del self.buffer[:-100]
        self.buffer.extend(self.serial.read(min(self.serial.in_waiting or 1, 4096)))
        return None

    def close(self):
        try:
            self.command("stop")
        finally:
            self.serial.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("motor", choices=("m0", "m1"))
    args = parser.parse_args()
    selected = 0 if args.motor == "m0" else 1
    other = 1 - selected
    link = Frames(args.port)
    try:
        link.command("stop")
        link.command("clear")
        start = time.monotonic()
        idle = None
        while time.monotonic() - start < 2.0:
            frame = link.read()
            if frame is None:
                continue
            if (int(frame[9]) == int(frame[21]) == 0 and
                    frame[10] == frame[22] == 0 and
                    int(frame[11]) == int(frame[23]) == 0):
                idle = frame
                break
        if idle is None or not 8.0 <= idle[7] <= 13.0:
            raise RuntimeError("idle state, fault, or bus voltage is not safe")
        print(f"idle bus={idle[7]:.2f}V positions={idle[3]:.1f}/{idle[15]:.1f}deg", flush=True)
        link.command(f"{args.motor} cal")
        deadline = time.monotonic() + 16.0
        seen = set()
        count = 0
        bus_min = math.inf
        bus_max = -math.inf
        current_max = 0.0
        while time.monotonic() < deadline:
            frame = link.read()
            if frame is None:
                if time.monotonic() - link.last_frame > 1.0:
                    raise RuntimeError("USB telemetry stopped")
                continue
            count += 1
            bus = frame[7]
            bus_min = min(bus_min, bus)
            bus_max = max(bus_max, bus)
            state = int(frame[9 + 12 * selected])
            fault = int(frame[10 + 12 * selected])
            warning = int(frame[11 + 12 * selected])
            other_state = int(frame[9 + 12 * other])
            other_fault = int(frame[10 + 12 * other])
            other_warning = int(frame[11 + 12 * other])
            iq = frame[5 + 12 * selected]
            id_current = frame[6 + 12 * selected]
            current = math.hypot(iq, id_current)
            current_max = max(current_max, current)
            if not math.isfinite(bus) or not 8.0 <= bus <= 13.0:
                raise RuntimeError(f"bus out of range: {bus}")
            if not math.isfinite(current) or current > 2.0:
                raise RuntimeError(f"motor current estimate out of range: {current}")
            if state == 5 or fault or other_state != 0 or other_fault:
                raise RuntimeError(f"state/fault selected={state}/{fault} other={other_state}/{other_fault}")
            if (warning | other_warning) & ((1 << 1) | (1 << 5)):
                raise RuntimeError(f"sensor or alignment warning: selected={warning} other={other_warning}")
            seen.add(state)
            if 2 in seen and state == 0:
                print(f"cal complete frames={count} states={sorted(seen)} "
                      f"bus={bus_min:.2f}..{bus_max:.2f}V current_max={current_max:.3f}A", flush=True)
                return
            if count % 2000 == 0:
                print(f"progress state={state} bus={bus:.2f}V iq={iq:.3f}A", flush=True)
        raise RuntimeError(f"calibration timed out; states={sorted(seen)}")
    finally:
        link.close()


if __name__ == "__main__":
    main()
