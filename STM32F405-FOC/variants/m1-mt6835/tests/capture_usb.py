"""Validate decimated USB logging from a M0 10 kHz or M1 20 kHz loop.

Frame: 12 little-endian float32 + the JustFloat terminator = 52 bytes.
Channel 0 packs t_u24 with the status word, channel 1 packs the sample counter
with the group number; see tools/bench/README.md for the group channel maps.

pip install pyserial
"""
import argparse
import struct
import time

import serial

FRAME_BYTES = 52
CHANNELS = 12
TERMINATOR = b"\x00\x00\x80\x7f"
T_24_MASK = 0xFFFFFF
GROUP_NAMES = {0: "raw", 1: "current", 2: "voltage", 3: "control", 4: "diagnostics", 5: "current_step"}


def capture(port, seconds, group, sample_hz=10000, usb_divider=2):
    header = struct.Struct("<II")
    pending = bytearray()
    previous = None
    frames = 0
    started = None
    synced = False
    with serial.Serial() as link:
        link.port = port
        link.baudrate = 1000000  # CDC line coding only, not the USB bit rate.
        link.timeout = 0.1
        link.dtr = False
        link.open()
        # Finish any previous session while the MCU producer is stopped.
        drain_until = time.monotonic() + 0.25
        while time.monotonic() < drain_until:
            link.read(max(1, link.in_waiting))
        link.reset_input_buffer()
        link.dtr = True
        link.write(f"send {group}\r\n".encode("ascii"))
        last_data = time.monotonic()
        selection_deadline = last_data + 2.0
        while started is None or time.monotonic() - started < seconds:
            data = link.read(max(1, min(link.in_waiting, 65536)))
            now = time.monotonic()
            if started is None and now > selection_deadline:
                raise RuntimeError("Selected telemetry group did not arrive within 2 seconds")
            if not data:
                if now - last_data > 2:
                    raise RuntimeError("USB stream stalled: inspect g_usb_stats; "
                                       "reset/replug after a latched fault")
                continue
            last_data = now
            pending.extend(data)
            if not synced:
                # Opening an existing CDC session can expose an initial partial frame.
                pos = pending.find(TERMINATOR)
                if pos < 0:
                    if len(pending) > 4096:
                        raise RuntimeError("No JustFloat boundary found")
                    continue
                del pending[:pos + len(TERMINATOR)]
                synced = True
            used = 0
            while len(pending) - used >= FRAME_BYTES:
                if bytes(pending[used + CHANNELS * 4:used + FRAME_BYTES]) != TERMINATOR:
                    raise RuntimeError(f"Frame boundary lost after {frames} frames")
                time_us, index = header.unpack_from(pending, used)
                if index >> 24 != group:
                    if previous is None:
                        used += FRAME_BYTES
                        continue  # DTR can start the old group before send is processed.
                    raise RuntimeError(f"Group changed mid-capture: {index >> 24} != {group}")
                time_us &= T_24_MASK
                seq = index & T_24_MASK
                if previous is not None:
                    previous_time, previous_seq = previous
                    period_us = 1000000 * usb_divider // sample_hz
                    if not period_us - 5 <= ((time_us - previous_time) & T_24_MASK) <= period_us + 5:
                        raise RuntimeError(f"Timing excursion: {previous_time} -> {time_us} us")
                    if ((seq - previous_seq) & T_24_MASK) != usb_divider:
                        raise RuntimeError(f"Sample counter jumped: {previous_seq} -> {seq}")
                previous = (time_us, seq)
                used += FRAME_BYTES
                frames += 1
            del pending[:used]
            if frames and started is None:
                started = now
        elapsed = time.monotonic() - started
        rate = frames / elapsed
        frame_hz = sample_hz / usb_divider
        if not frame_hz * 0.99 <= rate <= frame_hz * 1.01:
            raise RuntimeError(f"Continuous frames but unexpected rate: {rate:.1f} samples/s")
        print(f"PASS: group {group} ({GROUP_NAMES[group]}), {frames} consecutive frames, "
              f"{elapsed:.2f} s, {rate:.1f} samples/s, {frames * FRAME_BYTES / elapsed:.0f} bytes/s")
        link.dtr = False
        drain_until = time.monotonic() + 0.25
        while time.monotonic() < drain_until:
            link.read(max(1, link.in_waiting))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Native USB CDC port, not the CH340 UART port")
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--group", type=int, default=0, choices=sorted(GROUP_NAMES),
                        help="logging group to validate with `send X`")
    parser.add_argument("--sample-hz", type=int, default=10000, choices=(10000, 20000),
                        help="control sample rate: 10000 for M0; 20000 for M1")
    parser.add_argument("--usb-divider", type=int, default=2,
                        help="control cycles per USB frame, default 2")
    args = parser.parse_args()
    if args.seconds < 10:
        parser.error("Use at least 10 seconds for rate validation")
    if not 1 <= args.usb_divider <= 1000 or args.sample_hz % args.usb_divider:
        parser.error("USB divider must be in 1..1000 and divide the sample rate")
    capture(args.port, args.seconds, args.group, args.sample_hz, args.usb_divider)
