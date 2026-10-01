"""Read an opt-in, motor-disabled encoder clock diagnostic via ST-Link."""
import argparse
import hashlib
import json
import re
import struct
import subprocess
import time
from pathlib import Path

from gimbal_snapshot import CUBE, NM


def crc8(command, data):
    # Same byte order and polynomial as App/Hardware/tle5012b_protocol.c.
    crc = 0xff
    for byte in (command >> 8, command & 255, data >> 8, data & 255):
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1d if crc & 128 else 0)) & 255
    return crc ^ 255


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("elf", type=Path)
    parser.add_argument("out", type=Path)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    symbols = {}
    for line in subprocess.check_output([str(NM), "-S", str(args.elf)], text=True).splitlines():
        parts = line.split()
        if len(parts) == 4:
            symbols[parts[3]] = (int(parts[0], 16), int(parts[1], 16))

    def read_bytes(name, address, size):
        proc = subprocess.run([str(CUBE), "programmer", "-c", "port=SWD", "freq=1000",
                               "mode=HOTPLUG", "-r32", hex(address), str(size)],
                              capture_output=True, timeout=15)
        log = proc.stdout.decode(errors="replace") + proc.stderr.decode(errors="replace")
        (args.out / (name + ".log")).write_text(log, encoding="utf-8")
        if proc.returncode:
            raise RuntimeError(log)
        words = []
        for match in re.finditer(r"0x[0-9a-fA-F]+\s*:\s*((?:[0-9a-fA-F]{8}[ \t]*)+)", log):
            words.extend(int(x, 16) for x in match[1].split())
        if len(words) * 4 != size:
            raise RuntimeError(f"incomplete read: {name}")
        return struct.pack("<" + "I" * len(words), *words)

    def read(name, code):
        address, size = symbols[name]
        return struct.unpack("<" + code * (size // struct.calcsize(code)),
                             read_bytes(name, address, size))

    # This build deliberately excludes normal startup symbols. Verify its code
    # directly rather than interpreting it with the normal snapshot layout.
    expected = args.elf.with_suffix(".bin").read_bytes()
    flashed = b"".join(read_bytes(f"flash_readback_{offset:05x}", 0x08000000 + offset,
        min(32768, len(expected) - offset)) for offset in range(0, len(expected), 32768))
    differences = [n for n in range(0, len(expected), 4) if flashed[n:n+4] != expected[n:n+4]]
    if len(differences) > 64:
        raise RuntimeError("flashed firmware differs from diagnostic ELF/bin")
    corrected = bytearray(flashed)
    for offset in differences:
        corrected[offset:offset+4] = read_bytes(f"flash_word_{offset:05x}", 0x08000000 + offset, 4)
    if bytes(corrected) != expected:
        raise RuntimeError("flashed firmware differs from diagnostic ELF/bin")
    ccer = [struct.unpack("<I", read_bytes("tim1_ccer", 0x40010020, 4))[0],
            struct.unpack("<I", read_bytes("tim8_ccer", 0x40010420, 4))[0]]
    if ccer != [4096, 0]:
        raise RuntimeError("motor outputs must be off")

    deadline = time.monotonic() + 20
    while read("encoder_diag_complete", "I") != (0x454e4344,):
        if time.monotonic() >= deadline:
            raise RuntimeError("diagnostic did not finish")
        time.sleep(1)
    hz = read("encoder_diag_hz", "I")
    masks = read("encoder_diag_spi_valid", "I")
    raw = read("encoder_diag_raw", "H")
    result = {"outputs_off": True, "pwm_ccer": ccer,
              "elf_sha256": hashlib.sha256(args.elf.read_bytes()).hexdigest(),
              "calibration_records_hex": read_bytes("calibration_records", 0x080e0000, 48).hex(),
              "rates": []}
    if "encoder_diag_long_samples" in symbols:
        samples = read("encoder_diag_long_samples", "I")[0]
        crc_good = read("encoder_diag_long_crc_good", "I")
        angle_good = read("encoder_diag_long_angle_good", "I")
        angle_min = read("encoder_diag_long_min", "I")
        angle_max = read("encoder_diag_long_max", "I")
        result["long_read"] = dict(m0_clock_hz=2625000, m1_clock_hz=5250000,
            samples=samples, channels=[dict(motor=motor, crc_good=crc_good[lane],
                angle_good=angle_good[lane],
                angle_range_degrees=[angle_min[lane]*360/32768, angle_max[lane]*360/32768]
                                    if angle_good[lane] else None)
                for lane, motor in enumerate(("M1", "M0"))])
    for rate, clock in enumerate(hz):
        row = {"m0_clock_hz": clock, "m0_method": "GPIO" if rate == 6 else "SPI1",
               "m0_clock_is_upper_bound": rate == 6,
               "m1_clock_hz": 5250000, "channels": []}
        for lane, motor in enumerate(("M1", "M0")):
            frames = []
            for sample in range(32):
                offset = ((rate * 2 + lane) * 32 + sample) * 2
                data, safety = raw[offset:offset + 2]
                command = 0x8021 if sample & 1 else 0x8001
                transfer_ok = bool(masks[rate * 2 + lane] & (1 << sample))
                crc_ok = transfer_ok and crc8(command, data) == (safety & 255)
                response = next((n for n in range(4)
                                 if safety & 0xf00 == (15 ^ (1 << n)) << 8), None)
                frames.append(dict(command=command, data=data, safety=safety,
                    transfer_ok=transfer_ok, crc_ok=crc_ok, response=response,
                    normal_angle_valid=bool(crc_ok and safety & 0xf000 == 0xf000
                                            and response == (3 if lane else 0)),
                    angle_degrees=(data & 0x7fff) * 360 / 32768 if sample & 1 and crc_ok else None))
            channel = dict(motor=motor, lane=lane, frames=frames,
                           crc_good=sum(f["crc_ok"] for f in frames),
                           all_ffff=sum(f["data"] == 65535 and f["safety"] == 65535 for f in frames))
            row["channels"].append(channel)
        result["rates"].append(row)
    (args.out / "encoder-link.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps([dict(m0_hz=r["m0_clock_hz"], m0_method=r["m0_method"], channels=[
        {k: c[k] for k in ("motor", "crc_good", "all_ffff")} for c in r["channels"]])
        for r in result["rates"]]), flush=True)
    if "long_read" in result:
        print(json.dumps(result["long_read"]), flush=True)


if __name__ == "__main__":
    main()
