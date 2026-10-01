"""Read-only ST-Link snapshot of the stopped gimbal using its current ELF."""
import argparse
import hashlib
import json
import math
import re
import struct
import subprocess
from pathlib import Path

CUBE = Path("C:/Users/kongmeng/.vscode/extensions/stmicroelectronics.stm32cube-ide-core-1.4.0-win32-x64/resources/binaries/win32/x86_64/cube.exe")
NM = Path("D:/STM32CubeIDE/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344/tools/bin/arm-none-eabi-nm.exe")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("elf", type=Path)
    parser.add_argument("out", type=Path)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    def run(argv):
        proc = subprocess.run([str(x) for x in argv], capture_output=True, timeout=15)
        log = proc.stdout.decode(errors="replace") + proc.stderr.decode(errors="replace")
        if proc.returncode:
            raise RuntimeError(log)
        return log

    symbols = {}
    for line in run([NM, "-S", args.elf]).splitlines():
        parts = line.split()
        if len(parts) == 4:
            symbols.setdefault(parts[3], []).append((int(parts[0], 16), int(parts[1], 16)))

    # Never interpret live RAM using addresses from a different firmware build.
    binary = args.elf.with_suffix(".bin")
    if not binary.exists():
        raise RuntimeError("matching .bin is required to verify the flashed build")
    expected = binary.read_bytes()

    def read(name, address, size):
        log = run([CUBE, "programmer", "-c", "port=SWD", "freq=1000", "mode=HOTPLUG", "-r32", hex(address), str(size)])
        (args.out / (name + ".log")).write_text(log, encoding="utf-8")
        words = []
        for match in re.finditer(r"0x[0-9a-fA-F]+\s*:\s*((?:[0-9a-fA-F]{8}[ \t]*)+)", log):
            words.extend(int(x, 16) for x in match[1].split())
        if len(words) != size // 4:
            raise RuntimeError(f"incomplete read for {name}")
        return struct.pack("<" + "I" * len(words), *words)

    flashed = b"".join(read(f"flash_readback_{offset:05x}", 0x08000000 + offset,
                           min(32768, len(expected) - offset))
                       for offset in range(0, len(expected), 32768))
    # Running-target bulk reads occasionally return a repeated placeholder.
    # Re-read every differing word individually; never waive a real mismatch.
    differences = [offset for offset in range(0, len(expected), 4)
                   if flashed[offset:offset+4] != expected[offset:offset+4]]
    if len(differences) > 64:
        raise RuntimeError("flashed firmware differs from ELF/bin; RAM snapshot refused")
    corrected = bytearray(flashed)
    for offset in differences:
        corrected[offset:offset+4] = read(f"flash_word_{offset:05x}", 0x08000000 + offset, 4)
    flashed = bytes(corrected)
    if flashed != expected:
        raise RuntimeError("flashed firmware differs from ELF/bin; RAM snapshot refused")

    result = {"elf_sha256": hashlib.sha256(args.elf.read_bytes()).hexdigest()}
    result["calibration_records"] = []
    for axis in range(2):
        record = read(f"calibration_record_{axis}", 0x080e0000 + axis * 24, 24)
        version, identity, zero, direction, saved_hash, magic = struct.unpack("<IIfiII", record)
        calculated_hash = 2166136261
        for byte in record[:16]:
            calculated_hash = ((calculated_hash ^ byte) * 16777619) & 0xffffffff
        result["calibration_records"].append(dict(version=version, identity=identity,
            zero=zero if math.isfinite(zero) else None, direction=direction, checksum_valid=saved_hash == calculated_hash,
            record_valid=version == 5 and magic == 0x464f4331 and saved_hash == calculated_hash
                and direction in (-1, 1) and 0 <= zero < 6.2831853072))
    for name in ("foc0", "foc1"):
        address, size = symbols[name][0]
        if size != 88:
            raise RuntimeError("unexpected foc_t layout")
        data = read(name, address, size)
        result[name] = dict(zip(("state", "fault"), struct.unpack_from("<II", data, 60)))
        result[name].update(calibrated=bool(data[68]), zero_ready=bool(data[69]),
            calibration_zero=struct.unpack_from("<f", data)[0], calibration_direction=struct.unpack_from("<i", data, 4)[0])
    for name in ("dual_fault", "dual_angle_sample_valid", "dual_init_crc_valid", "encoder_errors", "rejected"):
        address, size = symbols[name][0]
        data = read(name, address, size)
        result[name] = list(struct.unpack("<" + "I" * (size // 4), data))
    for name in ("dual_write_counter_min", "dual_update_counter_max", "dual_timing_site",
                 "dual_timing_counter", "dual_timing_direction", "dual_timing_reason",
                 "dual_timing_ready", "dual_timing_mode", "dual_timing_pending",
                 "dual_timing_foc_state", "dual_timing_sequence", "dual_encoder_cycles_max"):
        if name not in symbols:
            continue
        address, size = symbols[name][0]
        data = read(name, address, size)
        result[name] = list(struct.unpack("<" + "I" * (size // 4), data))
    for name in ("dual_init_first_crc_valid", "dual_init_attempts", "dual_init_spi_valid", "dual_init_status", "dual_init_safety"):
        address, size = symbols[name][0]
        data = read(name, address, size)
        code = "H" if name in ("dual_init_status", "dual_init_safety") else "I"
        result[name] = list(struct.unpack("<" + code * (size // (2 if code == "H" else 4)), data))
    # Sensor diagnostics use SPI lane order, unlike USB's M0/M1 motor order.
    # frame_pair(): lane 0 = SPI3/PA0/M1; lane 1 = SPI1/PA1/M0.
    result["encoder_startup_channels"] = []
    for lane, motor, spi, cs in ((0, "M1", "SPI3", "PA0"), (1, "M0", "SPI1", "PA1")):
        result["encoder_startup_channels"].append(dict(lane=lane, motor=motor, spi=spi, cs=cs,
            status=result["dual_init_status"][lane], safety=result["dual_init_safety"][lane],
            crc_and_sensor_id_valid=bool(result["dual_init_crc_valid"][0] & (1 << lane))))
    for name in ("travel", "origin", "forward", "ticks"):
        result[name] = []
        for axis, (address, size) in enumerate(sorted(symbols[name])):
            data = read(f"{name}_{axis}", address, size)
            result[name].append(struct.unpack("<I" if name == "ticks" else "<f", data)[0])
    if "field_diagnostics" in symbols:
        result["field_diagnostics"] = []
        for axis, (address, size) in enumerate(sorted(symbols["field_diagnostics"])):
            result["field_diagnostics"].append(list(struct.unpack("<8f", read(f"field_{axis}", address, size))))
    result["pwm_ccer"] = [struct.unpack("<I", read("tim1_ccer", 0x40010020, 4))[0],
                          struct.unpack("<I", read("tim8_ccer", 0x40010420, 4))[0]]
    result["outputs_off"] = result["pwm_ccer"] == [0x1000, 0]
    (args.out / "snapshot.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
