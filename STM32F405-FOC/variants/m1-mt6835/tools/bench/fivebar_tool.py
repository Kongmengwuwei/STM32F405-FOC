"""Five-bar FK/IK and workspace using the firmware C solver; optional serial export.
Offline commands never open a serial port. Angles in deg, positions in mm.
"""

import argparse
import ctypes as c
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
PROJECT = ROOT.parents[1]
F2 = c.c_float * 2
M2 = F2 * 2


class Geometry(c.Structure):
    _fields_ = [
        ("pivot", M2),
        ("active", F2),
        ("passive", F2),
        ("zero_ccw", F2),
        ("q_min", F2),
        ("q_max", F2),
        ("q_sign", c.c_int * 2),
        ("elbow_sign", c.c_int * 2),
        ("assembly_sign", c.c_int),
        ("min_sine", c.c_float),
    ]


class Pose(c.Structure):
    _fields_ = [
        ("xy", F2),
        ("q", F2),
        ("elbow", M2),
        ("jacobian", M2),
        ("inverse", M2),
        ("serial_sine", F2),
        ("parallel_sine", c.c_float),
    ]


STATUS = [
    "ok",
    "geometry",
    "nonfinite",
    "unreachable",
    "joint_limit",
    "serial_singularity",
    "parallel_singularity",
    "assembly_branch",
    "path",
    "rate",
]


def geometry_dict(g):
    return {
        name: (
            [[float(v) for v in row] for row in value]
            if name == "pivot"
            else list(value) if isinstance(value, c.Array) else value
        )
        for name, _ in g._fields_
        for value in [getattr(g, name)]
    }


def geometry_from_dict(data):
    g = Geometry()
    for name, typ in g._fields_:
        if name == "pivot":
            setattr(g, name, M2(*(F2(*row) for row in data[name])))
        elif issubclass(typ, c.Array):
            setattr(g, name, typ(*data[name]))
        else:
            setattr(g, name, data[name])
    return g


class Solver:
    def __init__(self, cc="gcc", config=None):
        compiler = shutil.which(cc)
        if not compiler:
            raise ValueError("Host GCC missing; pass --cc C:/mingw64/bin/gcc.exe")
        sources = [
            ROOT / "dual" / f
            for f in [
                "fivebar.c",
                "fivebar.h",
                "fivebar_config.h",
                "fivebar_calibration.h",
            ]
        ]
        digest = hashlib.sha256(b"".join(p.read_bytes() for p in sources)).hexdigest()[
            :12
        ]
        output = PROJECT / "build/host-tests"
        output.mkdir(parents=True, exist_ok=True)
        wrapper = output / "fivebar_host.c"
        wrapper.write_text(
            '#include "fivebar.h"\n#include "fivebar_config.h"\n'
            "void fb_default_geometry(fb_geometry_t *g) { *g=(fb_geometry_t)FIVEBAR_GEOMETRY_INITIALIZER; }\n"
            "int fb_default_calibrated(void) { return FIVEBAR_ZERO_CALIBRATED; }\n",
            encoding="utf-8",
        )
        library = output / (
            "fivebar_" + digest + (".dll" if sys.platform == "win32" else ".so")
        )
        if not library.exists():
            subprocess.run(
                [
                    compiler,
                    "-std=c11",
                    "-O2",
                    "-shared",
                    "-fPIC",
                    "-I",
                    str(ROOT / "dual"),
                    str(sources[0]),
                    str(wrapper),
                    "-lm",
                    "-o",
                    str(library),
                ],
                check=True,
                timeout=60,
            )
        self.lib = c.CDLL(str(library))
        self.lib.fb_default_geometry.argtypes = [c.POINTER(Geometry)]
        self.lib.fb_valid.argtypes = [c.POINTER(Geometry)]
        self.lib.fb_valid.restype = c.c_bool
        for name in ["fb_forward", "fb_inverse"]:
            getattr(self.lib, name).argtypes = [
                c.POINTER(Geometry),
                c.POINTER(c.c_float),
                c.POINTER(Pose),
            ]
        self.geometry = Geometry()
        self.lib.fb_default_geometry(c.byref(self.geometry))
        self.calibrated = bool(self.lib.fb_default_calibrated())
        if config:
            data = json.loads(Path(config).read_text(encoding="utf-8"))
            self.geometry = geometry_from_dict(data["geometry"])
            self.calibrated = bool(data["calibrated"])
        if not self.lib.fb_valid(c.byref(self.geometry)):
            raise ValueError("Invalid geometry")

    def solve(self, values, inverse=True):
        if len(values) != 2 or not all(math.isfinite(v) for v in values):
            raise ValueError("Finite pair required")
        pose = Pose()
        result = (self.lib.fb_inverse if inverse else self.lib.fb_forward)(
            c.byref(self.geometry), F2(*values), c.byref(pose)
        )
        if result:
            return STATUS[result], None
        return STATUS[result], pose


def workspace(solver, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    g = solver.geometry
    points = []
    for x in range(-160, 161, 2):
        for y in range(30, 241, 2):
            status, p = solver.solve([x, y])
            if p:
                points.append(
                    (x, y, min(p.serial_sine[0], p.serial_sine[1], p.parallel_sine))
                )
    fig, ax = plt.subplots(figsize=(7, 6))
    scatter = ax.scatter(
        [p[0] for p in points],
        [p[1] for p in points],
        c=[p[2] for p in points],
        s=8,
        cmap="viridis",
        vmin=g.min_sine,
        vmax=1,
    )
    status, p = solver.solve([60, 60], inverse=False)
    if p:
        for i, label in enumerate(["M0", "M1"]):
            ax.plot(
                [g.pivot[i][0], p.elbow[i][0], p.xy[0]],
                [g.pivot[i][1], p.elbow[i][1], p.xy[1]],
                "o-",
                lw=3,
                label=label,
            )
        ax.annotate("wheel / foot", p.xy, xytext=(10, 10), textcoords="offset points")
    ax.axvline(0, color="gray", lw=0.5)
    ax.axhline(0, color="gray", lw=0.5)
    fig.colorbar(scatter, ax=ax, fraction=0.046, pad=0.04, label="min singularity sine")
    ax.set(
        xlabel="x right (mm)",
        ylabel="y down (mm)",
        title=f"Five-bar workspace: M0/M1 joint {g.q_min[0]:g}..{g.q_max[0]:g} / {g.q_min[1]:g}..{g.q_max[1]:g} deg",
    )
    ax.set_aspect("equal")
    ax.invert_yaxis()
    ax.legend(loc="upper left")
    ax.grid(alpha=0.2)
    ax.text(
        0,
        -0.16,
        f"Pivot spacing {math.dist(list(g.pivot[0]), list(g.pivot[1])):g} mm; "
        f"active {g.active[0]:g}/{g.active[1]:g}, passive {g.passive[0]:g}/{g.passive[1]:g} mm\n"
        f"Rod zeros {g.zero_ccw[0]:.2f}/{g.zero_ccw[1]:.2f} deg: "
        + (
            "calibrated; motor-shaft estimate"
            if solver.calibrated
            else "physical calibration required"
        ),
        transform=ax.transAxes,
        fontsize=9,
        bbox=dict(facecolor="white", alpha=0.8),
    )
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(fig)
    return {
        "points": len(points),
        "plot": str(Path(path).resolve()),
        "calibrated": solver.calibrated,
    }


class Frames:
    def __init__(self):
        self.pending = bytearray()

    def feed(self, data):
        self.pending.extend(data)
        result = []
        while len(self.pending) >= 100:
            if self.pending[96:100] != b"\x00\x00\x80\x7f":
                del self.pending[0]
                continue
            row = struct.unpack("<24f", self.pending[:96])
            del self.pending[:100]
            if row[23] in (0, 1, 2, 3, 4):
                result.append(row)
        return result


def read_group(link, parser, group, timeout=3):
    link.reset_input_buffer()
    link.write(f"send {group}\r\n".encode("ascii"))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rows = [r for r in parser.feed(link.read(4096)) if r[23] == group]
        if rows:
            return rows[-1]
    raise ValueError(
        f"No group {group} frames; check the firmware and serial connection"
    )


def decode_geometry(row):
    if row[23] != 4 or not all(math.isfinite(v) for v in row):
        raise ValueError("Corrupt geometry frame")
    return {
        "geometry": {
            "pivot": [list(row[:2]), list(row[2:4])],
            "active": [row[4], row[6]],
            "passive": [row[5], row[7]],
            "zero_ccw": list(row[8:10]),
            "q_sign": [int(v) for v in row[10:12]],
            "elbow_sign": [int(v) for v in row[12:14]],
            "assembly_sign": int(row[14]),
            "q_min": [row[15], row[17]],
            "q_max": [row[16], row[18]],
            "min_sine": row[19],
        },
        "calibrated": row[20] == 1,
        "homed_at_export": row[22] == 1,
        "note": "Body frame x right/y down, mm. M0 right, M1 left. Motor-shaft estimate includes gearbox uncertainty.",
    }


def export_calibration(port, json_path, header_path):
    import serial

    with serial.Serial(port, 1000000, timeout=0.05) as link:
        link.dtr = True
        try:
            row = read_group(link, Frames(), 4)
        finally:
            link.write(b"send 0\r\n")
    data = decode_geometry(row)
    if row[21] != 1 or not data["calibrated"]:
        raise ValueError("Known rod-pose calibration has not been completed")
    if header_path:
        # Header intentionally persists zeros only; reject a different linkage.
        expected = {
            "pivot": [[35, 0], [-35, 0]],
            "active": [80, 80],
            "passive": [145, 145],
            "q_sign": [-1, 1],
            "elbow_sign": [1, -1],
            "assembly_sign": -1,
        }
        if any(data["geometry"][k] != v for k, v in expected.items()):
            raise ValueError("Unexpected linkage geometry")
        z = data["geometry"]["zero_ccw"]
        Path(header_path).write_text(
            "#ifndef FIVEBAR_CALIBRATION_H\n#define FIVEBAR_CALIBRATION_H\n"
            "// Exported from an operator-confirmed known rod pose.\n"
            f"#ifndef FIVEBAR_ZERO_M0_CCW\n#define FIVEBAR_ZERO_M0_CCW {z[0]:.9f}f\n#endif\n"
            f"#ifndef FIVEBAR_ZERO_M1_CCW\n#define FIVEBAR_ZERO_M1_CCW {z[1]:.9f}f\n#endif\n"
            "#ifndef FIVEBAR_ZERO_CALIBRATED\n#define FIVEBAR_ZERO_CALIBRATED 1\n#endif\n#endif\n",
            encoding="utf-8",
        )
    Path(json_path).parent.mkdir(parents=True, exist_ok=True)
    Path(json_path).write_text(
        json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return data


def move(port, solver, target, seconds, output):
    import serial

    if not 1 <= seconds <= 30:
        raise ValueError("Duration must be 1..30 seconds")
    rows = []
    parser = Frames()
    accepted = False
    with serial.Serial(port, 1000000, timeout=0.02) as link:
        link.dtr = True
        # No drive until live calibration, homing and the actual geometry match.
        geometry_row = read_group(link, parser, 4)
        data = decode_geometry(geometry_row)
        if not data["calibrated"] or geometry_row[21] != 1 or geometry_row[22] != 1:
            raise ValueError("Known-stop home and rod zero calibration required")
        solver.geometry = geometry_from_dict(data["geometry"])
        status, p = solver.solve(target)
        if not p:
            raise ValueError(f"Target rejected: {status}")
        row = read_group(link, parser, 3)
        if int(row[11]) & 3 != 3 or row[19] != 0 or row[20] != 0 or row[22] != 1:
            raise ValueError("Need a valid, calibrated, fault-free current foot pose")
        started = time.monotonic()
        last_tx = 0
        last_rx = started
        last_row = row
        command = f"foot pos {target[0]:.2f} {target[1]:.2f}\r\n".encode("ascii")
        try:
            while time.monotonic() - started < seconds:
                now = time.monotonic()
                if now - last_rx > 0.25:
                    raise ValueError("Foot telemetry stale; stopping")
                if now - last_tx >= 0.05:
                    link.write(command)
                    last_tx = now
                for row in parser.feed(link.read(4096)):
                    if row[23] != 3:
                        continue
                    last_rx = time.monotonic()
                    last_row = row
                    rows.append([last_rx - started, *row])
                    if row[19] or row[20] or row[22] != 1:
                        raise ValueError("FOC fault or homing lost")
                    if accepted and not int(row[11]) & 4:
                        raise ValueError(
                            "Foot controller stopped; no automatic restart"
                        )
                    if (
                        int(row[11]) & 4
                        and abs(row[0] - target[0]) < 0.03
                        and abs(row[1] - target[1]) < 0.03
                    ):
                        accepted = True
                if now - started > 0.3 and not accepted:
                    raise ValueError(
                        f"Firmware rejected foot target, status {last_row[10]:.0f}"
                    )
        finally:
            # Support the leg when output is released, just as for legacy stop.
            link.write(b"foot stop\r\nsend 0\r\n")
    result = {
        "target_mm": target,
        "command_accepted": accepted,
        "samples": len(rows),
        "final_estimate_mm": list(last_row[4:6]),
        "geometry": data,
        "hardware_output": "stop_requested",
    }
    if output:
        Path(output).parent.mkdir(parents=True, exist_ok=True)
        Path(output).write_text(
            json.dumps({**result, "rows": rows}, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cc", default="gcc")
    ap.add_argument("--config")
    commands = ap.add_subparsers(dest="command", required=True)
    for name in ["ik", "fk"]:
        parser = commands.add_parser(name)
        parser.add_argument("values", nargs=2, type=float)
    parser = commands.add_parser("workspace")
    parser.add_argument("--out", required=True)
    parser = commands.add_parser("export")
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--json", required=True)
    parser.add_argument("--header")
    parser = commands.add_parser(
        "move", help="Hardware motion; support the mechanism before use"
    )
    parser.add_argument("values", nargs=2, type=float)
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--seconds", type=float, default=5)
    parser.add_argument("--out")
    args = ap.parse_args()
    if args.command == "export":
        result = export_calibration(args.port, args.json, args.header)
    else:
        solver = Solver(args.cc, args.config)
        if args.command == "move":
            result = move(args.port, solver, args.values, args.seconds, args.out)
        elif args.command == "workspace":
            result = workspace(solver, args.out)
        else:
            status, p = solver.solve(args.values, args.command == "ik")
            result = {"status": status, "physically_calibrated": solver.calibrated}
            if p:
                result.update(
                    xy_mm=list(p.xy),
                    joint_deg=list(p.q),
                    rod_ccw_deg=[
                        solver.geometry.zero_ccw[i] + solver.geometry.q_sign[i] * p.q[i]
                        for i in range(2)
                    ],
                    jacobian_mm_per_deg=[list(row) for row in p.jacobian],
                    singularity_margin=min(
                        p.serial_sine[0], p.serial_sine[1], p.parallel_sine
                    ),
                )
            else:
                print(json.dumps(result))
                return 1
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ValueError, OSError, subprocess.SubprocessError) as error:
        print(str(error), file=sys.stderr)
        sys.exit(2)
