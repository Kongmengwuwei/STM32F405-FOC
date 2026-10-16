"""Offline protocol/config regressions. Never touches COM ports."""

import importlib.util
import json
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "fivebar_tool", ROOT / "tools/bench/fivebar_tool.py"
)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)
row = [0.0] * 24
row[0] = 35
row[2] = -35
row[4:8] = [80, 145, 80, 145]
row[8:10] = [20, 160]
row[10:15] = [-1, 1, 1, -1, -1]
row[15:19] = [2, 108, 2, 108]
row[19:24] = [0.12, 1, 1, 1, 4]
wire = struct.pack("<24f", *row) + b"\x00\x00\x80\x7f"
frames = tool.Frames()
assert not frames.feed(b"noise" + wire[:35])
assert frames.feed(wire[35:] + wire) == [tuple(struct.unpack("<24f", wire[:96]))] * 2
live = tool.decode_geometry(tuple(struct.unpack("<24f", wire[:96])))
g = tool.geometry_from_dict(live["geometry"])
assert tool.geometry_dict(g) == live["geometry"]
assert live["calibrated"] and live["homed_at_export"]
for wrong in [
    wire[:92] + struct.pack("<f", 3) + wire[96:],
    wire[:92] + struct.pack("<f", 4) + wire[96:],
]:
    parsed = tool.Frames().feed(wrong)
    assert len(parsed) == 1
try:
    corrupt = row.copy()
    corrupt[8] = float("nan")
    tool.decode_geometry(corrupt)
    raise AssertionError("nonfinite accepted")
except ValueError:
    pass


class FakeLink:
    def __init__(self):
        self.commands = []

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.commands.append(data)

    def read(self, n):
        return wire


link = FakeLink()
assert tool.read_group(link, tool.Frames(), 4)[23] == 4
assert link.commands == [b"send 4\r\n"]
print(
    "PASS: fragmented dual frames, group selection, geometry ABI and nonfinite rejection (no hardware)"
)

# Exercise serial command ownership with a simulated stream, never a real port.
from types import SimpleNamespace
from unittest.mock import patch


class SimulatedSerial:
    scenario = "normal"
    instances = []

    def __init__(self, *args, **kwargs):
        self.commands = []
        self.group = 4
        self.driven = False
        self.drive_reads = 0
        self.instances.append(self)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.commands.append(data)
        if data.startswith(b"send "):
            self.group = int(data[5:6])
        if data.startswith(b"foot pos "):
            self.driven = True
        if data.startswith(b"foot stop"):
            self.driven = False

    def read(self, n):
        if self.group == 4:
            r = row.copy()
            if self.scenario == "uncalibrated":
                r[20] = 0
        else:
            r = [0.0] * 24
            r[0:6] = [0, 160, 0, 160, 0, 160]
            r[11] = 7 if self.driven else 3
            r[17:19] = [4, 4]
            r[22] = 1
            r[23] = 3
            if self.driven:
                self.drive_reads += 1
                if self.drive_reads > 1 and self.scenario == "fault":
                    r[19] = 11
                if self.drive_reads > 1 and self.scenario == "stopped":
                    r[11] = 3
        return struct.pack("<24f", *r) + b"\x00\x00\x80\x7f"


class SimulatedClock:
    def __init__(self):
        self.t = 0

    def __call__(self):
        self.t += 0.01
        return self.t


class FakeSolver:
    def solve(self, values):
        return "ok", object()


for scenario in ["normal", "uncalibrated", "fault", "stopped"]:
    SimulatedSerial.scenario = scenario
    with patch.dict(
        sys.modules, {"serial": SimpleNamespace(Serial=SimulatedSerial)}
    ), patch.object(tool.time, "monotonic", SimulatedClock()):
        try:
            result = tool.move("FAKE", FakeSolver(), [0, 160], 1, None)
            assert scenario == "normal" and result["command_accepted"]
        except ValueError:
            assert scenario != "normal"
    commands = SimulatedSerial.instances[-1].commands
    if scenario == "uncalibrated":
        assert not any(cmd.startswith(b"foot pos") for cmd in commands)
    else:
        assert commands[-1] == b"foot stop\r\nsend 0\r\n"
print(
    "PASS: simulated hardware tool calibration gate, target refresh, fault/stop cancellation and final stop"
)
