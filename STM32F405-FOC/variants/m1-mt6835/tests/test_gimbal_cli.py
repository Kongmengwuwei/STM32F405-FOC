"""Host checks for the reusable tool's command/monitor/shutdown contract."""
import contextlib
import io
import json
import sys
import unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import gimbal_cli as cli


class Link:
    def __init__(self):
        self.serial = self
        self.buffer = bytearray()
        self.last_frame = 0
        self.commands = []
        self.targets = [0, 0]
        self.states = [0, 0]
        self.faults = [0, 0]
        self.reject = False
        self.run_fault = False
        self.closed = False

    def reset_input_buffer(self):
        pass

    def close(self):
        self.closed = True

    def command(self, value):
        self.commands.append(value)
        if value == "stop":
            self.states = [5 if f else 0 for f in self.faults]
        elif value.startswith("gimbal pos") and not self.reject:
            self.targets = [float(v) for v in value.split()[2:]]
            self.states = [4, 4]
            if self.run_fault:
                self.faults[0] = 10
                self.states[0] = 5

    def read(self):
        values = []
        for target, state, fault in zip(self.targets, self.states, self.faults):
            values += [0, 0, target, target, .1, .1, 0, 11.5, 2 if state == 4 else 0, state, fault, 0]
        return tuple(values)


class ToolTests(unittest.TestCase):
    def run_cli(self, arguments, link):
        tick = [0.0]
        def clock():
            tick[0] += .01
            return tick[0]
        output = io.StringIO()
        with patch.object(cli, "select_port", return_value="COM8"), \
             patch.object(cli, "Frames", return_value=link), \
             patch.object(cli.time, "monotonic", clock), contextlib.redirect_stdout(output):
            code = cli.main(arguments)
        return code, json.loads(output.getvalue())

    def test_status_never_sends_or_stops(self):
        link = Link()
        link.states = [4, 4]
        code, result = self.run_cli(["status"], link)
        self.assertEqual(code, 0)
        self.assertEqual(link.commands, [])
        self.assertTrue(link.closed)
        self.assertEqual(result["telemetry"]["axes"][0]["role"], "pitch")
        self.assertEqual(result["telemetry"]["axes"][1]["role"], "yaw")

    def test_move_pair_then_verified_stop(self):
        link = Link()
        code, result = self.run_cli(["move", "--pitch", "3", "--yaw", "-4"], link)
        self.assertEqual(code, 0)
        self.assertEqual(link.commands, ["gimbal pos 3.00 -4.00", "stop"])
        self.assertTrue(result["settled"] and result["stop_confirmed"])
        self.assertFalse(result["holding_after_exit"])

    def test_explicit_keep_holding(self):
        link = Link()
        code, result = self.run_cli(["move", "--pitch", "1", "--yaw", "2", "--keep-holding"], link)
        self.assertEqual(code, 0)
        self.assertEqual(link.commands, ["gimbal pos 1.00 2.00"])
        self.assertTrue(result["holding_after_exit"] and link.closed)

    def test_keep_holding_still_stops_on_fault(self):
        link = Link()
        link.run_fault = True
        code, result = self.run_cli(["move", "--pitch", "1", "--yaw", "2", "--keep-holding"], link)
        self.assertEqual(code, 1)
        self.assertEqual(link.commands[-1], "stop")
        self.assertTrue(result["stop_confirmed"] and link.closed)

    def test_rejected_command_does_not_claim_arrival(self):
        link = Link()
        link.reject = True
        code, result = self.run_cli(["move", "--pitch", "3", "--yaw", "4"], link)
        self.assertEqual(code, 1)
        self.assertIn("not accepted", result["error"])
        self.assertTrue(result["stop_confirmed"])

    def test_preexisting_fault_not_cleared(self):
        link = Link()
        link.states[0], link.faults[0] = 5, 1
        code, result = self.run_cli(["move", "--pitch", "0", "--yaw", "0"], link)
        self.assertEqual(code, 1)
        self.assertEqual(link.commands, ["stop"])
        self.assertTrue(result["stop_confirmed"])

    def test_sustained_current_guard(self):
        controller = cli.Gimbal(Link())
        frame = list(Link().read())
        frame[5] = 9.0
        data = cli.unpack(frame)
        for _ in range(19):
            controller.check(data)
        with self.assertRaises(RuntimeError):
            controller.check(data)

    def test_input_boundaries_and_nonfinite(self):
        self.assertEqual(cli.angle(-85), -85)
        for value in (85.01, -90, float("nan"), float("inf"), 1.234):
            with self.assertRaises(ValueError):
                cli.angle(value)

    def test_discovery_follows_board_not_old_com_number(self):
        board = SimpleNamespace(vid=0x0483, pid=0x5740, device="COM9", serial_number=cli.BOARD_SERIAL)
        unrelated = SimpleNamespace(vid=None, pid=None, device="COM8", serial_number=None)
        with patch.object(cli.list_ports, "comports", return_value=[unrelated, board]):
            self.assertEqual(cli.select_port(), "COM9")
            with self.assertRaises(RuntimeError):
                cli.select_port("COM8")

    def test_duplicate_board_identity_requires_selection(self):
        boards = [SimpleNamespace(vid=0x0483, pid=0x5740, device=p, serial_number=cli.BOARD_SERIAL)
                  for p in ("COM8", "COM9")]
        with patch.object(cli.list_ports, "comports", return_value=boards):
            with self.assertRaises(RuntimeError):
                cli.select_port()
            self.assertEqual(cli.select_port("COM9"), "COM9")


if __name__ == "__main__":
    unittest.main()
