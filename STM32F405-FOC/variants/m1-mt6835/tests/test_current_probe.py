"""Reject stale overview bytes before an old-layout current capture."""
import sys
from pathlib import Path
import struct
import unittest
from unittest.mock import patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools' / 'bench'))
import current_probe

TAIL = b'\x00\x00\x80\x7f'

def frame(seq):
    return struct.pack('<II10f', (4 << 24) | (seq * 100),
                       (5 << 24) | seq, *([0.0] * 9 + [12.0])) + TAIL

class Link:
    in_waiting = 100
    def __init__(self):
        self.clock = 0.0
        self.seq = 500
        self.commands = []
    def write(self, data):
        self.commands.append(data)
    def read(self, size):
        self.clock += 0.001
        if self.clock < 0.05:
            # A 100-byte overview whose last 52 bytes mimic a valid old frame.
            return bytes(48) + frame(42)
        self.seq += 1
        return frame(self.seq)

class CurrentCaptureTest(unittest.TestCase):
    def test_overview_tail_is_drained_before_frame_identity(self):
        link = Link()
        with patch.object(current_probe.time, 'monotonic', lambda: link.clock):
            raw, events, skipped = current_probe.collect(link, 5, 0.01)
        words = np.frombuffer(raw, dtype='<u4').reshape(-1, 13)
        self.assertTrue(np.all((words[:, 1] & 0xffffff) > 500))
        self.assertTrue(np.all(np.diff(words[:, 1] & 0xffffff) == 1))
        self.assertEqual(link.commands, [b'send 5\n'])
        self.assertEqual(events, [])

if __name__ == '__main__':
    unittest.main()
