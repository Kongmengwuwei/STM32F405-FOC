"""Verify signed multi-turn endpoint conversion and reducer coordinate limits."""

from copy import deepcopy
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "bench"))
from leg_teach import complete, raw_mean

record = {"teach": {"motor_anchor_deg": [48.65, 213.93]}}
a = np.zeros((100, 24))
a[:, :2] = [90, -87]
a[:, 13:15] = [48.65, 213.93]
a[:, 4:6] = [318.65, 324.93]
r = complete(deepcopy(record), a, 2)["boundary_B"]
assert r["encoder_sign_toward_B"] == [1, -1]
assert r["joint_travel_deg"] == [90, 87]
assert np.allclose(r["motor_unwrapped_deg"], [678.65, -395.07])
assert r["candidate_joint_interval_positive_toward_B_deg"] == [[2, 88], [2, 85]]
a[:, :2] = [-113.60, 115.65]
wide = complete(deepcopy(record), a, 2)
assert wide["boundary_B"]["requires_range_confirmation"]
assert not wide["boundary_B"]["limits_enabled_for_drive"]
assert wide["boundary_B"]["encoder_sign_toward_B"] == [-1, 1]
a[:, :2] = [90, -87]
for kind in ["anchor", "span", "margin"]:
    bad = a.copy()
    margin = 2
    if kind == "anchor":
        bad[:, 13] += 0.01
    if kind == "span":
        bad[:, 0] = 0.5
    if kind == "margin":
        margin = 45
    try:
        complete(deepcopy(record), bad, margin)
    except RuntimeError:
        pass
    else:
        raise AssertionError(kind)
a[:50, 4:6] = 359.99
a[50:, 4:6] = 0.01
assert np.max(np.abs((raw_mean(a) + 180) % 360 - 180)) < 0.00001
print(
    "PASS: signed reducer travel, multi-turn endpoints, margins, anchor validation, circular mean"
)
