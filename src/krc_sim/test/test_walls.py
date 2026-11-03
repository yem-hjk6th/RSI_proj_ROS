"""krc_sim on the wall programs (T12): planned Z per segment, layers, +X and -X passes, lifts."""
import csv
import math
import xml.etree.ElementTree as ET

import pytest
from sim_paths import PATTERN, WALL_PATTERN

from krc_sim.path import PatternPath
from krc_sim.sim import KrcSim, SimConfig


def _rows():
    with open(WALL_PATTERN.parent / "usr_ELP_W4L28_CL_261023_segs.csv", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _z_cmd(x):
    return 0.8 + (2.3 - 0.8) * (x - 600.0) / (970.0 - 600.0)


def test_wall_path_follows_the_planned_z():
    p = PatternPath(WALL_PATTERN, lead_in_mm=20.0)
    rows = _rows()
    assert p.layered and len(p.segs) == len(rows) + 1           # every row has a length (lifts climb 1.3 mm)
    for sg, r in zip(p.segs[1:], rows):
        assert sg.p0 == (float(r["x0"]), float(r["y0"]), float(r["z0"]))
        assert sg.p1 == (float(r["x1"]), float(r["y1"]), float(r["z1"]))
        assert sg.row == r
    assert p.row is None                                         # on the lead-in
    p.i = 2
    assert p.row["kind"] == "A" and p.row["leg"] == "1" and p.row["layer"] == "1"
    lead = p.segs[0]
    assert lead.p0[0] == 965.0 and lead.p1 == p.segs[1].p0
    assert lead.p0[2] - _z_cmd(965.0) == pytest.approx(lead.p1[2] - _z_cmd(945.0))
    assert [round(s.length, 6) for s, r in zip(p.segs[1:], rows) if r["kind"] == "lift"] == [1.3] * 27


def test_wall_path_prints_every_layer_both_directions():
    p = PatternPath(WALL_PATTERN, lead_in_mm=0.0)
    passes = []                  # (y, direction, z - z_cmd(x) at the middle of the pass)
    prev = p.pos
    k = 0
    while not p.done:
        p.step(0.004, 100)
        x, y, z = p.pos
        k += 1
        if abs(x - 857.5) < 0.2 and abs(x - prev[0]) > 0:
            d = "-X" if x < prev[0] else "+X"
            if not passes or passes[-1][:2] != (y, d) or k - passes[-1][3] > 10:
                passes.append((y, d, round(z - _z_cmd(x), 2), k))
        prev = (x, y, z)
    assert p.pos == pytest.approx((945.0, -505.0, 37.3))
    total = sum(s.length / s.v100 for s in p.segs)
    assert k == math.ceil(total / 0.004 - 1e-9)
    assert len(passes) == 4 * 28
    walls = {-505.0: "W1", -480.0: "W2", -455.0: "W3", -430.0: "W4"}
    for layer in range(28):
        got = [(walls[y], d, dz) for y, d, dz, _ in passes[4 * layer:4 * layer + 4]]
        dz = round(1.3 * layer, 2)
        order = ["W1", "W2", "W3", "W4"] if layer % 2 == 0 else ["W4", "W3", "W2", "W1"]
        assert got == [(order[0], "-X", dz), (order[1], "+X", dz), (order[2], "-X", dz), (order[3], "+X", dz)], layer


def test_single_layer_path_still_follows_z_cmd():
    p = PatternPath(PATTERN, lead_in_mm=20.0)
    assert not p.layered
    for s in p.segs:
        assert s.p0[2] == _z_cmd(s.p0[0]) and s.p1[2] == _z_cmd(s.p1[0])


def test_sim_packets_carry_the_layer_z():
    sim = KrcSim(SimConfig(pattern_file=str(WALL_PATTERN), lead_in_mm=0.0))
    sim._consec = 0
    zs = []
    for k in range(3):
        _, pkt = sim._next_packet(k)
        zs.append(float(ET.fromstring(pkt).find("RIst").get("Z")))
    assert zs == pytest.approx([2.2] * 3)
    sim.path.i = next(i for i, s in enumerate(sim.path.segs) if s.p0[2] > 30.0)
    _, pkt = sim._next_packet(3)
    assert float(ET.fromstring(pkt).find("RIst").get("Z")) > 30.0
