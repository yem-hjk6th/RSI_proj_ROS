"""Acceptance test 1: the geometry equals the legacy semi_geom / fan_geom classes (floats within 1e-9, ints and names
exactly), on every ARC2 chord vertex and on random points within 2 mm of the legs; plus the pattern copies and the
load-time checks."""
import csv
import json
import math
import shutil
from pathlib import Path

import numpy as np
import pytest

from legacy_paths import ARC2_CHORDS, ARC2_JSON, FAN1_JSON, LEGACY_PATTERNS, import_legacy
from path_geometry import FanLine, PathLine, load_pattern, patterns_dir

TOL = 1e-9
N_RANDOM = 10_000
KIND_SETS = (("A",), ("B",), ("A", "B"))


def _same_float(a, b):
    a, b = float(a), float(b)
    if math.isnan(b):
        return math.isnan(a)
    if math.isinf(b):
        return a == b
    return abs(a - b) <= TOL


def _near_points(pat, n, seed):
    """n points within 2 mm of the legs: uniform s along a random leg (and up to 1 mm past its ends), offset
    uniform in [-2, 2] mm along the local normal"""
    rng = np.random.default_rng(seed)
    lines = list(pat.lines)
    out = np.empty((n, 2))
    for i in range(n):
        ln = lines[rng.integers(len(lines))]
        s = rng.uniform(-1.0, ln.L + 1.0)
        off = rng.uniform(-2.0, 2.0)
        if isinstance(ln, PathLine):
            P, nrm = ln.points(np.array([s]))
            out[i] = P[0] + off * nrm[0]
        else:
            out[i] = ln.p0 + s * ln.t + off * ln.n
    return out


def _chord_vertices():
    with open(LEGACY_PATTERNS / ARC2_CHORDS, newline="", encoding="utf-8") as f:
        pts = [(float(r["x"]), float(r["y"])) for r in csv.DictReader(f)]
    with open(LEGACY_PATTERNS / "usr_ELP_ARC2_261103_segs.csv", newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            pts += [(float(r["x0"]), float(r["y0"])), (float(r["x1"]), float(r["y1"]))]
    return np.array(pts)


def _segs_ends(name):
    with open(LEGACY_PATTERNS / name, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return np.array([(float(r[f"x{k}"]), float(r[f"y{k}"])) for r in rows for k in (0, 1)])


@pytest.fixture(scope="module")
def semi_pair():
    SG = import_legacy("semi_geom")
    return load_pattern(patterns_dir() / ARC2_JSON), SG.SemiPattern(LEGACY_PATTERNS / ARC2_JSON)


@pytest.fixture(scope="module")
def fan_pair():
    FG = import_legacy("fan_geom")
    return load_pattern(patterns_dir() / FAN1_JSON), FG.FanPattern(LEGACY_PATTERNS / FAN1_JSON)


def test_patterns_byte_identical():
    names = sorted(p.name for p in LEGACY_PATTERNS.iterdir())
    assert names == sorted(p.name for p in patterns_dir().iterdir())
    for n in names:
        assert (patterns_dir() / n).read_bytes() == (LEGACY_PATTERNS / n).read_bytes(), n


def test_same_lines(semi_pair, fan_pair):
    port, leg = semi_pair
    assert [(ln.kind, ln.leg, ln.name, ln.mdf) for ln in port.lines] == \
        [(ln.kind, ln.leg, ln.name, ln.mdf) for ln in leg.lines.lines]
    for a, b in zip(port.lines, leg.lines.lines):
        assert _same_float(a.L, b.L) and _same_float(a.delta, b.delta)
        assert np.abs(a.p0 - b.p0).max() <= TOL and np.abs(a.p1 - b.p1).max() <= TOL
    port, leg = fan_pair
    assert [(ln.kind, ln.leg, ln.name, ln.mdf) for ln in port.lines] == \
        [(ln.kind, ln.leg, ln.name, ln.mdf) for ln in leg.lines.lines]
    for a, b in zip(port.lines, leg.lines.lines):
        assert isinstance(a, FanLine)
        assert _same_float(a.L, b.L) and a.delta == b.delta
        assert np.abs(a.p0 - b.p0).max() <= TOL and np.abs(a.t - b.t).max() <= TOL and np.abs(a.n - b.n).max() <= TOL


def _check_semi(port, leg, P):
    for a, b in zip(port.lines, leg.lines.lines):
        sa, ea = a.project(P)
        sb, eb = b.project(P)
        assert np.array_equal(np.isnan(sa), np.isnan(sb)) and np.array_equal(np.isinf(ea), np.isinf(eb))
        fin = np.isfinite(sb)
        assert np.abs(sa[fin] - sb[fin]).max(initial=0) <= TOL and np.abs(ea[fin] - eb[fin]).max(initial=0) <= TOL
        for s in sb[fin]:
            assert _same_float(a.delta_at(s), b.delta_at(s))
            assert a.curv_at(s) == b.curv_at(s) and a.lobe_at(s) == b.lobe_at(s)
            assert _same_float(a.R_at(s), b.R_at(s))
            assert np.abs(a.t_at(s) - b.t_at(s)).max() <= TOL and np.abs(a.n_at(s) - b.n_at(s)).max() <= TOL
    _check_locate(port, leg, P)


def _check_locate(port, leg, P):
    hits = 0
    for q in P:
        for kinds in KIND_SETS:
            ha, hb = port.lines.locate(q, kinds), leg.lines.locate(q, kinds)
            assert (ha is None) == (hb is None), (q, kinds)
            if hb is None:
                continue
            hits += 1
            assert (ha.line.kind, ha.line.leg, ha.line.name) == (hb.line.kind, hb.line.leg, hb.line.name)
            assert _same_float(ha.s, hb.s) and _same_float(ha.e, hb.e)
    assert hits > 0


def test_semi_chord_vertices(semi_pair):
    port, leg = semi_pair
    P = _chord_vertices()
    assert len(P) > 300
    _check_semi(port, leg, P)


def test_semi_random(semi_pair):
    port, leg = semi_pair
    _check_semi(port, leg, _near_points(port, N_RANDOM, 1))


def _check_fan(port, leg, P):
    for a, b in zip(port.lines, leg.lines.lines):
        sa, ea = a.project(P)
        d = P - b.p0
        assert np.abs(sa - d @ b.t).max() <= TOL and np.abs(ea - d @ b.n).max() <= TOL
        for s in sa:
            assert a.delta_at(s) == b.delta and a.curv_at(s) == 0 and a.lobe_at(s) == 0 and a.R_at(s) == math.inf
        assert np.abs(a.t_at(0.0) - b.t).max() <= TOL and np.abs(a.n_at(0.0) - b.n).max() <= TOL
    _check_locate(port, leg, P)


def test_fan_vertices(fan_pair):
    port, leg = fan_pair
    _check_fan(port, leg, _segs_ends("usr_ELP_FAN1_261103_segs.csv"))


def test_fan_random(fan_pair):
    port, leg = fan_pair
    _check_fan(port, leg, _near_points(port, N_RANDOM, 2))


def test_fan_delta_is_segs_delta(fan_pair):
    port, _ = fan_pair
    with open(LEGACY_PATTERNS / "usr_ELP_FAN1_261103_segs.csv", newline="", encoding="utf-8") as f:
        want = {(r["kind"], int(r["leg"])): float(r["delta_deg"]) for r in csv.DictReader(f) if r["kind"] in "AB"}
    assert {(ln.kind, ln.leg): ln.delta_at(ln.L / 2) for ln in port.lines} == want


def test_semi_sign_conventions(semi_pair):
    port, _ = semi_pair
    for ln in port.lines:
        for s in np.linspace(0, ln.L, 50):
            t, n = ln.t_at(s), ln.n_at(s)
            assert np.allclose(n, [-t[1], t[0]])                      # left normal
            a = math.degrees(math.atan2(t[1], t[0])) - (180.0 if ln.kind == "A" else 0.0)
            assert -180.0 <= ln.delta_at(s) < 180.0
            assert math.isclose(math.cos(math.radians(ln.delta_at(s) - a)), 1.0, abs_tol=1e-12)
        assert ln.delta_at(-5.0) == ln.delta_at(0.0) and ln.delta_at(ln.L + 5) == ln.delta_at(ln.L)


# ------------------------------------------------------------------------------------------------ loading
def _copy_patterns(tmp_path):
    for p in patterns_dir().iterdir():
        shutil.copyfile(p, tmp_path / p.name)
    return tmp_path


def test_unknown_geometry(tmp_path):
    d = _copy_patterns(tmp_path)
    for geo in ("arc", None, "Semi"):
        c = json.loads((d / ARC2_JSON).read_text())
        if geo is None:
            c.pop("geometry")
        else:
            c["geometry"] = geo
        (d / "p.json").write_text(json.dumps(c))
        with pytest.raises(ValueError):
            load_pattern(d / "p.json")


def _tamper(d: Path, name: str, row: int, col: str, value: str):
    p = d / name
    with open(p, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    rows[row][col] = value
    with open(p, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


@pytest.mark.parametrize("json_name,segs,row,col,value", [
    (ARC2_JSON, "usr_ELP_ARC2_261103_segs.csv", 2, "length_mm", "70.0"),       # length
    (ARC2_JSON, "usr_ELP_ARC2_261103_segs.csv", 2, "sweep", "1"),              # sweep vs phi0 / phi1
    (ARC2_JSON, "usr_ELP_ARC2_261103_segs.csv", 2, "cy", "-490.0"),            # end point not on the arc
    (ARC2_JSON, "usr_ELP_ARC2_261103_segs.csv", 3, "s0_mm", "62.0"),           # pieces not contiguous
    (ARC2_JSON, "usr_ELP_ARC2_261103_segs.csv", 0, "kind", "B"),               # kind vs direction in X
    (FAN1_JSON, "usr_ELP_FAN1_261103_segs.csv", 2, "tx", "0.9"),               # t vs end points
    (FAN1_JSON, "usr_ELP_FAN1_261103_segs.csv", 2, "ny", "-0.96593"),          # n_y > 0
    (FAN1_JSON, "usr_ELP_FAN1_261103_segs.csv", 2, "kind", "A"),               # kind vs t_x
    (FAN1_JSON, "usr_ELP_FAN1_261103_segs.csv", 2, "leg", "2"),                # duplicate (kind, leg)
])
def test_load_checks(tmp_path, json_name, segs, row, col, value):
    d = _copy_patterns(tmp_path)
    load_pattern(d / json_name)
    _tamper(d, segs, row, col, value)
    with pytest.raises(ValueError):
        load_pattern(d / json_name)
