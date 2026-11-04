"""T13 acceptance 2-4 on both sessions of 2026-11-03 (FAN1-CL, ARC2-OL), every logged frame of the four cameras.

2. Real-log golden:
   a. the port (``gates.elp_gate`` / ``gates.bore_gate``) equals the legacy gates run offline on the same ring at the
      same frame time: reason and leg exactly, s_tip, s_dep and e within 1e-9 mm (cuts: ELP t_frame_perf and
      t_done_perf, bores t_done);
   b. the legacy offline gates equal the live logs. ELP: decision and leg on every frame, s_tip within 0.001 mm, for
      both cuts. Bores at the t_done cut: reason and leg on every frame, s_tip within 0.1 mm, and every frame that
      differs by more than the printed precision (``PRINT_TOL``: 0.0005 mm plus the 1-us rounding of the logged
      times) is reproduced (reason, leg, s_tip within ``PRINT_TOL``) by some cut in [t_frame, t_done].
3. Gate log (ARC2-OL): the port's rows equal ``semi_gate_20261103_235936.csv`` (cam, leg, row, curv, lobe exactly;
   s and e within 0.0005 mm; delta within 0.01 deg; R within 0.0005 mm), and the legacy ``_GateLog`` strings on the
   same ring exactly.
4. Angle cap: port = legacy for max_deg in {0, 10, 15, 30, 45, 90} on every bore frame of both sessions (cut
   t_done); the counts of ``angle`` rejections are the task file's.
"""
import csv
import gzip
import math

import numpy as np
import pytest

from gate_sessions import BORES, CAMS, ELPS, available, frame_times, legacy_bore_gate, legacy_e, legacy_elp, \
    live, port_pattern, session
from path_geometry.gates import GATE_LOG_COLS, LAB_BACK, bore_gate, elp_gate, gate_log_row, gate_log_values

pytestmark = pytest.mark.skipif(not available(), reason="session data not present")

SESSIONS = ("fan1", "arc2")
F_TOL = 1e-9
# The logs print s_tip to 3 decimals and every time (t_frame, pc_ts_perf) to 1 us. A value equal to the live one can
# therefore differ from the printed one by 0.0005 mm plus the time rounding: at most 2 us x 30 mm/s = 6e-5 mm.
PRINT_TOL = 5e-4 + 6e-5
CAPS = (0.0, 10.0, 15.0, 30.0, 45.0, 90.0)
# task file, acceptance 4: frames rejected as angle at cap 0 / 15 / 30 / 45 / 90 (legacy gates, t_done cut)
ANGLE_COUNTS = {("fan1", "bore1"): (835, 476, 122, 0, 0), ("fan1", "bore2"): (809, 342, 111, 0, 0),
                ("arc2", "bore1"): (1219, 1019, 814, 612, 0), ("arc2", "bore2"): (1229, 1025, 824, 621, 0)}
REPORT = {}


def _port_and_legacy(S, cam, cut, caps=(90.0,)):
    """{(frame i, cap): (port (g, why), legacy (g, why))} at the cut (column name)"""
    kind = CAMS[cam][0]
    pat = port_pattern(S.name)
    tf, td = frame_times(S, cam)
    tc = tf if cut == "frame" else td
    if cam in ELPS:
        lg = legacy_elp(S, kind)
        lt = lg.rig.lat

        def fn(ring, i):
            g = lg.gate(float(tf[i]), ring)
            return {90.0: (elp_gate(pat, ring, float(tf[i]), kind, lt), (g, "" if g is not None else lg.last_skip))}
    else:
        lgs = {c: legacy_bore_gate(S, kind, c) for c in caps}

        def fn(ring, i):
            return {c: (bore_gate(pat, ring, float(tf[i]), kind, c), lgs[c](S.legacy_pat, ring, float(tf[i])))
                    for c in caps}
    ks = S.k_at(tc)
    return S.sweep([(int(k), i) for i, k in enumerate(ks)], fn)


@pytest.fixture(scope="module")
def results():
    out = {}
    for name in SESSIONS:
        S = session(name)
        for cam in BORES:
            out[name, cam, "done"] = _port_and_legacy(S, cam, "done", CAPS)
        for cam in ELPS:
            for cut in ("frame", "done"):
                out[name, cam, cut] = _port_and_legacy(S, cam, cut)
    return out


def _assert_same(S, cam, i, port, leg):
    (gp, wp), (gl, wl) = port, leg
    kind = CAMS[cam][0]
    assert wp == wl, (S.name, cam, i)
    assert (gp is None) == (gl is None), (S.name, cam, i)
    if gp is None:
        return 0
    assert gp.leg == gl.leg, (S.name, cam, i)
    assert np.array_equal(gp.tip, gl.tip)
    assert abs(gp.s_tip - gl.s_tip) <= F_TOL
    assert abs(gp.e - legacy_e(S, kind, gl.tip)) <= F_TOL
    if cam in BORES:
        assert abs(gp.s_dep - (gl.s_tip - LAB_BACK)) <= F_TOL
    return 1


@pytest.mark.parametrize("name", SESSIONS)
@pytest.mark.parametrize("cam", BORES + ELPS)
def test_port_equals_legacy_offline(results, name, cam):
    S = session(name)
    n_frames = len(S.frames[cam])
    for cut in (("done",) if cam in BORES else ("frame", "done")):
        res = results[name, cam, cut]
        assert len(res) == n_frames
        n_ok = 0
        for i in range(n_frames):
            for cap, (p, lg) in res[i].items():
                n_ok += _assert_same(S, cam, i, p, lg)
        assert n_ok > 0


@pytest.mark.parametrize("name", SESSIONS)
@pytest.mark.parametrize("cam", ELPS)
def test_legacy_offline_equals_live_elp(results, name, cam, capsys):
    S = session(name)
    gated, _, leg, s_tip = live(S, cam)
    for cut in ("frame", "done"):
        res = results[name, cam, cut]
        g = [res[i][90.0][1][0] for i in range(len(gated))]
        dec = np.array([x is not None for x in g])
        assert np.array_equal(dec, gated), (name, cam, cut, int((dec != gated).sum()))
        lg = np.array([x.leg if x is not None else -1 for x in g])
        assert np.array_equal(lg, leg)
        ds = np.abs(np.array([x.s_tip for x in g if x is not None]) - s_tip[gated])
        assert ds.max() <= 1e-3, (name, cam, cut, ds.max())
        REPORT[name, cam, cut] = f"{len(g)} frames, {int(gated.sum())} gated, decision and leg equal, max |ds_tip| " \
                                 f"{ds.max():.4f} mm"
    with capsys.disabled():
        for cut in ("frame", "done"):
            print(f"\n[legacy offline vs live] {name} {cam} cut t_{cut}: {REPORT[name, cam, cut]}")


def _bore_cut_search(S, cam, frames):
    """{frame i: [(k, why, leg, s_tip)]} of the legacy gate (cap 90) at every cut in [t_frame, t_done]"""
    kind = CAMS[cam][0]
    g = legacy_bore_gate(S, kind, 90.0)
    tf, td = frame_times(S, cam)
    q = []
    for i in frames:
        for k in range(int(S.k_at(tf[i])), int(S.k_at(td[i])) + 1):
            q.append((k, (i, k)))

    def fn(ring, key):
        gl, why = g(S.legacy_pat, ring, float(tf[key[0]]))
        return ("ok", gl.leg, gl.s_tip) if gl is not None else (why, -1, math.nan)
    out = {}
    for (i, k), v in sorted(S.sweep(q, fn).items()):
        out.setdefault(i, []).append((k, *v))
    return out


@pytest.fixture(scope="module")
def bore_live(results):
    """per (session, cam): the frames whose s_tip differs by more than 0.0005 mm at the t_done cut, and the cut that
    reproduces each of them"""
    out = {}
    for name in SESSIONS:
        S = session(name)
        for cam in BORES:
            res = results[name, cam, "done"]
            gated, why, leg, s_tip = live(S, cam)
            g = [res[i][90.0][1] for i in range(len(gated))]
            off = [i for i in range(len(g)) if g[i][0] is not None and abs(g[i][0].s_tip - s_tip[i]) > PRINT_TOL]
            search = _bore_cut_search(S, cam, off)
            match = {}
            for i in off:
                hits = [k for k, w, lg, s in search[i] if w == "ok" and lg == leg[i] and abs(s - s_tip[i]) <= PRINT_TOL]
                match[i] = hits[0] if hits else None
            out[name, cam] = (g, off, match)
    return out


@pytest.mark.parametrize("name", SESSIONS)
@pytest.mark.parametrize("cam", BORES)
def test_legacy_offline_equals_live_bore(bore_live, name, cam, capsys):
    gated, why, leg, s_tip = live(session(name), cam)
    g, off, match = bore_live[name, cam]
    w = np.array(["ok" if x is not None else r for x, r in g])
    assert np.array_equal(w, why), (name, cam, int((w != why).sum()))
    lg = np.array([x.leg if x is not None else -1 for x, _ in g])
    assert np.array_equal(lg, leg)
    ds = np.abs(np.array([x.s_tip for x, _ in g if x is not None]) - s_tip[gated])
    assert ds.max() <= 0.1, ds.max()
    missing = [i for i in off if match[i] is None]
    assert not missing, (name, cam, missing[:10])
    msg = (f"{len(w)} frames ({int(gated.sum())} ok), reason and leg equal at t_done; s_tip differs by more than the "
           f"printed precision on {len(off)} frames (max {ds.max():.3f} mm), each reproduced by an earlier cut in "
           f"[t_frame, t_done]")
    with capsys.disabled():
        print(f"\n[legacy offline vs live] {name} {cam}: {msg}")


# ------------------------------------------------------------------------------------------------ angle cap
@pytest.mark.parametrize("name", SESSIONS)
@pytest.mark.parametrize("cam", BORES)
def test_angle_cap_port_equals_legacy(results, name, cam, capsys):
    res = results[name, cam, "done"]
    counts = {}
    for cap in CAPS:
        n = 0
        for i in range(len(res)):
            (gp, wp), (gl, wl) = res[i][cap]
            assert wp == wl and (gp is None) == (gl is None), (name, cam, cap, i)
            if gp is not None:
                assert gp.leg == gl.leg and gp.s_tip == gl.s_tip
            n += wp == "angle"
        counts[cap] = n
    exp = ANGLE_COUNTS[name, cam]
    assert tuple(counts[c] for c in (0.0, 15.0, 30.0, 45.0, 90.0)) == exp
    assert counts[0.0] > 0 and counts[90.0] == 0
    with capsys.disabled():
        print(f"\n[angle cap] {name} {cam}: angle rejections at cap " +
              ", ".join(f"{c:g}: {counts[c]}" for c in CAPS))


@pytest.mark.parametrize("name", SESSIONS)
def test_angle_at_obs(name, capsys):
    """angle_at="obs" tests delta at s_tip - 5; s_tip - 5 < 0 uses the start tangent's angle and is flagged. On fan
    lines delta is constant, so obs = tip. Only the angle test changes: every other reason stays."""
    S = session(name)
    pat = port_pattern(name)
    out = []
    for cam in BORES:
        kind = CAMS[cam][0]
        tf, td = frame_times(S, cam)

        def fn(ring, i):
            return {(c, a): bore_gate(pat, ring, float(tf[i]), kind, c, a)
                    for c in (15.0, 90.0) for a in ("tip", "obs")}
        res = S.sweep([(int(k), i) for i, k in enumerate(S.k_at(td))], fn)
        n_diff = n_flag = 0
        for i, r in res.items():
            for c in (15.0, 90.0):
                (gt, wt), (go, wo) = r[c, "tip"], r[c, "obs"]
                assert wt == wo or "angle" in (wt, wo)
                if go is not None:
                    assert go.angle_at == "obs" and go.s_angle == go.s_dep == go.s_tip - LAB_BACK
                    assert go.delta == go.line.delta_at(go.s_dep)
                    if go.obs_before_start:
                        assert go.s_dep < 0 and go.delta == go.line.delta_at(0.0)
                if gt is not None:
                    assert gt.s_angle == gt.s_tip and gt.obs_before_start == (gt.s_dep < 0)
            g90 = r[90.0, "obs"][0]
            n_flag += g90 is not None and g90.obs_before_start
            n_diff += r[15.0, "tip"][1] != r[15.0, "obs"][1]
        if name == "fan1":
            assert n_diff == 0
        else:
            assert n_diff > 0
        out.append(f"{cam}: cap 15 tip vs obs differ on {n_diff} frames; {n_flag} passed frames with s_dep < 0 "
                   f"(start tangent angle)")
    with capsys.disabled():
        print(f"\n[angle_at obs] {name}: " + "; ".join(out))


def test_s_dep_before_start_count_arc2(results, capsys):
    """T14 review: 78 ARC2 bore rows have s_dep < 0 and are binned by the start tangent angle. They are the frames the
    gate flags with obs_before_start (cap 90, tip)."""
    n = 0
    for cam in BORES:
        res = results["arc2", cam, "done"]
        for i in range(len(res)):
            g = res[i][90.0][0][0]
            if g is not None and g.obs_before_start:
                assert g.s_dep < 0 and g.line.delta_at(g.s_dep) == g.line.delta_at(0.0)
                n += 1
    assert n == 78
    with capsys.disabled():
        print(f"\n[s_dep < 0] arc2: {n} bore frames flagged obs_before_start")


# ------------------------------------------------------------------------------------------------ gate log
def _read_semi_gate():
    S = session("arc2")
    from legacy_paths import ARC2
    p = ARC2 / "roboter" / f"semi_gate_{S.ses}.csv.gz"
    with gzip.open(p, "rt", newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        assert next(r) == GATE_LOG_COLS
        return list(r)


@pytest.fixture(scope="module")
def gate_log_rows(results, bore_live):
    """port and legacy gate log rows of every gated frame, at the cut that reproduces the live s_tip: ELP t_frame_perf;
    bores t_done, or the matched earlier cut"""
    S = session("arc2")
    pat = port_pattern("arc2")
    legacy_rows = []

    class Capture:
        _w = True

        def row(self, *a):
            legacy_rows.append(a)
    SG = S.legacy_mod
    q = []
    for cam in CAMS:
        tf, td = frame_times(S, cam)
        gated, *_ = live(S, cam)
        k_cut = S.k_at(tf if cam in ELPS else td)
        if cam in BORES:
            _, off, match = bore_live["arc2", cam]
            for i in off:
                k_cut[i] = match[i]
        q += [(int(k_cut[i]), (cam, i)) for i in np.flatnonzero(gated)]
    gates = {c: legacy_bore_gate(S, CAMS[c][0], 90.0) for c in BORES}
    gates["elpA"], gates["elpB"] = legacy_elp(S, "A"), legacy_elp(S, "B")

    def fn(ring, key):
        cam, i = key
        kind = CAMS[cam][0]
        t = float(frame_times(S, cam)[0][i])
        old = SG.GATE_LOG
        SG.GATE_LOG = Capture()
        try:
            n0 = len(legacy_rows)
            if cam in ELPS:
                gates[cam].gate(t, ring)
                g, why = elp_gate(pat, ring, t, kind, gates[cam].rig.lat)
            else:
                gates[cam](S.legacy_pat, ring, t)
                g, why = bore_gate(pat, ring, t, kind, 90.0)
        finally:
            SG.GATE_LOG = old
        assert g is not None and len(legacy_rows) == n0 + 1, key
        a = legacy_rows[-1]
        legacy = [str(v) for v in _legacy_format(*a)]
        back = 0.0 if cam in ELPS else LAB_BACK
        port = [str(v) for v in gate_log_row(cam, t, g.line, g.s_tip, g.e, back)]
        return port, legacy, gate_log_values(cam, t, g.line, g.s_tip, g.e, back)
    return S.sweep(q, fn)


def _legacy_format(cam, t, ln, s_tip, e, back):
    """the strings legacy _GateLog.row writes (the same code, without a file)"""
    so = s_tip - back
    return [cam, f"{t:.6f}", ln.leg, ln.name, f"{s_tip:.3f}", f"{e:.3f}", f"{ln.delta_at(s_tip):+.2f}",
            ln.curv_at(s_tip), f"{ln.R_at(s_tip):.3f}", ln.lobe_at(s_tip), f"{so:.3f}", f"{ln.delta_at(so):+.2f}",
            ln.curv_at(so), f"{ln.R_at(so):.3f}", ln.lobe_at(so)]


def test_legacy_gate_log_format_is_the_legacy_code():
    """_legacy_format is a copy of semi_geom._GateLog.row's formatting: check it against the legacy class itself."""
    import io
    S = session("arc2")
    SG = S.legacy_mod
    log = SG._GateLog()
    buf = io.StringIO()
    log._w, log._f = csv.writer(buf), buf
    ln = S.legacy_pat.lines.get("A", 2)
    for s in (0.3, 4.0, 30.0, 61.0, 200.0):
        log.row("bore1", 1774138.5, ln, s, 0.1234, LAB_BACK)
        log.row("elpA", 1774138.5, ln, s, -0.02, 0.0)
    rows = list(csv.reader(io.StringIO(buf.getvalue())))
    exp = [[str(v) for v in _legacy_format(c, 1774138.5, ln, s, e, b)]
           for s in (0.3, 4.0, 30.0, 61.0, 200.0) for c, e, b in (("bore1", 0.1234, LAB_BACK), ("elpA", -0.02, 0.0))]
    assert rows == exp


def test_gate_log_port_equals_legacy_strings(gate_log_rows):
    for key, (port, legacy, _) in gate_log_rows.items():
        assert port == legacy, key


def test_gate_log_equals_semi_gate_csv(gate_log_rows, capsys):
    """The port's unrounded values against the printed live rows: s and e within 0.0005 mm (``PRINT_TOL``: plus the
    1-us rounding of the logged times), delta within 0.01 deg, R within 0.0005 mm."""
    live_rows = _read_semi_gate()
    by = {}
    for r in live_rows:
        assert (r[0], r[1]) not in by
        by[r[0], r[1]] = r
    port = {(p[0], p[1]): (p, v) for p, _, v in gate_log_rows.values()}
    assert set(port) == set(by), (len(port), len(by), len(set(port) ^ set(by)))
    cols = {c: j for j, c in enumerate(GATE_LOG_COLS)}
    worst = dict(s=0.0, e=0.0, delta=0.0, R=0.0)
    for key, (p, v) in port.items():
        r = by[key]
        for c in ("cam", "leg", "row", "curv_tip", "lobe_tip", "curv_obs", "lobe_obs"):
            assert p[cols[c]] == r[cols[c]], (key, c)
        for c, w, tol in (("s_tip", "s", PRINT_TOL), ("s_obs", "s", PRINT_TOL), ("e", "e", PRINT_TOL),
                          ("delta_tip", "delta", 0.01), ("delta_obs", "delta", 0.01), ("R_tip", "R", 5e-4),
                          ("R_obs", "R", 5e-4)):
            a, b = float(v[c]), float(r[cols[c]])
            if w == "R" and math.isinf(a) and math.isinf(b):
                continue
            d = abs(a - b)
            if w == "delta":
                d = min(d, 360.0 - d)          # -180 and +180 are the same angle
            assert d <= tol + 1e-9, (key, c, v[c], r[cols[c]])
            worst[w] = max(worst[w], d)
    cams = {}
    for c, _ in port:
        cams[c] = cams.get(c, 0) + 1
    with capsys.disabled():
        print(f"\n[gate log] arc2: {len(port)} rows equal semi_gate ({cams}); max |ds| {worst['s']:.5f} mm, "
              f"|de| {worst['e']:.5f} mm, |d delta| {worst['delta']:.4f} deg, |dR| {worst['R']:.5f} mm")
