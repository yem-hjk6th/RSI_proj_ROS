"""PR #44 review, follow-up 2: the failure map's gate log check in the real session flow, without ROS.

The layout is built by the code the bringup and the export run, not by hand:
- the session folder and the path of session_meta.json come from ``startup.session_paths`` (the bringup launch);
- the readouts open their gate logs where the nodes do: ``gate_log_folder("", <session_meta>)``, i.e. next to
  session_meta.json (ELP: ``width_readout_elp.config.make_readout``; bore1: ``path_geometry.gates.GateLog`` in that
  folder, written by ``width_readout_bore1.readout.Bore1Readout`` on a synthetic RSI track along ARC2 row A1);
- the bore1 frames become Bore1Record-like messages (``record.record_kwargs``, stamp = frame time, as the node
  publishes them), and ``print_session.export.export`` writes the legacy layout.

Asserted: ``Session`` finds session_meta.json at the session root (the session id from ``print_session.session``),
the export copies ``semi_gate_<camera>.csv`` into ``raw_video/<ses>/`` with ``t`` on the exported clock, and
``gate_log_check`` joins every gated bore1 frame with its gate log row.
"""
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

pd = pytest.importorskip("pandas")
pytest.importorskip("path_geometry")

from path_geometry.failure_map import Session, bores, gate_log_check, gate_log_files  # noqa: E402
from path_geometry.gates import GATE_LOG_COLS, GateLog  # noqa: E402
from path_geometry.geometry import find_pattern  # noqa: E402
from print_session import topics as T  # noqa: E402
from print_session.export import export  # noqa: E402
from print_session.startup import SESSION_META, default_rig, session_paths  # noqa: E402
from width_readout_bore1.gate import load_pattern  # noqa: E402
from width_readout_bore1.head import make_head  # noqa: E402
from width_readout_bore1.readout import B1_COLS, Bore1Readout  # noqa: E402
from width_readout_bore1.record import record_kwargs, sec_ns  # noqa: E402
from width_readout_elp.config import gate_log_folder, make_readout  # noqa: E402
from width_readout_elp.rsi_ring import RsiRing  # noqa: E402

ARC2 = "pattern_usr_ELP_ARC2_261103.json"
SES = "20261104_120000"
T0 = 1_793_000_000.0            # stamp clock (epoch seconds, as KrcState.stamp)
WALL_MINUS_PERF = 1_792_990_000.123457


def _time(t):
    s, ns = sec_ns(t)
    return NS(sec=s, nanosec=ns)


def _track(pat, leg=1, s0=10.0, v=12.0, dt=0.004, n=1200):
    """RSI packets along ARC2 row A<leg> (the bore gate's -X lines), from s0 at v mm/s"""
    ln = pat.lines.get("A", leg)
    t = T0 + dt * np.arange(n)
    P, _ = ln.points(s0 + v * (t - T0))
    return t, np.column_stack([P, pat.z_cmd(P[:, 0])])


def _session(tmp_path):
    paths = session_paths(tmp_path / "sessions", SES)
    paths.dir.mkdir(parents=True)
    paths.meta.write_text(json.dumps({"realtime": {"pattern": str(find_pattern(ARC2))},
                                      "print_session": {"session": SES, "bag": str(paths.bag)}}), encoding="utf-8")
    return paths


def _run_readouts(tmp_path, paths):
    gdir = gate_log_folder("", str(paths.meta))
    _, elp_log = make_readout(str(default_rig()), ARC2, None, gdir, "elp")
    assert elp_log is not None
    elp_log.close()
    replay = tmp_path / "bore1_rt_empty.csv"
    replay.write_text(",".join(B1_COLS) + "\n", encoding="utf-8")          # no logged predictions
    pat = load_pattern(ARC2)
    log = GateLog(Path(gdir) / "semi_gate_bore1.csv")        # as the bore1 node opens it
    ro = Bore1Readout(pat, make_head("replay", replay_csv=str(replay)), gate_log=log, camera="bore1")
    t, P = _track(pat)
    ring = RsiRing()
    for k in range(len(t)):
        ring.push(float(t[k]), P[k])
    frames = T0 + 0.2 + (1.0 / 29.97) * np.arange(int((t[-1] - T0 - 0.2) * 29.97))
    rows = [ro.process(float(tf), ring, i + 1, frame=np.zeros((2, 2, 3), np.uint8), clock=lambda: 0.0)
            for i, tf in enumerate(frames)]
    log.close()
    states = [NS(stamp=_time(float(tk)), ipoc=4 * k, rist=[*P[k], 0.0, 0.0, 0.0], override_pct=100,
                 vel_act_mm_s=12.0, rpm_ext=0) for k, tk in enumerate(t)]
    replies = [NS(stamp=s.stamp, pc_ts_perf=float(tk) - WALL_MINUS_PERF, ipoc=s.ipoc, ov_cmd=100, source="default",
                  delay=0) for s, tk in zip(states, t)]
    recs = [NS(stamp=_time(r.t_frame), **record_kwargs(r)) for r in rows]
    return pat, rows, {T.STATE: states, T.REPLY: replies, T.BORE1_RECORD: recs}


def test_gate_log_check_in_the_session_flow(tmp_path):
    paths = _session(tmp_path)
    sess = Session(paths.dir)                                  # the bringup's folder: meta at the root
    assert sess.meta_path == paths.meta and sess.ses == SES and sess.pattern_name().endswith(ARC2)
    pat, rows, msgs = _run_readouts(tmp_path, paths)
    ok = [r for r in rows if r.gate == "ok"]
    assert len(ok) >= 30 and len(ok) < len(rows)               # gated frames and rejected ones (the track's start)
    assert sorted(p.name for p in paths.dir.glob("semi_gate_*.csv")) == ["semi_gate_bore1.csv", "semi_gate_elp.csv"]

    out = tmp_path / "export"
    counts = export(msgs, out, SES, meta_path=paths.meta)
    assert counts["semi_gate_bore1.csv"] == len(ok) and counts["semi_gate_elp.csv"] == 0
    vd = out / "raw_video" / SES
    assert (vd / SESSION_META).is_file() and (vd / "semi_gate_bore1.csv").is_file()

    sess = Session(out)
    assert sess.ses == SES and sess.meta_path == vd / SESSION_META
    assert sorted(f.name for f in gate_log_files(sess)) == ["semi_gate_bore1.csv", "semi_gate_elp.csv"]
    B = bores(pat, sess)
    assert len(B) == len(ok)
    m = gate_log_check(B, sess)
    assert m is not None and len(m) == len(ok)                 # every gated frame joined with its gate log row
    assert m.t.dtype == float and m.s_tip_y.dtype == float     # the header-only ELP log does not turn them to object
    assert (m.leg_x == m.leg_y).all() and np.allclose(m.s_tip_x, m.s_tip_y, atol=1e-3)
    exported = pd.read_csv(vd / "semi_gate_bore1.csv")
    assert list(exported.columns) == GATE_LOG_COLS
    raw = pd.read_csv(paths.dir / "semi_gate_bore1.csv")
    assert np.allclose(exported.t + WALL_MINUS_PERF, raw.t, rtol=0, atol=2e-6)   # stamp clock -> exported clock
    assert np.allclose(exported.t, B.t_frame, rtol=0, atol=1e-6) and (exported.t.to_numpy() == B.t_frame.to_numpy()).all()


def test_gate_logs_are_copied_unchanged_without_rsi_packets(tmp_path):
    paths = _session(tmp_path)
    _, rows, msgs = _run_readouts(tmp_path, paths)
    counts = export({T.BORE1_RECORD: msgs[T.BORE1_RECORD]}, tmp_path / "export", SES, meta_path=paths.meta)
    assert counts["semi_gate_bore1.csv"] == sum(r.gate == "ok" for r in rows)
    got = (tmp_path / "export" / "raw_video" / SES / "semi_gate_bore1.csv").read_text(encoding="utf-8")
    assert got.splitlines() == (paths.dir / "semi_gate_bore1.csv").read_text(encoding="utf-8").splitlines()
    assert not math.isnan(float(got.splitlines()[1].split(",")[1]))
