"""CtrlRecord fields (cl_controller.record): the legacy ctrl_<ses>.csv columns, filled after every update.

Replays E1 through the port and checks every record against the port's own log_row (same update), the legacy column
list and the legacy formulas the log row does not have (rhat_f = u, ov_pub, empty cells)."""
import ast
import math

import pytest

import e1_data as E
from cl_controller.record import CTRL_COLS, TRI_STATE, ctrl_record

LEGACY_CTRL_COLS = ["t_perf", "t_wall", "n_upd", "win", "leg", "on_a", "s_tip", "n_bins", "veff", "w_est", "rhat",
                    "rhat_f", "logk", "slow", "have_est", "k", "ov_raw", "ov_round", "ov_pub", "src", "rows_age_s",
                    "forced", "plan", "state", "reason", "gate", "valid_elp", "why_elp", "rhat_elp", "lag_elp",
                    "valid_b1", "why_b1", "rhat_b1", "lag_b1", "e", "du", "u", "lnk_ff", "v_tip", "n_corr",
                    "n_switch", "w_target", "lnk_tgt", "nobed_s", "hold_bed", "lnk_ol", "du_blind", "t_blind"]


def test_columns_are_the_legacy_ctrl_cols():
    assert CTRL_COLS == LEGACY_CTRL_COLS
    run_cl = E.LEGACY / "run_cl.py"
    if run_cl.exists():                       # the list in legacy run_cl.py itself (read, not imported)
        tree = ast.parse(run_cl.read_text(encoding="utf-8"))
        cols = [ast.literal_eval(n.value) for n in tree.body
                if isinstance(n, ast.Assign) and getattr(n.targets[0], "id", "") == "CTRL_COLS"]
        assert cols == [CTRL_COLS]


def _same(a, b):
    if isinstance(a, float) or isinstance(b, float):
        fa = math.nan if b is None or b == "" else float(b)
        return (math.isnan(a) and math.isnan(fa)) or a == fa
    return a == b


@pytest.mark.skipif(not E.available(), reason="E1 data not present")
def test_records_match_the_log_rows_of_e1():
    from cl_controller.controller import Controller
    from cl_controller.params import build_configs
    from cl_controller.pattern import load_pattern
    from cl_controller.replay import replay

    d = E.load()
    pat = load_pattern(E.PATTERN)
    cfg, law_cfg, plateaus = build_configs(E.e1_params(), pat)
    ctrl = Controller(cfg, pat, law_cfg, plateaus)
    recs = []
    pt, pi, px = E.packets(d)
    rows = replay(ctrl, pt, pi, px, E.readouts(d), d["ctrl"]["t_perf"].to_numpy(float),
                  on_update=lambda now, c: recs.append(ctrl_record(c, now, now + 1000.0)))
    assert len(recs) == len(rows) == 1719
    keys = [k for k in rows[0] if k in CTRL_COLS and k != "t_perf"]   # log_row.t_perf is the controller clock
    for rec, row in zip(recs, rows):
        assert list(rec) == CTRL_COLS
        assert rec["t_perf"] == rec["t_wall"] + 1000.0
        assert rec["rhat_f"] == rec["u"]
        for k in keys:
            v = row[k]
            if k in TRI_STATE:
                v = -1 if v == "" else int(v)
            elif k in ("on_a", "have_est", "forced"):
                v = bool(v)
            assert _same(rec[k], v), (rec["n_upd"], k, rec[k], row[k])
    inside = [r for r in recs if r["win"] == "inside"]
    assert inside and {r["state"] for r in inside} >= {"bore1"}
    assert all(r["valid_elp"] in (-1, 0, 1) and r["hold_bed"] in (-1, 0, 1) for r in recs)
    assert all(math.isnan(r["du_blind"]) for r in recs)          # E1: blind_tau 0, the leg-start fix never acts
    first = recs[0]
    assert (first["win"], first["src"], first["ov_pub"], first["state"]) == ("before", "outside", 40, "")
