"""Every run_cl.py argument is a node parameter with the same (effective) default, or is owned by another node; and
config/e1.yaml rebuilds exactly the configuration session_meta.json says E1 ran with.

run_cl.py is both the single-layer one (legacy/realtime_cl_261023) and the multi-layer one
(legacy/realtime_cl_261023_ml), which adds --control-sources and --sat-stop-s (T12), and the cage-2 one
(legacy/realtime_cl_cage2_semi_261103, T15), which adds the ELP-B / bore2 flags."""
import dataclasses
import json

import pytest

import e1_data as E
import legacy_import
from cl_controller.controller import CtrlCfg, Refused
from cl_controller.law import LawCfg
from cl_controller.params import NOT_HERE, PARAMS, build_configs, defaults
from cl_controller.pattern import load_pattern

run_cl = legacy_import.run_cl()
legacy_ctrl = legacy_import.rt_controller()
legacy_law = legacy_import.rt_law_v3()

# run_cl default None -> the value it ends up with (CtrlCfg / LawCfg field, or the node's sentinel for "not given")
EFFECTIVE_NONE = {
    "w12_t": 0.0, "w_target": legacy_ctrl.CtrlCfg.w12_t, "alpha_m": legacy_ctrl.CtrlCfg.alpha_m,
    "ov_outside": legacy_ctrl.CtrlCfg.ov_outside, "ov_min": legacy_ctrl.CtrlCfg.ov_min,
    "ov_max": legacy_ctrl.CtrlCfg.ov_max, "ov_base": 0.0, "v_ref_mm_s": legacy_law.LawCfg.v_ref_mm_s,
    "v_start_mm_s": legacy_law.LawCfg.v_start_mm_s, "open_loop_schedule_mm_s": "",
    "v_min_mm_s": legacy_ctrl.CtrlCfg.v_min_mm_s, "fallback_ramp_s": legacy_ctrl.CtrlCfg.fallback_ramp_s,
    "limit_probe": -1, "plan": "", "primary": "", "b1_win_mm": legacy_ctrl.CtrlCfg.b1_win_mm,
    "gain": legacy_law.LawCfg.gain_elp, "db": legacy_law.LawCfg.db, "ack": legacy_law.LawCfg.ack,
    "u0": legacy_law.LawCfg.u0, "tau_slow": legacy_law.LawCfg.tau_slow, "tau_model": legacy_law.LawCfg.tau_model,
    "start_mix": legacy_law.LawCfg.start_mix, "blind_tau": legacy_law.LawCfg.blind_tau,
    "blind_ref": legacy_law.LawCfg.blind_ref, "blind_dir": legacy_law.LawCfg.blind_dir,
    "guard_bed": legacy_law.LawCfg.guard_bed, "target_schedule": legacy_law.LawCfg.tgt_schedule,
    "speed_check": legacy_ctrl.CtrlCfg.speed_check, "root": "", "mode": "shadow", "pattern": "",
    # legacy _ml run_cl.py: None = both sources / the derived wall stop rule
    "control_sources": "", "sat_stop_s": -1.0,
}
ML_ONLY = {"control_sources", "sat_stop_s"}
# cage-2 run_cl.py: the flags of the controller; --elp-b None = "required" there (print_session cage2 requires it)
CAGE2_ONLY = {"elp_b": "off", "b_from_leg": 1, "bore2": "off", "bore_sd_max": float("inf"), "bore_s_use": 0.0,
              "bore1_offset_mm": 0.0, "bore2_offset_mm": 0.0}


# CtrlCfg fields added by the cage-2 controller (E1 ran without them: their defaults = the E1 behaviour)
CAGE2_CFG = ("elp_b", "b_from_leg", "dir_win_s", "dir_vmin", "bore2", "bore_s_use")


def _legacy_args(ml=False, cage2=False):
    argv = ["--mode", "shadow", "--pattern", "x.json"]
    if cage2:
        with legacy_import.cage2() as L:
            ns = L["run_cl"].parse_args(argv)
    elif ml:
        with legacy_import.ml() as L:
            ns = L["run_cl"].parse_args(argv)
    else:
        ns = run_cl.parse_args(argv)
    return {k: v for k, v in vars(ns).items() if k not in ("mode", "pattern")}, ns


def test_every_run_cl_argument_is_covered():
    args, _ = _legacy_args()
    args_ml, _ = _legacy_args(ml=True)
    args_c2, _ = _legacy_args(cage2=True)
    assert set(args_ml) - set(args) == ML_ONLY
    assert set(CAGE2_ONLY) <= set(args_c2) - set(args)
    names = set(args) | set(args_ml) | set(args_c2) | {"mode", "pattern"}
    missing = names - set(PARAMS) - set(NOT_HERE)
    assert not missing, missing
    assert not (set(PARAMS) & set(NOT_HERE))
    assert set(PARAMS) <= names


@pytest.mark.parametrize("ml", [False, True], ids=["single_layer", "ml"])
def test_defaults_match_run_cl(ml):
    args, _ = _legacy_args(ml)
    for name, (default, _) in PARAMS.items():
        if name in ("mode", "pattern") or (name in ML_ONLY and not ml) or name in CAGE2_ONLY:
            continue
        legacy = args[name]
        want = EFFECTIVE_NONE[name] if legacy is None else legacy
        assert default == want and type(default) is type(want), (name, default, want)


def test_cage2_defaults_match_run_cl():
    args, _ = _legacy_args(cage2=True)
    for name, want in CAGE2_ONLY.items():
        default = PARAMS[name][0]
        assert default == want and type(default) is type(want), name
        legacy = args[name]
        if name == "elp_b":
            assert legacy is None                   # required in run_cl.py (print_session cage2 refuses it missing)
        elif name == "b_from_leg":
            assert legacy is None                   # None = the CtrlCfg value
            with legacy_import.cage2() as L:
                assert L["rt_controller"].CtrlCfg.b_from_leg == want
        else:
            assert legacy == want, (name, legacy)


def test_default_params_give_legacy_cfgs():
    """With only mode / plan / pattern given, the configs equal the legacy run_cl.main result for the same flags."""
    pat = load_pattern(E.PATTERN)
    cfg, lc, _ = build_configs({**defaults(), "mode": "closed", "plan": "A"}, pat)
    _, ns = _legacy_args()
    ns.mode, ns.plan = "closed", "A"
    llc = run_cl.build_law_cfg(ns)
    _, ns_ml = _legacy_args(ml=True)
    ns_ml.mode, ns_ml.plan = "closed", "A"
    with legacy_import.ml() as L:
        llc_ml = L["run_cl"].build_law_cfg(ns_ml)
    for f in dataclasses.fields(LawCfg):
        if f.name in ("w_base", "alpha_m", "v_ref_mm_s"):
            continue
        assert getattr(lc, f.name) == getattr(llc_ml, f.name), f.name
        if f.name != "sources":                     # the single-layer LawCfg has no sources field
            assert getattr(lc, f.name) == getattr(llc, f.name), f.name


def test_e1_yaml_reproduces_session_meta():
    meta = json.loads((E.DATA / "raw_video" / "session_meta.json").read_text(encoding="utf-8"))["realtime"]
    p = E.e1_params()
    assert p["mode"] == meta["mode"] == "closed"
    assert p["pattern"] == meta["pattern"].replace("\\", "/").split("/")[-1]
    cfg, lc, _ = build_configs(p, load_pattern(E.PATTERN))
    for f in dataclasses.fields(CtrlCfg):
        if f.name not in meta["cfg"]:      # E1 ran the single-layer pipeline: no sat_s / sat_min_dz (sat_s 0 = off)
            assert getattr(cfg, f.name) == f.default and f.name in ("sat_s", "sat_min_dz", *CAGE2_CFG), f.name
            continue
        want = meta["cfg"][f.name]
        got = getattr(cfg, f.name)
        assert (list(got) if isinstance(got, tuple) else got) == want, f.name
    for f in dataclasses.fields(LawCfg):
        if f.name not in meta["law"]:      # E1 ran realtime_cl_261022: no blind_* fields (blind_tau 0 = off)
            assert getattr(lc, f.name) == getattr(LawCfg, f.name, f.default), f.name
            continue
        got, want = getattr(lc, f.name), meta["law"][f.name]
        assert (list(got) if isinstance(got, tuple) else got) == want, f.name
    ev = json.loads((E.DATA / "roboter" / "eval_v3_20261022_165128.json").read_text(encoding="utf-8"))
    assert ev["plan"] == p["plan"] and ev["order"] == ["bore1", "elp"] and ev["W_star"] == p["w_target"]


@pytest.mark.parametrize("bad", [dict(plan=""), dict(tau_slow=0.0), dict(gain=2.0), dict(start_mix=1.5),
                                 dict(v_min_mm_s=-1.0), dict(force_seg="3:bore1"), dict(force_seg="x", plan="B"),
                                 dict(open_loop_schedule_mm_s="12,16", plan="A"), dict(ov_base=55.0),
                                 dict(ov_min=50, ov_max=40), dict(mode="fast")])
def test_refused_like_run_cl(bad):
    p = {**defaults(), "mode": "closed", "plan": "A", **bad}
    with pytest.raises(Refused):
        build_configs(p, load_pattern(E.PATTERN))
