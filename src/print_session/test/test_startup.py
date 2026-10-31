"""Start-up plan: the session_meta "realtime" block (golden: E1's session_meta.json and the legacy run_cl.speed_rules),
the parameters derived for the bridge and the readout, and the refusals of run_cl.main that no node does."""
import argparse
import json
import logging
import sys
from dataclasses import asdict

import pytest

from e1_paths import DATA, E1_YAML, LEGACY, LEGACY_PATTERN

from print_session.startup import Refused, load_params, plan

CL_PATTERNS = E1_YAML.parents[1] / "src" / "cl_controller" / "patterns"


def _params(**over):
    p = load_params(E1_YAML)
    for key, v in over.items():
        node, _, name = key.partition("__")
        p.setdefault(node, {})[name] = v
    return p


def _plan(mode="closed", image_topic="/elp/image_raw", **over):
    return plan(_params(**over), mode, image_topic, search=[CL_PATTERNS])


def test_e1_realtime_block_equals_session_meta():
    meta = json.loads((DATA / "raw_video" / "session_meta.json").read_text(encoding="utf-8"))["realtime"]
    p = _plan(print_session__bore1_readout=True)
    rt = json.loads(json.dumps(p.realtime))           # as written to the file
    assert set(rt) == set(meta)                         # mode, cfg, rig, pattern, version, law, order, speed
    assert rt["mode"] == meta["mode"] == "closed"
    assert rt["cfg"] == meta["cfg"]
    assert rt["order"] == meta["order"] == ["bore1", "elp"]
    assert rt["speed"] == meta["speed"]
    extra = set(rt["law"]) - set(meta["law"])           # E1 ran realtime_cl_261022: no leg-start fix fields
    assert extra and all(k.startswith("blind_") for k in extra)
    assert {k: rt["law"][k] for k in meta["law"]} == meta["law"]
    assert rt["law"]["blind_tau"] == 0.0
    assert rt["pattern"].endswith(meta["pattern"].replace("\\", "/").split("/")[-1])
    assert rt["rig"].endswith("rig_cage1012.json")


def _legacy_speed(mode, argv):
    """legacy run_cl.speed_rules on the same flags (the port's CtrlCfg has the legacy fields and defaults)."""
    if str(LEGACY) not in sys.path:
        sys.path.insert(0, str(LEGACY))
    import run_cl
    from rt_law_v3 import pattern_speed_info

    from cl_controller.controller import CtrlCfg
    args = run_cl.parse_args(["--mode", mode, "--pattern", str(LEGACY_PATTERN), *argv])
    cfg = CtrlCfg(window_ignore_z=args.window_ignore_z)
    for k in ("ov_min", "ov_max", "v_min_mm_s", "fallback_ramp_s", "v_ref_mm_s"):      # run_cl.main:595-604
        if getattr(args, k) is not None:
            setattr(cfg, k, getattr(args, k))
    law_cfg = run_cl.build_law_cfg(args)
    law_cfg.w_base, law_cfg.alpha_m, law_cfg.v_ref_mm_s = cfg.w12_t, cfg.alpha_m, cfg.v_ref_mm_s
    if mode == "step":
        cfg.v_min_mm_s = 0.0
    info = pattern_speed_info(str(LEGACY_PATTERN))
    info["pattern"] = LEGACY_PATTERN.name
    refusal, sp, _ = run_cl.speed_rules(cfg, law_cfg, info, args, logging.getLogger("legacy"))
    assert not refusal
    return sp, cfg


@pytest.mark.skipif(not (LEGACY / "run_cl.py").exists(), reason="legacy code not present")
@pytest.mark.parametrize("mode,plan_,over", [
    ("closed", "A", {}), ("shadow", "A", dict(cl_controller__v_min_mm_s=6.0)),
    ("closed", "Bpp", dict(cl_controller__open_loop_schedule_mm_s="12,16,20")),
    ("closed", "A", dict(cl_controller__target_schedule="midleg")),
    ("step", "A", {}),
])
def test_speed_block_equals_legacy_speed_rules(mode, plan_, over):
    flags = ["--plan", plan_]
    for k, v in over.items():
        flags += ["--" + k.split("__")[1].replace("_", "-"), str(v)]
    want, lcfg = _legacy_speed(mode, flags)
    p = _plan(mode, cl_controller__plan=plan_, **over)
    assert p.realtime["speed"] == json.loads(json.dumps(want))
    for k in ("ov_base", "ov_default", "ov_min", "ov_floor"):
        assert p.realtime["cfg"][k] == getattr(lcfg, k), k


def test_bridge_gets_the_controller_limits():
    p = _plan("shadow", cl_controller__plan="A")
    b = p.bridge
    assert (b["mode"], b["ov_default"], b["ov_min"], b["ov_max"], b["v_min_mm_s"]) == ("shadow", 40, 1, 100, 3.4)
    assert b["pattern_file"] == str(p.pattern) == p.readout["pattern_json"] == p.controller["pattern"]
    from krc_rsi_bridge.decider import DeciderConfig, effective_limits
    from krc_rsi_bridge.window import PatternGeom
    dc = DeciderConfig(**{k: (tuple(v) if k == "step_seq" else v) for k, v in b.items()
                          if k in DeciderConfig.__dataclass_fields__})
    assert effective_limits(dc, PatternGeom.load(p.pattern)) == (12, 100, 12)
    s = _plan("step", cl_controller__plan="A")
    assert s.bridge["v_min_mm_s"] == 0.0 and s.realtime["cfg"]["ov_floor"] == 0


@pytest.mark.parametrize("mode,image_topic,over,match", [
    ("fast", "/elp/image_raw", dict(cl_controller__plan="A"), "mode"),
    ("closed", "/elp/image_raw", {}, "bore1 readout"),                         # E1 as is: plan A_b1, no bore1
    ("shadow", "", dict(cl_controller__plan="A"), "ELP"),
    ("closed", "/elp/image_raw", dict(cl_controller__plan="A", cl_controller__limit_probe=5), "limit_probe"),
    ("closed", "/elp/image_raw", dict(cl_controller__plan="A", krc_rsi_bridge__ov_default=50), "ov_default"),
    ("closed", "/elp/image_raw", dict(cl_controller__plan="A", krc_rsi_bridge__mode="closed"), None),   # same: ok
    ("closed", "/elp/image_raw", dict(cl_controller__plan="A", width_readout_elp__pattern_json=str(LEGACY / "pattern"
                                      / "pattern_usr_ELP_R1v12_CL_261021.json")), "pattern"),
    ("closed", "/elp/image_raw", dict(cl_controller__plan=""), "plan"),          # cl_controller's own refusal
    ("closed", "/elp/image_raw", dict(cl_controller__plan="A", cl_controller__ov_min=50,
                                      cl_controller__ov_max=40), "window"),
])
def test_refusals(mode, image_topic, over, match):
    if match is None:
        assert _plan(mode, image_topic, **over)
        return
    with pytest.raises(Refused, match=match):
        _plan(mode, image_topic, **over)


def test_step_without_image_topic_runs_without_readout():
    p = _plan("step", "", cl_controller__plan="A")
    assert p.readout is None


def test_no_bore1_warning_and_e1_with_bore1():
    p = _plan("closed", cl_controller__plan="A")
    assert any("ELP -> model only" for line in p.log)
    q = _plan("closed", print_session__bore1_readout=True)
    assert q.realtime["law"]["plan"] == "A_b1" and not any("model only" in line for line in q.log)
    assert asdict(q)["session"]["bore1_readout"] is True
