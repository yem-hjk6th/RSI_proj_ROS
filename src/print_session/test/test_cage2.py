"""T15 (print_session): the cage-2 bringup plan and its refusals, without ROS.

- config/cage2_bduo.yaml (with replay heads instead of the weights) gives the realtime block of session
  bduo_20261103_195120 (cfg, law, law_b, order, elp_b, control_sources, MDF side, bore_dual), starts the ELP-B and
  bore2 readouts with their rigs bound by role, hands bore_sd_max / the offsets to the bore nodes, and ends the session
  when bore2 (primary) ends.
- One test per refusal (startup.py docstring, run_cl.py line numbers in the names), the geometry refusal (T13 review,
  PR #42), and the camera-file checks of the cage-2 readouts.
"""
import copy
import json
import math
from pathlib import Path, PurePosixPath

import pytest
import yaml

from e1_paths import REPO
from print_session.cameras import plan_cameras
from print_session.startup import BORE1, BORE2, CONTROLLER, READOUT, READOUT_B, SESSION, Refused, plan

CFG = REPO / "src" / "print_session" / "config" / "cage2_bduo.yaml"
CL_PATTERNS = REPO / "src" / "cl_controller" / "patterns"
PATH_PATTERNS = REPO / "src" / "path_geometry" / "patterns"
BDUO = REPO / "data" / "bduo_20261103_195120"
CAMERAS = REPO / "src" / "camera_capture" / "config" / "cameras_cage.yaml"


def _quality(cam):
    from width_readout_bore1.config import cage2_quality_json
    return str(cage2_quality_json(cam))


def _params(**over):
    y = yaml.safe_load(CFG.read_text(encoding="utf-8"))
    p = {k: dict(v["ros__parameters"]) for k, v in y.items()}
    for cam, node in (("bore1", BORE1), ("bore2", BORE2)):
        p[node].update(head="replay", replay_csv=str(BDUO / "raw_video" / f"{cam}_rt_20261103_195120.csv.gz"),
                       weight_dir="", quality_json=_quality(cam))
    for key, v in over.items():
        node, _, name = key.partition("__")
        if v is None:
            p.setdefault(node, {}).pop(name, None)
        else:
            p.setdefault(node, {})[name] = v
    return p


def _plan(mode="closed", image_topic="/elp/image_raw", **over):
    return plan(_params(**over), mode, image_topic, search=[CL_PATTERNS])


def _ends_with(path, tail: str) -> bool:
    """`path` ends with the components of `tail` (written with "/"), on any operating system's separator."""
    parts = PurePosixPath(tail).parts
    return Path(path).parts[-len(parts):] == parts


pytestmark = pytest.mark.skipif(not (BDUO / "raw_video").is_dir(), reason="bduo session data not present")


def test_plan_reproduces_the_session_realtime_block():
    meta = json.loads((BDUO / "raw_video" / "session_meta.json").read_text(encoding="utf-8"))["realtime"]
    p = _plan()
    rt = json.loads(json.dumps(p.realtime))
    assert rt["mode"] == meta["mode"] == "closed"
    assert set(rt["cfg"]) - set(meta["cfg"]) == {"sat_s", "sat_min_dz"} and rt["cfg"]["sat_s"] == 0.0
    assert {k: rt["cfg"][k] for k in meta["cfg"]} == meta["cfg"]
    assert rt["law"] == meta["law"] and rt["law_b"] == meta["law_b"] and rt["order"] == meta["order"]
    assert rt["speed"] == {k: meta["speed"][k] for k in rt["speed"]}
    assert (rt["elp_b"], rt["control_sources"], rt["b_mdf_side"]) == (meta["elp_b"], meta["control_sources"],
                                                                        meta["b_mdf_side"])
    bd = meta["bore_dual"]
    assert {k: rt["bore_dual"][k] for k in ("bore2", "bore2_ctl", "bore_sd_max", "bore_s_use", "bore1_offset_mm",
                                            "bore2_offset_mm")} == {k: bd[k] for k in rt["bore_dual"]}
    assert rt["rig"].endswith(meta["rig"]) and rt["rig_b"].endswith(meta["rig_b"])


def test_plan_starts_the_cage2_nodes():
    p = _plan()
    assert _ends_with(p.readout["rig_json"], "cage2opt_elpA/rig_cage2opt_elpA_261102_m2.json")
    assert p.readout_b == dict(kind="B", camera="elp_b", width_topic="/width/elp_b", pattern_json=str(p.pattern),
                               rig_json=p.readout_b["rig_json"], image_topic="/elp_b/image_raw", b_mdf_side="pos")
    assert _ends_with(p.readout_b["rig_json"], "cage2opt_elpB/rig_cage2opt_elpB_261102.json")
    assert p.bore1 == dict(pattern_json=str(p.pattern), sd_max=0.05, w_offset=0.0) and p.needs_bore1
    assert p.bore2 == dict(pattern_json=str(p.pattern), camera="bore2", width_topic="/width/bore2",
                           record_topic="/width/bore2/record", image_topic="/bore2/image_raw", sd_max=0.05,
                           w_offset=0.0)
    assert p.needs_bore2
    assert any(line.startswith("cage 2 ELP-B ON") for line in p.log)
    assert any(line.startswith("bore2 readout: width_readout_bore2") for line in p.log)


def test_bore2_shadow_starts_the_node_but_the_session_does_not_need_it():
    p = _plan(cl_controller__bore2="shadow")
    assert p.bore2 is not None and not p.needs_bore2 and p.realtime["bore_dual"]["bore2_ctl"] == "off"


def test_elp_b_off_logs_elp_b_only():
    p = _plan(cl_controller__elp_b="off", cl_controller__bore2="off")
    assert p.readout_b is not None and p.bore2 is None and p.realtime["elp_b"] == "off"
    assert any("ELP-B readout logged only" in line for line in p.log)


def test_no_cage2_is_the_e1_plan():
    p = _plan(print_session__cage2=False, cl_controller__elp_b=None, cl_controller__bore2=None)
    assert p.readout_b is None and p.bore2 is None and "elp_b" not in p.realtime
    assert p.readout["rig_json"].endswith("rig_cage1012.json")


def test_refused_cage2_without_elp_b_run_cl_1154():
    with pytest.raises(Refused, match="set cl_controller.elp_b"):
        _plan(cl_controller__elp_b=None, cl_controller__bore2="off")
    _plan("step", "", cl_controller__elp_b=None, cl_controller__bore2="off")         # step: no control


def test_refused_elp_b_on_without_cage2_run_cl_1158():
    with pytest.raises(Refused, match="needs the ELP-B camera"):
        _plan(print_session__cage2=False, cl_controller__bore2="off")


def test_refused_bore2_without_cage2():
    with pytest.raises(Refused, match="needs the bore2 camera"):
        _plan(print_session__cage2=False, cl_controller__elp_b="off", cl_controller__bore2="shadow")


def test_refused_bore2_without_quality_run_cl_1125():
    with pytest.raises(Refused, match="needs quality_json"):
        _plan(width_readout_bore2__quality_json="")


@pytest.mark.parametrize("mode", ["primary", "fallback", "shadow"])
def test_refused_bore2_without_head_run_cl_1365(tmp_path, mode):
    with pytest.raises(Refused, match="weight_dir"):
        _plan(cl_controller__bore2=mode, width_readout_bore2__head="torch", width_readout_bore2__weight_dir="")
    wd = tmp_path / "w"
    wd.mkdir()
    with pytest.raises(Refused, match="missing"):
        _plan(cl_controller__bore2=mode, width_readout_bore2__head="torch", width_readout_bore2__weight_dir=str(wd))


def test_refused_bore1_weights_without_quality_run_cl_1120():
    with pytest.raises(Refused, match="frozen E1 head"):
        _plan(width_readout_bore1__quality_json="")


@pytest.mark.parametrize("node,key,value", [(BORE1, "sd_max", 0.1), (BORE2, "sd_max", math.inf),
                                            (BORE2, "w_offset", 0.1), (BORE1, "w_offset", -0.1)])
def test_refused_bore_section_overriding_the_controller(node, key, value):
    with pytest.raises(Refused, match="follow from cl_controller"):
        _plan(**{f"{node}__{key}": value})
    _plan(**{f"{node}__{key}": 0.05 if key == "sd_max" else 0.0})                     # the same value is fine


def test_refused_offsets_and_flags_of_the_controller():
    with pytest.raises(Refused, match="bore2_offset_mm"):                             # run_cl.py 1127
        _plan(cl_controller__bore2_offset_mm=0.4)
    with pytest.raises(Refused, match="bore_sd_max"):                                 # run_cl.py 1198
        _plan(cl_controller__bore_sd_max=math.inf)


def test_refused_rig_of_the_other_role():
    from width_readout_elp.config import role_rig_json
    with pytest.raises(Refused, match="role"):
        _plan(width_readout_elp__rig_json=str(role_rig_json("elpB")))
    with pytest.raises(Refused, match="role"):
        _plan(width_readout_elp_b__rig_json=str(role_rig_json("elpA")))


def test_refused_same_rig_for_both(tmp_path):
    import shutil
    from width_readout_elp.config import role_rig_json
    d = tmp_path / "rig"
    shutil.copytree(role_rig_json("elpA").parent, d)
    rig = d / "rig_cage2_shared.json"
    shutil.copyfile(d / "rig_cage2opt_elpA_261102_m2.json", rig)
    with pytest.raises(Refused, match="one rig per role"):
        _plan(width_readout_elp__rig_json=str(rig), width_readout_elp_b__rig_json=str(rig))


@pytest.mark.parametrize("key,value", [("kind", "A"), ("width_topic", "/width/elp"), ("camera", "elp")])
def test_refused_elp_b_section_values(key, value):
    with pytest.raises(Refused, match="ELP-B readout is"):
        _plan(**{f"{READOUT_B}__{key}": value})


@pytest.mark.parametrize("name", ["pattern_usr_ELP_FAN1_261103.json", "pattern_usr_ELP_ARC2_261103.json"])
@pytest.mark.parametrize("cage2", [True, False])
def test_refused_geometry_pattern(name, cage2):
    """T13 review (PR #42): no bringup with a fan / semicircle pattern until their controller exists (T16)."""
    over = dict(print_session__cage2=cage2, cl_controller__pattern=str(PATH_PATTERNS / name))
    if not cage2:
        over.update(cl_controller__elp_b=None, cl_controller__bore2=None)
    with pytest.raises(Refused, match="geometry"):
        _plan(**over)


def test_cameras_of_the_cage2_readouts(tmp_path):
    p = _plan()
    cams = yaml.safe_load(CAMERAS.read_text(encoding="utf-8"))
    cams = {k: dict(v["ros__parameters"]) for k, v in cams.items()}
    extra = {READOUT_B: p.readout_b["image_topic"], BORE2: p.bore2["image_topic"]}
    with pytest.raises(Refused, match="publish_raw false"):              # the cage file keeps elp_b / bore2 raw off
        plan_cameras(cams, "S", tmp_path, "/elp/image_raw", True, False, "/bore1/image_raw", extra)
    ok = copy.deepcopy(cams)
    ok["elp_b"]["publish_raw"] = ok["bore2"]["publish_raw"] = True
    assert len(plan_cameras(ok, "S", tmp_path, "/elp/image_raw", True, False, "/bore1/image_raw", extra)) == 4
    del ok["bore2"]
    with pytest.raises(Refused, match="width_readout_bore2 reads /bore2/image_raw"):
        plan_cameras(ok, "S", tmp_path, "/elp/image_raw", True, False, "/bore1/image_raw", extra)


def test_section_names():
    assert (READOUT, READOUT_B, BORE1, BORE2, CONTROLLER, SESSION) == (
        "width_readout_elp", "width_readout_elp_b", "width_readout_bore1", "width_readout_bore2", "cl_controller",
        "print_session")
