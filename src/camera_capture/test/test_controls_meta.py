"""V4L2 controls (set in order, every one read back), session_meta.json updates, and the verify tool."""
import json
import multiprocessing as mp
import shutil

import pytest

from camera_capture.camera import CameraConfig
from camera_capture.controls import (V4l2Ctl, apply_controls, check_allowed, exposure_units_100us, parse_controls,
                                     parse_get_ctrl, parse_list_ctrls)
from camera_capture.session_meta import read_meta, update_meta, write_meta
from camera_capture.verify import compare_mean, load_frame, main as verify_main
from helpers import ListLog
from repo_paths import E1_FRAMES, E1_META

ELP_CONTROLS = ["auto_exposure=1", "exposure_time_absolute=78", "gain=0", "white_balance_automatic=0",
                "white_balance_temperature=4600"]


def test_exposure_in_v4l2_units():
    # E1: CAP_PROP_EXPOSURE -7 on DirectShow = 2^-7 s = 7.8125 ms -> 78 x 100 us
    meta = json.loads(E1_META.read_text(encoding="utf-8"))
    assert meta["elp"]["controls"]["CAP_PROP_EXPOSURE"] == -7
    assert exposure_units_100us(2.0 ** meta["elp"]["controls"]["CAP_PROP_EXPOSURE"]) == 78


def test_parse_controls_keeps_the_order():
    assert parse_controls(ELP_CONTROLS) == [("auto_exposure", 1), ("exposure_time_absolute", 78), ("gain", 0),
                                            ("white_balance_automatic", 0), ("white_balance_temperature", 4600)]
    for bad in (["gain"], ["Gain=1"], ["gain=x"], ["gain=1", "gain=2"]):
        with pytest.raises(ValueError):
            parse_controls(bad)
    assert parse_controls(["", " "]) == []


def test_borescope_never_gets_exposure():
    check_allowed("borescope", [("brightness", 0)])
    with pytest.raises(ValueError, match="never set exposure"):
        check_allowed("borescope", [("exposure_time_absolute", 78)])
    with pytest.raises(ValueError):
        CameraConfig(name="bore1", kind="borescope", controls=["auto_exposure=1"], record=False)
    check_allowed("elp", parse_controls(ELP_CONTROLS))


class FakeCtl:
    """A camera's controls: white_balance_temperature only takes multiples of 10, and one control is missing."""

    def __init__(self):
        self.v = dict(auto_exposure=3, exposure_time_absolute=156, gain=0, white_balance_automatic=1,
                      white_balance_temperature=4000)
        self.calls = []

    def set(self, dev, name, value):
        self.calls.append((name, value))
        if name not in self.v:
            return f"unknown control '{name}'"
        if name == "exposure_time_absolute" and self.v["auto_exposure"] != 1:
            return "VIDIOC_S_EXT_CTRLS: failed: Permission denied"      # inactive while auto exposure is on
        self.v[name] = value - value % 10 if name == "white_balance_temperature" else value
        return None

    def get(self, dev, name):
        return self.v.get(name)


def test_apply_controls_sets_in_order_and_reads_every_one_back():
    ctl, log = FakeCtl(), ListLog()
    controls = parse_controls(ELP_CONTROLS[:4] + ["white_balance_temperature=4605", "zoom_absolute=1"])
    r = apply_controls(ctl, "/dev/x", controls, log)
    assert [c[0] for c in ctl.calls] == ["auto_exposure", "exposure_time_absolute", "gain",
                                         "white_balance_automatic", "white_balance_temperature", "zoom_absolute"]
    assert r["readback"] == dict(auto_exposure=1, exposure_time_absolute=78, gain=0, white_balance_automatic=0,
                                 white_balance_temperature=4600, zoom_absolute=None)
    assert r["mismatch"] == ["white_balance_temperature", "zoom_absolute"]
    assert "zoom_absolute" in r["errors"]
    t = log.text()
    assert "controls requested" in t and "controls read back" in t and "reads back 4600, requested 4605" in t


def test_wrong_order_shows_in_the_readback():
    ctl = FakeCtl()
    r = apply_controls(ctl, "/dev/x", [("exposure_time_absolute", 78), ("auto_exposure", 1)], ListLog())
    assert r["readback"]["exposure_time_absolute"] == 156 and r["mismatch"] == ["exposure_time_absolute"]


LIST_CTRLS = """
User Controls

                     brightness 0x00980900 (int)    : min=-64 max=64 step=1 default=0 value=0
                           gain 0x00980913 (int)    : min=0 max=100 step=1 default=0 value=0
      white_balance_temperature 0x0098091a (int)    : min=2800 max=6500 step=1 default=4600 value=4600 flags=inactive

Camera Controls

                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value=1 (Manual Mode)
         exposure_time_absolute 0x009a0902 (int)    : min=1 max=5000 step=1 default=156 value=78
"""


def test_parse_v4l2_ctl_output():
    d = parse_list_ctrls(LIST_CTRLS)
    assert d["exposure_time_absolute"] == dict(type="int", min=1, max=5000, step=1, default=156, value=78)
    assert d["auto_exposure"]["value"] == 1 and d["white_balance_temperature"]["flags"] == "inactive"
    got = parse_get_ctrl("auto_exposure: 1\nexposure_time_absolute: 78\n")
    assert got == dict(auto_exposure=1, exposure_time_absolute=78)


def test_v4l2_ctl_backend_runs_the_command(tmp_path):
    """The subprocess backend against a fake v4l2-ctl (the real one needs a camera)."""
    fake = tmp_path / "v4l2-ctl"
    log = tmp_path / "calls"
    fake.write_text("#!/bin/sh\n"
                    f"echo \"$@\" >> {log}\n"
                    "case \"$3\" in\n"
                    "  --get-ctrl=gain) echo 'gain: 7';;\n"
                    "  --set-ctrl=bad=1) echo \"unknown control 'bad'\" >&2; exit 1;;\n"
                    "  --list-ctrls) echo '  gain 0x00980913 (int)    : min=0 max=100 step=1 default=0 value=7';;\n"
                    "esac\n", encoding="utf-8")
    fake.chmod(0o755)
    c = V4l2Ctl(str(fake))
    assert c.set("/dev/v", "gain", 7) is None
    assert "unknown control" in c.set("/dev/v", "bad", 1)
    assert c.get("/dev/v", "gain") == 7
    assert parse_list_ctrls(c.list_ctrls("/dev/v"))["gain"]["value"] == 7
    assert log.read_text().splitlines()[0] == "-d /dev/v --set-ctrl=gain=7"


@pytest.mark.skipif(not shutil.which("v4l2-ctl"), reason="no v4l2-ctl")
def test_real_v4l2_ctl_without_a_device_reports_errors():
    c = V4l2Ctl()
    assert c.get("/dev/v4l/by-id/none", "gain") is None
    assert c.set("/dev/v4l/by-id/none", "gain", 0) is not None


# ------------------------------------------------------------------------------------------------ session_meta
def _writer(path, name, n):
    for k in range(n):
        update_meta(path, ("cameras", name), dict(k=k))


def test_concurrent_meta_updates_lose_nothing(tmp_path):
    p = tmp_path / "session_meta.json"
    write_meta(p, dict(realtime=dict(mode="closed")))
    ctx = mp.get_context("spawn")
    procs = [ctx.Process(target=_writer, args=(p, f"cam{i}", 40)) for i in range(4)]
    procs.append(ctx.Process(target=_writer, args=(p, "x", 40)))
    for pr in procs:
        pr.start()
    for k in range(40):
        update_meta(p, "v3_summary", dict(k=k))
    for pr in procs:
        pr.join(30)
        assert pr.exitcode == 0
    m = read_meta(p)
    assert m["realtime"] == dict(mode="closed") and m["v3_summary"] == dict(k=39)
    assert m["cameras"] == {f"cam{i}": dict(k=39) for i in range(4)} | {"x": dict(k=39)}
    assert not list(tmp_path.glob("*.tmp*"))


# ------------------------------------------------------------------------------------------------ verify
def test_mean_intensity_check_on_e1_frames(tmp_path):
    import cv2
    a = sorted(E1_FRAMES.glob("*.jpg"))[0]
    img = load_frame(str(a))
    assert compare_mean(img, img)["ok"]
    dark = (img.astype(float) * 0.85).astype("uint8")
    r = compare_mean(dark, img, tol_pct=5.0)
    assert not r["ok"] and r["diff_pct"] == pytest.approx(-15.0, abs=1.0)
    cv2.imwrite(str(tmp_path / "d.png"), dark)
    assert verify_main(["--mean", str(tmp_path / "d.png"), str(a), "--tol-pct", "20"]) == 0
    assert verify_main(["--mean", str(tmp_path / "d.png"), str(a), "--roi", "800,500,300,200"]) == 1


def test_verify_cli_finds_a_mismatch(tmp_path):
    from camera_capture.recorder import RecorderConfig, SegmentRecorder
    from helpers import items
    rec = SegmentRecorder(RecorderConfig(video_dir=str(tmp_path), session="S", width=32, height=24, codec="mjpg"))
    rec.start()
    for it in items(30):
        rec.put(it)
    rec.close()
    assert verify_main([str(tmp_path), "--json", str(tmp_path / "v.json")]) == 0
    with open(rec.cfg.csv_path, "a", newline="", encoding="utf-8") as f:     # one row without a frame
        f.write(f"1.0,2.0,31,1,31,{rec.files[0]['file']},3.0\r\n")
    assert verify_main([str(tmp_path)]) == 1
