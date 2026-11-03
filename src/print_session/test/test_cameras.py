"""The bringup's cameras parameter file and switches (print_session.cameras), without ROS."""
from pathlib import Path

import pytest

from e1_paths import REPO
from print_session.cameras import plan_cameras, switch
from print_session.startup import Refused, load_params

CAMERAS = REPO / "src" / "camera_capture" / "config"


def test_switch_is_strict():
    assert switch("readout", "true") and switch("readout", "True") and switch("readout", "1")
    assert not switch("readout", "false") and not switch("readout", "off")
    for bad in ("", "maybe", "/elp/image_raw"):
        with pytest.raises(Refused, match="true or false"):
            switch("readout", bad)


def test_example_camera_files_plan(tmp_path):
    e1 = load_params(CAMERAS / "cameras_e1.yaml")
    assert list(e1) == ["elp", "bore1"]
    cams = plan_cameras(e1, "S", tmp_path, "/elp/image_raw", readout=True, usb_cam=False)
    assert [(c.name, c.image_topic, c.preview_topic) for c in cams] == [
        ("elp", "/elp/image_raw", "/elp/preview"), ("bore1", "/bore1/image_raw", "/bore1/preview")]
    assert cams[0].overrides == dict(session="S", video_dir=str(tmp_path / "raw_video"),
                                     session_meta=str(tmp_path / "session_meta.json"))
    cage = load_params(CAMERAS / "cameras_cage.yaml")
    assert len(plan_cameras(cage, "S", tmp_path, "/elp/image_raw", True, False)) == 4


def test_example_files_keep_the_legacy_profiles():
    e1 = load_params(CAMERAS / "cameras_e1.yaml")
    elp, bore = e1["elp"], e1["bore1"]
    assert (elp["width"], elp["height"], elp["fps"], elp["fourcc"], elp["codec"]) == (1920, 1200, 60.0, "MJPG", "hevc")
    assert elp["controls"] == ["auto_exposure=1", "exposure_time_absolute=78", "gain=0", "white_balance_automatic=0",
                               "white_balance_temperature=4600"]
    assert (elp["segment_s"], elp["cq"], elp["fps_tag"], elp["file_prefix"], elp["csv_prefix"]) == \
        (60.0, 18, 78.4, "elp", "elp_frames")
    assert (bore["width"], bore["height"], bore["fps"], bore["codec"], bore["segment_s"]) == (1280, 720, 30.0, "mp4v",
                                                                                              900.0)
    assert (bore["kind"], bore["file_prefix"], bore["csv_prefix"]) == ("borescope", "bore", "borescope_frames")
    assert "controls" not in bore                       # a borescope never gets an exposure control
    for p in (CAMERAS / "cameras_e1.yaml", CAMERAS / "cameras_cage.yaml"):
        for name, sec in load_params(p).items():
            assert sec["device"].startswith(("/dev/v4l/by-id/", "/dev/v4l/by-path/")), (p.name, name)


def test_refusals(tmp_path):
    e1 = load_params(CAMERAS / "cameras_e1.yaml")
    with pytest.raises(Refused, match="start_camera"):
        plan_cameras(e1, "S", tmp_path, "/elp/image_raw", True, usb_cam=True)
    with pytest.raises(Refused, match="no camera in the camera file publishes it"):
        plan_cameras(e1, "S", tmp_path, "/other/image", True, False)
    assert len(plan_cameras(e1, "S", tmp_path, "", readout=False, usb_cam=False)) == 2
    with pytest.raises(Refused, match="the bringup sets them"):
        plan_cameras({"elp": dict(session="x")}, "S", tmp_path, "/elp/image_raw", True, False)
    with pytest.raises(Refused, match="both publish"):
        plan_cameras({"a": dict(image_topic="/x"), "b": dict(image_topic="/x")}, "S", tmp_path, "/x", True, False)
    with pytest.raises(Refused, match="same files"):
        plan_cameras({"elp": {}, "elp_b": dict(image_topic="/b")}, "S", tmp_path, "/elp/image_raw", True, False)
    assert plan_cameras({}, "S", Path("x"), "/elp/image_raw", True, True) == []
