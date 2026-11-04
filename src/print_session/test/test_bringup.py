"""Acceptance tests 3 and 4: the bringup launch.

3. Shadow mode against krc_sim on loopback for about 10 s: the replies carry no OvCmd, a bag is recorded,
   session_meta.json is written, and bag_to_legacy runs on the bag.
   It also logs the shadow-mode warning (T9: no OvCmd in the reply, 1 % override on RSI_OV4).
4. Without mode the launch refuses to start (and with a mode outside shadow | closed | step, and when a start-up
   check refuses); no session is created then.
"""
import errno
import json
import os
import signal
import socket
import subprocess
import time

import pytest
import yaml

from e1_paths import E1_YAML, LAUNCH

pytest.importorskip("rclpy")
pytest.importorskip("krc_interfaces.msg")
pytest.importorskip("launch_ros")


def _free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _bound(port: int) -> bool:
    """True once someone has bound the UDP port (our bind without SO_REUSEADDR fails then)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.bind(("127.0.0.1", port))
            return False
        except OSError as e:
            return e.errno == errno.EADDRINUSE


def _params(tmp_path, port, **cl):
    y = yaml.safe_load(E1_YAML.read_text(encoding="utf-8"))
    y["cl_controller"]["ros__parameters"].update(cl)
    y["krc_rsi_bridge"]["ros__parameters"].update(host="127.0.0.1", port=port)
    y["print_session"]["ros__parameters"]["krc_rsi_config"] = "RSI_MIN"   # krc_sim: shadow mode is allowed
    p = tmp_path / "params.yaml"
    p.write_text(yaml.safe_dump(y), encoding="utf-8")
    return p


def _launch(*args, **kw):
    return subprocess.Popen(["ros2", "launch", str(LAUNCH), *args], stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, start_new_session=True, **kw)


def _finish(proc, timeout):
    try:
        out, _ = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            out, _ = proc.communicate(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            out, _ = proc.communicate()
        pytest.fail("launch did not end by itself:\n" + out[-4000:])
    return out


def test_missing_mode_refuses(tmp_path):
    proc = _launch(f"params_file:={E1_YAML}", f"out_dir:={tmp_path}", "session:=x")
    out = _finish(proc, 60)
    assert proc.returncode != 0
    assert "mode" in out and ("required" in out.lower() or "missing" in out.lower()), out[-2000:]
    assert not (tmp_path / "x").exists()


@pytest.mark.parametrize("args,needle", [
    (["mode:=fast"], "mode"),
    (["mode:=closed"], "bore1"),                       # config/e1.yaml as it is: plan A_b1, no bore1 readout
    (["mode:=shadow"], "RSI_MIN"),                     # config/e1.yaml declares RSI_OV4: shadow is refused
])
def test_bad_mode_or_refused_start(tmp_path, args, needle):
    proc = _launch(*args, f"params_file:={E1_YAML}", f"out_dir:={tmp_path}", "session:=x")
    out = _finish(proc, 60)
    assert proc.returncode != 0 and needle in out, out[-2000:]
    assert not (tmp_path / "x" / "session_meta.json").exists()


def _missing_ipocs(msgs, T, out):
    """Failure message of the state / reply count check: which packets each topic lacks (start, end or a gap in the
    middle tells recorder discovery from a dropped burst) and the bridge's largest publish batch."""
    st = {int(m.ipoc) for m in msgs.get(T.STATE, [])}
    rp = {int(m.ipoc) for m in msgs.get(T.REPLY, [])}
    allp = sorted(st | rp)
    pos = {ip: k for k, ip in enumerate(allp)}

    def where(missing):
        k = sorted(pos[ip] for ip in missing)
        return f"{len(k)} (positions {k[:5]}...{k[-5:]} of {len(allp)})" if k else "0"
    batch = [line for line in out.splitlines() if "largest batch" in line]
    return f"states missing {where(rp - st)}, replies missing {where(st - rp)}; {batch[-1:] or 'no bridge summary'}"


def test_shadow_bringup_against_krc_sim(tmp_path):
    from krc_sim.sim import SimConfig, SimProcess

    from print_session import topics as T
    from print_session.bag_io import read_bag
    from print_session.bag_to_legacy import main as export_main
    from print_session.startup import pattern_dirs

    port = _free_udp_port()
    params = _params(tmp_path, port, plan="A")
    out_dir = tmp_path / "sessions"
    proc = _launch("mode:=shadow", f"params_file:={params}", f"out_dir:={out_dir}", "session:=smoke",
                   "image_topic:=/smoke/image")
    sdir = out_dir / "smoke"
    try:
        t_end = time.monotonic() + 60
        while time.monotonic() < t_end and not ((sdir / "session_meta.json").exists() and _bound(port)
                                                 and (sdir / "bag").exists()):
            assert proc.poll() is None, proc.stdout.read()[-4000:]
            time.sleep(0.2)
        assert _bound(port), "bridge did not bind"
        time.sleep(3.0)                                # recorder discovery
        pat = next(d / "pattern_usr_ELP_R1v30_CLp_261022.json" for d in pattern_dirs()
                   if (d / "pattern_usr_ELP_R1v30_CLp_261022.json").exists())
        st = SimProcess(SimConfig(port=port, pattern_file=str(pat), duration_s=10.0)).start().result(60)
    except BaseException:
        os.killpg(proc.pid, signal.SIGINT)
        print(proc.communicate(timeout=30)[0][-6000:])
        raise
    out = _finish(proc, 60)                            # the session ends by itself once RSI is gone
    print(out[-3000:])
    assert proc.returncode == 0, out[-4000:]

    # T9: the shadow-mode warning, from the launch and as a WARN line of the session node
    from print_session.startup import SHADOW_WARNING
    assert out.count(SHADOW_WARNING) >= 2, out[:4000]
    assert any("WARN" in line and SHADOW_WARNING in line for line in out.splitlines()), out[:4000]

    # the replies carry no OvCmd (shadow), and the bridge kept up
    assert st.sent >= 2400 and not st.timeout_stopped
    assert st.ov_seen == [] and st.nonzero_rkorr == 0

    # session_meta.json
    meta = json.loads((sdir / "session_meta.json").read_text(encoding="utf-8"))
    assert meta["realtime"]["mode"] == "shadow" and meta["realtime"]["law"]["plan"] == "A"
    assert meta["realtime"]["speed"]["ov_default"] == 40 and meta["realtime"]["cfg"]["ov_min"] == 12
    assert meta["print_session"]["end_reason"] == "RSI disconnected"
    assert (meta["src_name"], meta["ceiling_light"]) == ("usr_ELP_R1v30_CLp_261022.src", "on")
    assert meta["v3_summary"]["n_updates"] > 10

    # a bag was recorded
    bag = sdir / "bag"
    assert (bag / "metadata.yaml").exists()
    msgs = read_bag(bag)
    n_state = len(msgs.get(T.STATE, []))
    assert n_state >= 0.9 * st.sent, (n_state, st.sent)
    assert abs(len(msgs.get(T.REPLY, [])) - n_state) <= 10, _missing_ipocs(msgs, T, out)
    assert all(r.ov_sent is False for r in msgs[T.REPLY])
    assert len(msgs.get(T.CTRL, [])) > 10 and len(msgs.get(T.OVERRIDE, [])) > 10

    # the exporter runs on the bag
    exp = tmp_path / "export"
    assert export_main([str(bag), "--out", str(exp)]) == 0
    rd = exp / "roboter" / "smoke"
    for name in ("rsi_data", "ov_cmd", "ctrl", "readout", "readout_rows"):
        assert (rd / f"{name}_smoke.csv").exists(), name
    lines = (rd / "rsi_data_smoke.csv").read_text(encoding="utf-8").splitlines()
    assert len(lines) - 1 == n_state
    assert (exp / "raw_video" / "smoke" / "session_meta.json").exists()


# ---------------------------------------------------------------------------------------- T10: readout, cameras
@pytest.mark.parametrize("args,needle", [
    (["mode:=closed", "readout:=false"], "need the ELP readout"),       # no readout: step mode only
    (["mode:=step", "readout:=maybe"], "readout"),                      # true or false only
    (["mode:=step", "start_camera:=true", "cameras:=CAMS"], "start_camera"),   # one camera driver, not both
])
def test_readout_and_camera_switch_refusals(tmp_path, args, needle):
    cams = tmp_path / "cams.yaml"
    cams.write_text(yaml.safe_dump(dict(elp=dict(ros__parameters=dict(source="synthetic")))), encoding="utf-8")
    args = [a.replace("CAMS", str(cams)) for a in args]
    proc = _launch(*args, f"params_file:={E1_YAML}", f"out_dir:={tmp_path}", "session:=x")
    out = _finish(proc, 60)
    assert proc.returncode != 0 and needle in out, out[-2000:]
    assert not (tmp_path / "x" / "session_meta.json").exists()


def _synthetic_cameras(tmp_path):
    """Two camera_capture nodes on synthetic sources (no camera in CI), recording from the first RSI packet."""
    common = dict(source="synthetic", wait_for_rsi=True, preview_width=160)
    cams = dict(
        elp=dict(ros__parameters=dict(common, kind="generic", width=320, height=200, fps=30.0, codec="mjpg",
                                      segment_s=2.0)),
        bore1=dict(ros__parameters=dict(common, kind="borescope", width=160, height=120, fps=15.0, codec="mp4v",
                                        file_prefix="bore", csv_prefix="borescope_frames", segment_s=900.0)))
    p = tmp_path / "cameras.yaml"
    p.write_text(yaml.safe_dump(cams, sort_keys=False), encoding="utf-8")
    return p


def test_step_bringup_without_readout_with_cameras(tmp_path):
    """readout:=false (the bringup fix of T10: an empty image_topic:= is rejected by ros2 launch) in step mode, with
    a cameras file: no readout node, the cameras record to <session>/raw_video from the first RSI packet, CSV rows 1:1
    with the frames in the files, and every camera's block in session_meta.json."""
    from camera_capture.verify import check_dir, read_frames_csv
    from krc_sim.sim import SimConfig, SimProcess

    from print_session.startup import pattern_dirs

    port = _free_udp_port()
    params = _params(tmp_path, port, plan="A")
    out_dir = tmp_path / "sessions"
    proc = _launch("mode:=step", "readout:=false", f"params_file:={params}", f"out_dir:={out_dir}", "session:=cams",
                   f"cameras:={_synthetic_cameras(tmp_path)}")
    sdir = out_dir / "cams"
    try:
        t_end = time.monotonic() + 60
        while time.monotonic() < t_end and not ((sdir / "session_meta.json").exists() and _bound(port)):
            assert proc.poll() is None, proc.stdout.read()[-4000:]
            time.sleep(0.2)
        assert _bound(port), "bridge did not bind"
        time.sleep(3.0)                                # discovery; the cameras capture but do not record yet
        pat = next(d / "pattern_usr_ELP_R1v30_CLp_261022.json" for d in pattern_dirs()
                   if (d / "pattern_usr_ELP_R1v30_CLp_261022.json").exists())
        t_rsi = time.time()
        st = SimProcess(SimConfig(port=port, pattern_file=str(pat), duration_s=5.0)).start().result(60)
    except BaseException:
        os.killpg(proc.pid, signal.SIGINT)
        print(proc.communicate(timeout=60)[0][-6000:])
        raise
    out = _finish(proc, 90)
    print(out[-3000:])
    assert proc.returncode == 0, out[-4000:]
    assert st.sent >= 1000 and not st.timeout_stopped
    assert "[width_readout_elp" not in out                            # no readout node

    meta = json.loads((sdir / "session_meta.json").read_text(encoding="utf-8"))
    ps = meta["print_session"]
    assert ps["readout"] is False and ps["image_topic"] == "" and ps["cameras"] == ["elp", "bore1"]
    assert ps["end_reason"] == "RSI disconnected" and meta["realtime"]["mode"] == "step"
    raw = sdir / "raw_video"
    res = {r["csv"]: r for r in check_dir(raw)}
    assert set(res) == {"elp_frames_cams.csv", "borescope_frames_cams.csv"}
    for name, csv_name in (("elp", "elp_frames_cams.csv"), ("bore1", "borescope_frames_cams.csv")):
        r = res[csv_name]
        assert r["ok"] and r["rows"] > 20, r
        b = meta["cameras"][name]
        assert b["final"] and b["measured"]["frames_written"] == r["rows"]
        assert b["measured"]["frames_dropped_queue"] == 0 and b["measured"]["fallback"] == []
        rows = read_frames_csv(raw / csv_name)
        assert rows[0]["frame_idx"] == "1" and float(rows[0]["pc_ts_wall"]) >= t_rsi - 0.5   # waited for RSI
    assert all(f["file"].endswith(".avi") for f in res["elp_frames_cams.csv"]["files"])
    assert len(res["elp_frames_cams.csv"]["files"]) >= 2                                      # 2 s segments
    assert [f["file"][-4:] for f in res["borescope_frames_cams.csv"]["files"]] == [".mp4"]


# ---------------------------------------------------------------------------------------- T11: bore1 readout
def test_closed_a_b1_bringup_with_the_bore1_readout(tmp_path):
    """Plan A_b1 (E1's plan) in closed mode: print_session.bore1_readout starts width_readout_bore1 (replay head on
    E1's bore1 log, no weights in CI) on the synthetic bore1 camera; its records go into the bag, bag_to_legacy writes
    raw_video/<ses>/bore1_rt_<ses>.csv, and the node writes its session_meta block."""
    from krc_sim.sim import SimConfig, SimProcess

    from print_session import topics as T
    from print_session.bag_io import read_bag
    from print_session.bag_to_legacy import main as export_main
    from print_session.legacy_cols import B1_COLS
    from print_session.startup import pattern_dirs
    from e1_paths import DATA

    port = _free_udp_port()
    params = _params(tmp_path, port, plan="A_b1")
    y = yaml.safe_load(params.read_text(encoding="utf-8"))
    y["print_session"]["ros__parameters"]["bore1_readout"] = True
    y["width_readout_bore1"] = dict(ros__parameters=dict(
        head="replay", replay_csv=str(DATA / "raw_video" / "bore1_rt_20261022_165128.csv.gz")))
    params.write_text(yaml.safe_dump(y), encoding="utf-8")
    out_dir = tmp_path / "sessions"
    proc = _launch("mode:=closed", f"params_file:={params}", f"out_dir:={out_dir}", "session:=b1",
                   f"cameras:={_synthetic_cameras(tmp_path)}")
    sdir = out_dir / "b1"
    try:
        t_end = time.monotonic() + 60
        while time.monotonic() < t_end and not ((sdir / "session_meta.json").exists() and _bound(port)):
            assert proc.poll() is None, proc.stdout.read()[-4000:]
            time.sleep(0.2)
        assert _bound(port), "bridge did not bind"
        time.sleep(3.0)                                # discovery (recorder, readouts, cameras)
        pat = next(d / "pattern_usr_ELP_R1v30_CLp_261022.json" for d in pattern_dirs()
                   if (d / "pattern_usr_ELP_R1v30_CLp_261022.json").exists())
        st = SimProcess(SimConfig(port=port, pattern_file=str(pat), duration_s=6.0)).start().result(60)
    except BaseException:
        os.killpg(proc.pid, signal.SIGINT)
        print(proc.communicate(timeout=60)[0][-6000:])
        raise
    out = _finish(proc, 90)
    print(out[-3000:])
    assert proc.returncode == 0, out[-4000:]
    assert st.sent >= 1200 and not st.timeout_stopped and st.nonzero_rkorr == 0
    assert "[width_readout_bore1" in out and "REPLAY head" in out

    meta = json.loads((sdir / "session_meta.json").read_text(encoding="utf-8"))
    assert meta["print_session"]["bore1_readout"] is True and meta["realtime"]["law"]["plan"] == "A_b1"
    b = meta["bore1_readout"]
    assert b["final"] is True and b["head"]["kind"] == "replay" and b["frame_time"] == "receive"
    assert b["summary"]["frames"] > 20 and b["summary"]["errors"] == 0

    msgs = read_bag(sdir / "bag")
    recs = msgs.get(T.BORE1_RECORD, [])
    assert 20 < len(recs) <= b["summary"]["frames"]
    gates = {r.gate for r in recs}
    assert gates <= {"ok", "stale", "no_track", "z_or_dir", "leg_or_x"} and gates - {"stale"}, gates
    assert all(r.pushed == 0 for r in recs if r.gate == "ok")       # no logged prediction at these frame times
    assert {c.plan for c in msgs[T.CTRL]} == {"A_b1"}

    exp = tmp_path / "export"
    assert export_main([str(sdir / "bag"), "--out", str(exp)]) == 0
    lines = (exp / "raw_video" / "b1" / "bore1_rt_b1.csv").read_text(encoding="utf-8").splitlines()
    assert lines[0].split(",") == B1_COLS and len(lines) - 1 == len(recs)
