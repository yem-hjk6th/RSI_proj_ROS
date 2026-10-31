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
])
def test_bad_mode_or_refused_start(tmp_path, args, needle):
    proc = _launch(*args, f"params_file:={E1_YAML}", f"out_dir:={tmp_path}", "session:=x")
    out = _finish(proc, 60)
    assert proc.returncode != 0 and needle in out, out[-2000:]
    assert not (tmp_path / "x" / "session_meta.json").exists()


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
    assert abs(len(msgs.get(T.REPLY, [])) - n_state) <= 10
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
