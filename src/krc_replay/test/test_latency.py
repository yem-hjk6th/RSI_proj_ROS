"""T9 latency test (krc_replay.latency_test).

- core: the report on hand-made records (counts, window, absent OvCmd, PASS / FAIL);
- loopback, no ROS: the CLI against krc_rsi_bridge's reply process (closed, config/e1.yaml limits);
- loopback, ROS: the two documented commands, `ros2 launch krc_replay latency_bridge.launch.py` and
  `ros2 run krc_replay latency_test`, for 15 s. The report is printed into the test output.
"""
import os
import shutil
import signal
import subprocess
import time
from types import SimpleNamespace

import pytest

import replay_paths as P

DURATION_S = 15.0           # the CI run (T9: <= 20 s)


def _stats(**kw):
    from krc_sim.sim import SimStats
    st = SimStats()
    for k, v in kw.items():
        setattr(st, k, v)
    return st


def test_link_report_counts():
    from krc_replay.report import link_report
    st = _stats(sent=10, on_time=8, late_total=2, max_consecutive_late=2, latencies_s=[0.001] * 7 + [0.003])
    rec = SimpleNamespace(ov=[40, 40, 100, 12, 11, -1, 101, 40], rkorr_zero=[True] * 7 + [False])
    rep = link_report("127.0.0.1", 1, 0.04, st, rec, (12, 100))
    assert (rep.sent, rep.late_total, rep.max_consecutive_late, rep.replies) == (10, 2, 2, 8)
    assert rep.replies_out_of_window == 2            # 11 and 101
    assert rep.replies_without_ovcmd == 1
    assert rep.replies_nonzero_rkorr == 1
    assert rep.ov_seen == [11, 12, 40, 100, 101]
    assert rep.rtt_ms_p50 == pytest.approx(1.0) and rep.rtt_ms_max == pytest.approx(3.0)
    assert not rep.ok()
    txt = rep.text()
    for needle in ("packets sent", "late replies", "longest run 2", "Timeout                 no", "p50", "p99",
                   "nonzero RKorr", "OvCmd outside 12..100", "1 % override", "FAIL"):
        assert needle in txt, needle


def test_link_report_pass_and_timeout():
    from krc_replay.report import link_report
    rec = SimpleNamespace(ov=[40, 100], rkorr_zero=[True, True])
    ok = link_report("h", 1, 1, _stats(sent=2, on_time=2, latencies_s=[1e-4, 2e-4]), rec, (12, 100))
    assert ok.ok() and "PASS" in ok.text()
    to = link_report("h", 1, 1, _stats(sent=100, late_total=100, max_consecutive_late=100, timeout_stopped=True),
                     rec, (12, 100))
    assert not to.ok() and "Timeout                 YES" in to.text()


def test_cli_no_answer(capsys):
    from krc_replay.latency_test import main
    rc = main(["--host", "127.0.0.1", "--port", str(P.free_udp_port()), "--duration", "1", "--wait", "0.5",
               "--params-file", str(P.E1_YAML)])
    assert rc == 2 and "no answer" in capsys.readouterr().out


def test_cli_against_reply_process(capsys, tmp_path):
    """No ROS: the bridge's reply process alone, closed mode, the bringup's limits from config/e1.yaml."""
    import json
    from krc_replay.latency_test import main
    from krc_replay.params import bridge_params, controller_params
    from krc_rsi_bridge.decider import DeciderConfig
    from krc_rsi_bridge.server import RsiServer, ServerConfig
    port = P.free_udp_port()
    bp = bridge_params(controller_params(P.E1_YAML), port)
    dc = DeciderConfig(**{k: v for k, v in bp.items() if k in DeciderConfig.__dataclass_fields__})
    srv = RsiServer(ServerConfig(host="127.0.0.1", port=port, pattern_file=bp["pattern_file"], decider=dc))
    srv.start()
    try:
        out_json = tmp_path / "latency.json"
        rc = main(["--host", "127.0.0.1", "--port", str(port), "--duration", "4", "--params-file", str(P.E1_YAML),
                   "--json", str(out_json)])
    finally:
        srv.stop()
    out = capsys.readouterr().out
    with capsys.disabled():
        print("\n[latency_test, reply process only]\n" + out)
    rep = json.loads(out_json.read_text(encoding="utf-8"))
    assert rc == 0, out
    assert rep["sent"] >= 900 and not rep["timeout_stopped"]
    assert rep["replies_nonzero_rkorr"] == 0 and rep["replies_out_of_window"] == 0
    assert rep["replies_without_ovcmd"] == 0 and rep["ov_window"] == [12, 100]
    assert set(rep["ov_seen"]) <= {40, 100}          # no controller: ov_default inside, ov_outside outside


def _bound(port: int) -> bool:
    import errno
    import socket
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.bind(("127.0.0.1", port))
            return False
        except OSError as e:
            return e.errno == errno.EADDRINUSE


@pytest.mark.skipif(shutil.which("ros2") is None, reason="needs a sourced ROS 2 workspace")
def test_latency_commands_on_loopback(capsys, tmp_path):
    pytest.importorskip("krc_interfaces.msg")
    port = P.free_udp_port()
    bridge = subprocess.Popen(
        ["ros2", "launch", "krc_replay", "latency_bridge.launch.py", f"params_file:={P.E1_YAML}",
         "host:=127.0.0.1", f"port:={port}"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        t_end = time.monotonic() + 60
        while not _bound(port):
            assert bridge.poll() is None, bridge.stdout.read()[-4000:]
            assert time.monotonic() < t_end, "bridge did not bind"
            time.sleep(0.2)
        out_json = tmp_path / "latency.json"
        cmd = ["ros2", "run", "krc_replay", "latency_test", "--host", "127.0.0.1", "--port", str(port),
               "--duration", f"{DURATION_S:g}", "--params-file", str(P.E1_YAML), "--json", str(out_json)]
        run = subprocess.run(cmd, capture_output=True, text=True, timeout=DURATION_S + 60)
    finally:
        os.killpg(bridge.pid, signal.SIGINT)
        try:
            b_out = bridge.communicate(timeout=20)[0]
        except subprocess.TimeoutExpired:
            os.killpg(bridge.pid, signal.SIGKILL)
            b_out = bridge.communicate()[0]
    with capsys.disabled():
        print("\n[latency_test on loopback] $ " + " ".join(cmd[:3] + ["latency_test", "--host", "127.0.0.1",
                                                                    "--port", str(port), "--duration",
                                                                    f"{DURATION_S:g}", "--params-file",
                                                                    "config/e1.yaml"])
              + "\n" + run.stdout + run.stderr)
    assert "[latency_bridge] krc_rsi_bridge closed mode on 127.0.0.1" in b_out, b_out[-3000:]
    assert run.returncode == 0, run.stdout + run.stderr
    import json
    rep = json.loads(out_json.read_text(encoding="utf-8"))
    assert rep["sent"] >= 0.95 * DURATION_S / 0.004
    assert not rep["timeout_stopped"] and rep["replies_nonzero_rkorr"] == 0
    assert rep["replies_out_of_window"] == 0 and rep["replies_without_ovcmd"] == 0
