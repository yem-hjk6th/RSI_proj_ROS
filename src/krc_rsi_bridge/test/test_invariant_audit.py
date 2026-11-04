"""Safety-invariant audit (docs/SAFETY_INVARIANTS.md): tests that drive the real reply path of the RSI bridge.

- invariant 1: every reply the reply process sends, in every mode, carries RKorr = 0, and the only reply builders
  are the two zero-RKorr ones;
- invariant 2: the decider's output stays in [max(ov_min, floor), ov_max] over random configurations and random
  input sequences (controller values far outside the window, forced, stale, outside, broken positions);
- invariant 3: a stale heartbeat over UDP ramps the OvCmd in the reply bytes linearly to ov_default over
  fallback_ramp_s; random ramp parameters give the exact linear ramp in the decider;
- invariant 4: the reply loop contains no logging call, and a slow logging handler installed in the reply process and
  in the parent does not delay the replies;
- invariant 5: the reply carries no motion element, and no package depends on or imports a motion-command interface.
"""
import ast
import inspect
import logging
import math
import multiprocessing as mp
import os
import random
import re
import socket
import textwrap
import threading
import time
from pathlib import Path

import pytest
from bridge_paths import PATTERN, SRC
from loop_helpers import assert_pass, report

from krc_rsi_bridge import rsi_xml, server
from krc_rsi_bridge.decider import (CTRL_CONTROL, CTRL_DEFAULT, SRC_CONTROL, SRC_WATCHDOG, DeciderConfig,
                                    OverrideDecider, effective_limits)
from krc_rsi_bridge.server import RsiServer, ServerConfig
from krc_rsi_bridge.shared import F
from krc_rsi_bridge.window import PatternGeom
from krc_sim.packet import build_packet, parse_reply

PAT = PatternGeom.load(PATTERN)
INSIDE = (PAT.first_pt[0], PAT.first_pt[1], PAT.z_cmd(PAT.first_pt[0]))
OUTSIDE = (0.0, 0.0, 500.0)
BAD_POS = [None, (math.nan,) * 3, (math.inf, -math.inf, math.nan), (1.0,), "x"]


# ------------------------------------------------------------------------------------------------ invariant 2
def _random_cfg(rng: random.Random, mode: str) -> DeciderConfig:
    while True:
        lo_allowed = 0 if mode == "step" else 1
        ov_min = rng.randint(lo_allowed, 60)
        ov_max = rng.randint(ov_min, 100)
        cfg = DeciderConfig(mode=mode, ov_min=ov_min, ov_max=ov_max, ov_outside=rng.randint(-500, 500),
                            ov_default=rng.randint(ov_min, ov_max), v_min_mm_s=rng.choice([0.0, 1.0, 3.4, 6.0]),
                            fallback_ramp_s=rng.choice([0.0, 0.3, 2.0]), wd_s=rng.choice([0.05, 0.2, 1.0]),
                            step_seq=tuple(rng.randint(-200, 300) for _ in range(5)), step_hold_s=0.1)
        try:
            effective_limits(cfg, PAT)
            return cfg
        except ValueError:
            continue


@pytest.mark.parametrize("mode", ["closed", "shadow", "step"])
@pytest.mark.parametrize("seed", range(6))
def test_decider_output_always_in_window_random_inputs(mode, seed):
    """Invariant 2 as a property: whatever the configuration and the inputs, OvCmd is in [lo, hi]."""
    rng = random.Random(1000 * seed + len(mode))
    cfg = _random_cfg(rng, mode)
    d = OverrideDecider(cfg, PAT)
    lo, hi, floor = effective_limits(cfg, PAT)
    assert lo == max(cfg.ov_min, floor)
    t, hb, forced = 0.0, 0.0, False
    values = [-2**31, -1000, -1, 0, 1, 50, 99, 100, 101, 1000, 2**31 - 1]
    for k in range(3000):
        t += 0.004
        r = rng.random()
        if r < 0.6:
            hb = t                                       # fresh heartbeat
        elif r < 0.62:
            hb = -math.inf                               # never any
        if rng.random() < 0.002:
            forced = True                                # latch: never cleared below
        pos = INSIDE if rng.random() < 0.9 else (OUTSIDE if rng.random() < 0.5 else rng.choice(BAD_POS))
        ov_ctrl = rng.choice(values) if rng.random() < 0.5 else rng.randint(-300, 300)
        ov, src = d.decide(t, pos, forced, hb, ov_ctrl, rng.choice([CTRL_DEFAULT, CTRL_CONTROL]))
        assert isinstance(ov, int) and lo <= ov <= hi, (k, ov, src, lo, hi)


def test_decider_broken_positions_give_clamped_fallback():
    """A packet whose RIst could not be parsed (NaN, None, short) still gets a value inside the window."""
    d = OverrideDecider(DeciderConfig(ov_min=1, ov_max=80, ov_outside=100, ov_default=40), PAT)
    for pos in BAD_POS:
        ov, _ = d.decide(0.0, pos, False, 0.0, 95, CTRL_CONTROL)
        assert 12 <= ov <= 80


@pytest.mark.parametrize("kw", [dict(v_min_mm_s=math.nan), dict(v_min_mm_s=math.inf), dict(fallback_ramp_s=math.nan),
                                dict(fallback_ramp_s=math.inf), dict(wd_s=math.nan), dict(wd_s=math.inf),
                                dict(wd_s=-1.0), dict(ov_min=0, v_min_mm_s=0.0),
                                dict(ov_min=-5, v_min_mm_s=0.0), dict(ov_max=0), dict(ov_max=101)])
def test_bridge_refuses_non_finite_or_out_of_range_config(kw):
    """The bridge refuses at start (RsiServer / OverrideDecider) any limit that would disable the clamp, the floor,
    the watchdog or the ramp (ValueError; OverflowError is an ArithmeticError raised by the floor of v_min = inf)."""
    cfg = DeciderConfig(**{"ov_default": 40, **kw})
    with pytest.raises((ValueError, OverflowError)):
        OverrideDecider(cfg, PAT)


# ------------------------------------------------------------------------------------------------ invariant 3
@pytest.mark.parametrize("seed", range(8))
def test_watchdog_ramp_is_exactly_linear_random_parameters(seed):
    rng = random.Random(seed)
    ov_default = rng.randint(20, 60)
    ov_ctrl = rng.choice([rng.randint(12, 100), 12, 100])
    ramp_s = rng.choice([0.25, 0.8, 2.0, 3.3])
    wd_s = rng.choice([0.1, 0.5, 1.0])
    d = OverrideDecider(DeciderConfig(ov_default=ov_default, fallback_ramp_s=ramp_s, wd_s=wd_s), PAT)
    dt = 0.004
    t = 0.0
    for _ in range(100):                                       # control with a fresh heartbeat
        t += dt
        assert d.decide(t, INSIDE, False, t, ov_ctrl, CTRL_CONTROL) == (ov_ctrl, SRC_CONTROL)
    hb = t
    out = []
    while t < hb + wd_s + ramp_s + 0.5:
        t += dt
        out.append((t,) + d.decide(t, INSIDE, False, hb, ov_ctrl, CTRL_CONTROL))
    wd = [(tt, ov) for tt, ov, src in out if src == SRC_WATCHDOG]
    assert [src for _, _, src in out if src != SRC_WATCHDOG] == [SRC_CONTROL] * (len(out) - len(wd))
    t0 = wd[0][0]
    assert t0 - hb > wd_s and t0 - hb <= wd_s + dt + 1e-9      # fires on the first packet after wd_s
    for tt, ov in wd:
        f = min(1.0, (tt - t0) / ramp_s)
        assert ov == int(round(ov_ctrl + f * (ov_default - ov_ctrl)))
    assert all(ov == ov_default for tt, ov in wd if tt - t0 >= ramp_s)
    assert any(ov != ov_default for tt, ov in wd if tt - t0 < ramp_s) or ov_ctrl == ov_default


def test_stale_heartbeat_ramp_in_the_reply_bytes(udp_port, pattern_file):
    """The real reply process over UDP: control at 85, then no heartbeat. The OvCmd in the reply bytes ramps linearly
    (at the reply process's own receive times) from 85 to ov_default over fallback_ramp_s, every reply has RKorr 0."""
    ramp_s, wd_s, ov_default = 0.6, 0.2, 40
    srv = RsiServer(ServerConfig(port=udp_port, pattern_file=pattern_file,
                                 decider=DeciderConfig(ov_default=ov_default, fallback_ramp_s=ramp_s, wd_s=wd_s)))
    srv.start()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    replies = {}
    try:
        rist = INSIDE + (0.0, 90.0, 0.0)
        t_end_ctrl = time.monotonic() + 0.4
        ipoc = 5000
        while True:
            now = time.monotonic()
            if now < t_end_ctrl:
                srv.command(85, CTRL_CONTROL)
            elif now > t_end_ctrl + wd_s + ramp_s + 0.4:
                break
            ipoc += 1
            sock.sendto(build_packet(ipoc, rist, rist, 0, 100, 0, 120), ("127.0.0.1", udp_port))
            data, _ = sock.recvfrom(4096)
            replies[ipoc] = parse_reply(data)
            time.sleep(0.004)
        time.sleep(0.05)
        rows = srv.read_packets()
    finally:
        sock.close()
        srv.stop()
    assert all(rk == (0.0,) * 6 for _, _, rk in replies.values())
    by_ipoc = {int(r[F["ipoc"]]): r for r in rows}
    assert set(replies) <= set(by_ipoc)
    for ip, (ip2, ov, _) in replies.items():                   # the ring records what the reply carried
        assert ip2 == ip and ov == int(by_ipoc[ip][F["ov_sent"]])
    seq = [(by_ipoc[ip][F["t_perf"]], replies[ip][1], int(by_ipoc[ip][F["src"]])) for ip in sorted(replies)]
    assert any(src == SRC_CONTROL and ov == 85 for _, ov, src in seq)
    wd = [(t, ov) for t, ov, src in seq if src == SRC_WATCHDOG]
    assert len(wd) > 50
    t0 = wd[0][0]
    for t, ov in wd:
        f = min(1.0, (t - t0) / ramp_s)
        assert ov == int(round(85 + f * (ov_default - 85)))
    assert wd[0][1] == 85 and wd[-1][1] == ov_default
    assert len({ov for _, ov in wd}) > 10                      # a ramp, not a jump


# ------------------------------------------------------------------------------------------------ invariant 1
@pytest.mark.parametrize("mode", ["closed", "shadow", "step"])
def test_every_reply_of_the_reply_process_has_zero_rkorr(udp_port, pattern_file, mode):
    """Packets with any position, override and IPOC (also broken positions): every reply has RKorr 0 and the same
    IPOC; shadow mode sends no OvCmd, closed / step mode an OvCmd inside the window."""
    srv = RsiServer(ServerConfig(port=udp_port, pattern_file=pattern_file,
                                 decider=DeciderConfig(mode=mode, ov_default=40, step_hold_s=0.05)))
    srv.start()
    rng = random.Random(7)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    try:
        for k in range(400):
            srv.command(rng.randint(-500, 500), rng.choice([CTRL_DEFAULT, CTRL_CONTROL]))
            if rng.random() < 0.2:
                pkt = f'<Rob><RIst X="nan" Y="x" /><IPOC>{k + 1}</IPOC></Rob>'.encode()
            else:
                p = INSIDE if rng.random() < 0.7 else (rng.uniform(-2e3, 2e3), rng.uniform(-2e3, 2e3), 600.0)
                rist = tuple(p) + (rng.uniform(-180, 180), rng.uniform(-180, 180), rng.uniform(-180, 180))
                pkt = build_packet(k + 1, rist, rist, rng.randint(0, 5), rng.randint(0, 100), 0, 0)
            sock.sendto(pkt, ("127.0.0.1", udp_port))
            data, _ = sock.recvfrom(4096)
            ipoc, ov, rk = parse_reply(data)
            assert ipoc == k + 1 and rk == (0.0,) * 6
            assert rsi_xml.RKORR_ZERO.encode() in data and data.count(b"<RKorr") == 1
            if mode == "shadow":
                assert ov is None
            else:
                assert srv.ov_min <= ov <= srv.ov_max
    finally:
        sock.close()
        srv.stop()


def test_reply_bytes_rkorr_zero_random_ipoc_and_override():
    rng = random.Random(3)
    for _ in range(3000):
        ipoc = str(rng.randint(0, 2**64 - 1))
        ov = rng.randint(-2**40, 2**40)
        for b in (rsi_xml.build_ov_reply(ipoc, ov), rsi_xml.build_zero_reply(ipoc)):
            ip, _, rk = parse_reply(b)
            assert ip == int(ipoc) and rk == (0.0,) * 6


def _calls(fn):
    tree = ast.parse(textwrap.dedent(inspect.getsource(fn)))
    return [n for n in ast.walk(tree) if isinstance(n, ast.Call)]


def test_reply_loop_sends_only_the_zero_rkorr_builders():
    """The only send in the reply loop is sock.sendto(build_ov_reply(...) if ... else build_zero_reply(...), addr),
    and the RKorr element of both builders is the constant zero string."""
    sends = [c for c in _calls(server.reply_loop) if isinstance(c.func, ast.Attribute)
             and c.func.attr in ("sendto", "send", "sendall", "sendmsg")]
    assert len(sends) == 1
    payload = sends[0].args[0]
    assert isinstance(payload, ast.IfExp)
    names = {payload.body.func.id, payload.orelse.func.id}
    assert names == {"build_ov_reply", "build_zero_reply"}
    for fn in (rsi_xml.build_ov_reply, rsi_xml.build_zero_reply):
        body = ast.parse(textwrap.dedent(inspect.getsource(fn))).body[0].body
        code = "\n".join(ast.unparse(n) for n in body[1:])          # without the docstring
        assert "_REPLY_HEAD" in code and "RKorr" not in code      # no second, variable RKorr element
    assert re.fullmatch(r'<RKorr( [XYZABC]="0\.0000"){6} />', rsi_xml.RKORR_ZERO)
    assert rsi_xml._REPLY_HEAD == '<Sen Type="ImFree">' + rsi_xml.RKORR_ZERO


# ------------------------------------------------------------------------------------------------ invariant 4
def test_reply_loop_has_no_logging_and_no_blocking_ipc():
    """Between packets the reply loop never logs, prints, takes a lock, or waits on a queue / pipe."""
    bad = []
    for c in _calls(server.reply_loop):
        f = c.func
        name = f.attr if isinstance(f, ast.Attribute) else getattr(f, "id", "")
        base = f.value.id if isinstance(f, ast.Attribute) and isinstance(f.value, ast.Name) else ""
        if base in ("log", "logging") or name in ("print", "acquire", "get", "put", "recv_bytes", "join", "wait",
                                                  "get_logger", "flush"):
            bad.append(ast.unparse(c))
    assert not bad, bad
    src = inspect.getsource(server.reply_loop)
    assert "Lock" not in src and "Queue" not in src and "Pipe" not in src


class _SlowHandler(logging.Handler):
    """Sleeps on every record; counts the records emitted in a process other than the test process."""

    def __init__(self, counter, delay_s: float):
        super().__init__(logging.DEBUG)
        self.counter, self.delay_s, self.main_pid = counter, delay_s, os.getpid()

    def emit(self, record):
        if os.getpid() != self.main_pid:
            self.counter.value += 1
        time.sleep(self.delay_s)


def test_slow_logging_handler_does_not_delay_replies(udp_port, pattern_file, capsys):
    """A logging handler that blocks 0.2 s per record is installed on the root and the bridge loggers before the
    reply process forks (so the reply process inherits it), and the parent logs through it the whole time. The
    reply process emits no record, and the simulated controller sees no Timeout and no long run of late replies."""
    ctx = mp.get_context("fork")
    child_records = ctx.RawValue("q", 0)
    h = _SlowHandler(child_records, 0.2)
    loggers = [logging.getLogger(), logging.getLogger("krc_rsi_bridge"), server.log]
    old = [(lg, lg.level) for lg in loggers]
    for lg in loggers:
        lg.addHandler(h)
        lg.setLevel(logging.DEBUG)
    stop = threading.Event()

    def chatter():
        while not stop.is_set():
            server.log.info("parent-side log record")
            logging.getLogger("krc_rsi_bridge").warning("another one")

    try:
        srv = RsiServer(ServerConfig(port=udp_port, pattern_file=pattern_file, start_method="fork",
                                     decider=DeciderConfig(ov_default=40, wd_s=0.5)))
        srv.start()
        th = threading.Thread(target=chatter, daemon=True)
        th.start()
        try:
            from krc_sim.sim import SimConfig, SimProcess
            sim = SimProcess(SimConfig(port=udp_port, pattern_file=pattern_file, duration_s=4.0)).start()
            t_end = time.monotonic() + 4.0
            while time.monotonic() < t_end:
                srv.command(80, CTRL_CONTROL)
                time.sleep(0.02)
            st = sim.result(30)
        finally:
            stop.set()
            th.join(2.0)
            srv.stop()
    finally:
        for lg, lvl in old:
            lg.removeHandler(h)
            lg.setLevel(lvl)
    report(capsys, "slow logging handler 0.2 s / record", st)
    assert child_records.value == 0
    assert_pass(st, min_packets=900)


# ------------------------------------------------------------------------------------------------ invariant 5
MOTION_PKGS = ("trajectory_msgs", "control_msgs", "geometry_msgs", "moveit_msgs", "kuka_driver_interfaces",
               "controller_manager", "ros2_control", "hardware_interface")


def test_reply_carries_no_motion_element():
    """The reply's elements are Sen, RKorr (constant zero), OvCmd and IPOC: no joint, axis or Cartesian target."""
    for b in (rsi_xml.build_ov_reply("1", 50), rsi_xml.build_zero_reply("1")):
        tags = set(re.findall(rb"<([A-Za-z_]+)", b))
        assert tags <= {b"Sen", b"RKorr", b"OvCmd", b"IPOC"}


def test_no_package_depends_on_or_imports_a_motion_command_interface():
    """Invariant 5 at the package level: no package.xml depends on a motion-command interface and no node source
    imports one (KrcState.rist is the measured pose, published, never sent to the controller)."""
    hits = []
    for px in sorted(SRC.glob("*/package.xml")):
        for m in re.findall(r"<(?:\w+_)?depend>([^<]+)</", px.read_text(encoding="utf-8")):
            if m.strip() in MOTION_PKGS:
                hits.append(f"{px.parent.name}: depends on {m}")
    for py in sorted(SRC.glob("*/*/*.py")) + sorted(SRC.glob("*/launch/*.py")):
        if py.parent.name == "test":
            continue
        for line in py.read_text(encoding="utf-8").splitlines():
            if re.match(r"\s*(from|import)\s+(" + "|".join(MOTION_PKGS) + r")\b", line):
                hits.append(f"{py.relative_to(SRC)}: {line.strip()}")
    assert not hits, hits


def test_interfaces_carry_no_motion_target():
    """The only message the bridges accept from the PC side is OverrideCmd, and it has no motion field."""
    msg = SRC / "krc_interfaces" / "msg" / "OverrideCmd.msg"
    fields = [l.split()[1] for l in msg.read_text(encoding="utf-8").splitlines() if l.strip() and not l.startswith("#")]
    assert fields == ["stamp", "value_pct", "source", "seq"]
    subs = []
    for py in sorted(SRC.glob("krc_*_bridge/krc_*_bridge/*.py")):
        subs += re.findall(r"create_subscription\(\s*(\w+)", py.read_text(encoding="utf-8"))
    assert subs and set(subs) == {"OverrideCmd"}


def test_speed_floor_needs_the_pattern_speed():
    """decider.ov_floor: a floor (v_min > 0) without the pattern's speed at OvCmd 100 is refused, not skipped."""
    from krc_rsi_bridge.decider import ov_floor
    for v100 in (None, 0.0, -1.0):
        with pytest.raises(ValueError, match="speed floor needs"):
            ov_floor(3.4, v100)
    assert ov_floor(0.0, None) == 0
