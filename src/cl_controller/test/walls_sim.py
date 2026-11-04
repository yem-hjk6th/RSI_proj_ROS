"""Synthetic closed-loop run of a print program for the T12 tests, with the legacy _ml modules or with the port.

One deterministic loop drives either backend:
- krc_sim's PatternPath moves the TCP, one RSI packet every 4 ms, at v100 x override / 100. The override of a packet
  is the bridge decision after the previous packet: ov_outside outside the controller's print window, ov_default
  before the first publish and once forced (the RSI process latch, without its ramp), else the last published OvCmd;
  clamped to [ov_min, ov_max] (ov_min includes the speed floor).
- ELP frames at FRAME_HZ, each handled FRAME_DELAY_S after its time: the readout gate (RtReadout.gate) on the RSI
  history. A frame that passes gives synthetic rows d = 12.5 .. 24.5 mm (s_dep = s_tip - d >= 0.5), unless the plant
  blanks the ELP there (then no ELP rows, as if no frame came), and one bore1 row (d = 5 mm).
- A controller step every STEP_S (law update every t_u). A step is recorded as the state the bridge acts on after it:
  (published OvCmd, OverrideCmd.source), ov_default once forced.
Plant: the material laid at s_dep on the current -X pass of a leg has width W* (v_ref q / v_laid)^alpha_m, with ln q =
plant.lnq(leg, s_dep) + a small ripple and v_laid the path speed when the nozzle was there, lagged by tau_v; bore1 adds
plant.b1_bias to ln q.

Backends:
- "legacy": legacy/realtime_cl_261023_ml rt_readout.RtReadout.gate (rig latency only), rt_controller.Controller with
  rt_law_v3, bore1_rt.RowStore and one rsi_ring.RsiRing shared by readout and controller (as run_cl.py does). Its
  CtrlCfg / LawCfg carry the port's build_configs values; the plateaus come from the legacy rt_law_v3.
- "port": width_readout_elp.readout.RtReadout.gate on width_readout_elp.rsi_ring.RsiRing, cl_controller Controller.
"""
import math
import sys
import types
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Callable, Dict, List, Optional

import numpy as np

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[1]
for _pkg in ("krc_sim", "width_readout_elp"):          # source tree runs; installed packages win in colcon test
    try:
        __import__(_pkg)
    except ImportError:
        sys.path.append(str(SRC / _pkg))

from krc_sim.path import PatternPath  # noqa: E402
from width_readout_elp.config import default_rig_json  # noqa: E402
from width_readout_elp.readout import RtReadout  # noqa: E402
from width_readout_elp.rsi_ring import RsiRing  # noqa: E402

import legacy_import  # noqa: E402
from cl_controller.controller import Controller, CtrlCfg  # noqa: E402
from cl_controller.law import LawCfg  # noqa: E402
from cl_controller.params import build_configs, defaults  # noqa: E402
from cl_controller.pattern import load_pattern  # noqa: E402

PATTERNS = HERE.parent / "patterns"
WALL_CL = PATTERNS / "pattern_usr_ELP_W4L28_CL_261023.json"
SINGLE_CLP = PATTERNS / "pattern_usr_ELP_R1v30_CLp_261022.json"

PERIOD_S = 0.004
STEP_S = 0.05
FRAME_HZ = 30.0
FRAME_DELAY_S = 0.010
T0 = 1000.0
IPOC0 = 1_000_000
D_ELP = np.arange(12, 25) + 0.5
SKIPS = ("", "stale", "no_track", "z_or_dir", "leg_or_xwin", "min_s_tip")


@dataclass
class Plant:
    lnq: Callable[[int, float], float] = lambda leg, s: 0.0
    elp_off: Callable[[int, float], bool] = lambda leg, s_tip: False
    b1_bias: float = 0.08
    ripple: float = 0.015


@dataclass
class Run:
    frames: List[tuple] = field(default_factory=list)     # (frame j, skip, leg, dz, s_tip)
    steps: List[tuple] = field(default_factory=list)      # (t, published OvCmd, OverrideCmd.source)
    updates: List[dict] = field(default_factory=list)     # one per law update
    bridge: List[int] = field(default_factory=list)       # per packet: the override the TCP moved at
    t_open: Optional[float] = None
    t_close: Optional[float] = None
    sat_alarm: bool = False
    speed_alarm: bool = False
    n_packets: int = 0
    cfg: Optional[CtrlCfg] = None
    law_cfg: Optional[LawCfg] = None


class _Ro:
    def __init__(self, leg, s_tip, d, s, w):
        self.leg, self.s_tip = leg, s_tip
        self.rows = np.column_stack([d, s, w])


class PortBackend:
    def __init__(self, pj, cfg, law_cfg, plateaus, mode):
        self.pat = load_pattern(pj)
        self.ctrl = Controller(cfg, self.pat, law_cfg, plateaus, mode=mode)
        self.readout = RtReadout(default_rig_json(), pj)
        self.ring = RsiRing()

    def push_packet(self, t, ipoc, xyz):
        self.ctrl.push_state(t, ipoc, xyz)
        self.ring.push(t, xyz)

    def gate(self, t_f):
        g = self.readout.gate(t_f, self.ring)
        return (self.readout.last_skip, -1, math.nan, math.nan) if g is None else ("", g.leg, g.dz, g.s_tip)

    def push_rows(self, src, leg, s_tip, d, s, w, now):
        self.ctrl.push_rows(src, leg, s_tip, d, s, w, now)

    def step(self, now):
        self.ctrl.step(now)
        return self.ctrl.published_ov(), self.ctrl.source_label(), self.ctrl.t_update_ok == now

    @property
    def win(self):
        return self.ctrl.win

    def forced(self):
        return self.ctrl.force_default


class LegacyBackend:
    def __init__(self, pj, cfg, law_cfg, mode, lat):
        self._cm = legacy_import.ml()
        L = self._cm.__enter__()                  # left open for the run: legacy code imports inside functions
        rc, law, rr = L["rt_controller"], L["rt_law_v3"], L["rt_readout"]
        lcfg = rc.CtrlCfg(**{f.name: getattr(cfg, f.name) for f in fields(cfg)})
        llc = law.LawCfg(**{f.name: getattr(law_cfg, f.name) for f in fields(law_cfg)})
        self.pat = rr.Pattern(pj, pj.parent)
        plateaus = law.plateaus_at_ov(law.pattern_speed_info(pj), lcfg.ov_base)
        self.ctrl = rc.Controller(lcfg, self.pat, mode, llc, plateaus)
        self.ring = L["rsi_ring"].RsiRing(tau_v=lcfg.tau_v)
        self.readout = object.__new__(rr.RtReadout)         # the gate needs only the rig latency
        self.readout.rig, self.readout.pat, self.readout.last_skip = types.SimpleNamespace(lat=lat), self.pat, ""
        self._pub = None
        self.ctrl.publish = lambda ov, code, hb: setattr(self, "_pub", ov)
        self._upd = False
        self.ctrl.on_update = lambda now, c: setattr(self, "_upd", True)

    def close(self):
        self._cm.__exit__(None, None, None)

    def push_packet(self, t, ipoc, xyz):
        self.ring.push(t, ipoc, xyz)
        self.ctrl.win.update(t, xyz)

    def gate(self, t_f):
        g = self.readout.gate(t_f, self.ring)
        return (self.readout.last_skip, -1, math.nan, math.nan) if g is None else ("", g.leg, g.dz, g.s_tip)

    def push_rows(self, src, leg, s_tip, d, s, w, now):
        ro = _Ro(leg, s_tip, d, s, w)
        (self.ctrl.push_rows if src == "elp" else self.ctrl.push_bore1)(ro, now)

    def step(self, now):
        self._upd = False
        self.ctrl.step(now, self.ring)
        c = self.ctrl
        forced = c.speed_alarm or c.sat_alarm             # run_cl latches the RSI process ([f] / speed / wall rule)
        ov = c.cfg.ov_default if forced else self._pub
        label = "forced" if forced else (c.law.state if c.out_src == "control" else "default")
        return ov, label, self._upd

    @property
    def win(self):
        return self.ctrl.win

    def forced(self):
        return self.ctrl.speed_alarm or self.ctrl.sat_alarm


def configs(pj, params):
    """(CtrlCfg, LawCfg, plateaus, mode) from node parameters (port build_configs)."""
    p = {**defaults(), "mode": "closed", "plan": "A", **params}
    cfg, lc, pl = build_configs(p, load_pattern(pj))
    return cfg, lc, pl, p["mode"]


def update_row(c, now, pat) -> dict:
    """The fields of one law update compared between legacy and port (same for both controllers)."""
    i, L = c.info, c.law
    r = i.get("row") or {}
    leg = int(i["leg"])
    return dict(n_upd=int(c.n_updates), t=float(now), win=c.win.state, leg=leg, on_a=int(bool(i["on_a"])),
                dz=pat.plane_dz(leg) if leg >= 1 else math.nan, s_tip=float(i["s_tip"]), state=str(r.get("state", "")),
                reason=str(r.get("reason", "")), valid_elp=str(r.get("valid_elp", "")),
                valid_b1=str(r.get("valid_bore1", "")), n_bins=int(i["n_bins"]), rhat=float(i["rhat"]),
                u=float(L.u), ov_pub=int(c.cfg.ov_default if c.out_src == "outside" else c.out_ov), src=c.out_src,
                forced=int(bool(c.force_default)), sat=int(bool(c.sat_alarm)), n_corr=int(L.n_corr),
                n_switch=int(L.n_switch))


def run(backend: str, pj: Path = WALL_CL, params: Optional[dict] = None, plant: Optional[Plant] = None,
        t_max: Optional[float] = None, until: Optional[Callable[[Run], bool]] = None,
        cfg_hook: Optional[Callable[[CtrlCfg], None]] = None) -> Run:
    """Run the program `pj` closed loop (see the module docstring). t_max: seconds of packets at most; until(run):
    stop early once it returns True (checked after every controller step); cfg_hook(cfg): change the CtrlCfg that
    build_configs made (both backends get the changed one)."""
    plant = plant or Plant()
    cfg, lc, pl, mode = configs(pj, params or {})
    if cfg_hook is not None:
        cfg_hook(cfg)
    be = PortBackend(pj, cfg, lc, pl, mode) if backend == "port" else LegacyBackend(
        pj, cfg, lc, mode, RtReadout(default_rig_json(), pj).rig.lat)
    try:
        return _loop(be, pj, cfg, lc, plant, t_max, until)
    finally:
        if backend != "port":
            be.close()


def _loop(be, pj, cfg, lc, plant, t_max, until) -> Run:
    out = Run(cfg=cfg, law_cfg=lc)
    path = PatternPath(pj, lead_in_mm=20.0)
    x_start = be.pat.x_start
    v_ref, alpha, w_star, tau_v = cfg.v_ref_mm_s, cfg.alpha_m, cfg.w12_t, cfg.tau_v
    lo, hi = max(1, cfg.ov_min), min(100, cfg.ov_max)
    passes: Dict[int, tuple] = {}            # leg -> ([s], [v_laid]) of its current -X pass
    cur_leg = None
    v_lag = 0.0
    ov_applied = 100
    published = None
    next_step = T0
    j_next, k = 0, 0
    tail = 0
    while True:
        t = T0 + k * PERIOD_S
        if t_max is not None and t - T0 > t_max:
            break
        v = path.step(PERIOD_S, ov_applied) if k else 0.0
        xyz = path.pos
        v_lag += (1.0 - math.exp(-PERIOD_S / tau_v)) * (v - v_lag)
        row = path.row
        if row is not None and row["kind"] == "A" and float(row["x1"]) < float(row["x0"]):
            leg = int(row["leg"])
            if leg != cur_leg:
                passes[leg] = ([], [])
                cur_leg = leg
            s_now = x_start - xyz[0]
            ss, vv = passes[leg]
            if not ss or s_now > ss[-1]:
                ss.append(s_now)
                vv.append(v_lag)
        else:
            cur_leg = None
        be.push_packet(t, float(IPOC0 + 4 * k), np.array(xyz, float))
        if out.t_open is None and be.win.state != "before":
            out.t_open = t
        if out.t_close is None and be.win.state == "after":
            out.t_close = t
        # ELP frames handled by now
        while T0 + 0.0013 + j_next / FRAME_HZ + FRAME_DELAY_S <= t:
            t_f = T0 + 0.0013 + j_next / FRAME_HZ
            skip, leg, dz, s_tip = be.gate(t_f)
            out.frames.append((j_next, skip, leg, dz, s_tip))
            if not skip and leg in passes and passes[leg][0]:
                ss, vv = (np.asarray(a) for a in passes[leg])
                if not plant.elp_off(leg, s_tip):
                    s_dep = s_tip - D_ELP
                    keep = s_dep >= 0.5
                    d, s = D_ELP[keep], s_dep[keep]
                    be.push_rows("elp", leg, s_tip, d, s, _width(plant, leg, s, ss, vv, v_ref, alpha, w_star, 0.0), t)
                sb = np.array([s_tip - 5.0])
                be.push_rows("bore1", leg, s_tip, np.array([5.0]), sb,
                             _width(plant, leg, sb, ss, vv, v_ref, alpha, w_star, plant.b1_bias), t)
            j_next += 1
        # controller step
        if t >= next_step:
            next_step += STEP_S
            ov, label, updated = be.step(t)
            published = ov
            out.steps.append((t, int(ov), label))
            if updated:
                out.updates.append(update_row(be.ctrl, t, be.pat))
            if until is not None and until(out):
                break
        # bridge decision for the next packet
        if be.win.state != "inside":
            ov_b = cfg.ov_outside
        elif be.forced() or published is None:
            ov_b = cfg.ov_default
        else:
            ov_b = published
        ov_applied = int(min(max(ov_b, lo), hi))
        out.bridge.append(ov_applied)
        k += 1
        if path.done:
            tail += 1
            if tail > int(0.5 / PERIOD_S):
                break
    out.n_packets = k
    out.sat_alarm, out.speed_alarm = bool(be.ctrl.sat_alarm), bool(be.ctrl.speed_alarm)
    return out


def _width(plant, leg, s, ss, vv, v_ref, alpha, w_star, bias):
    v_laid = np.maximum(np.interp(s, ss, vv), 0.5)
    lnq = np.array([plant.lnq(leg, float(x)) for x in s]) + bias + plant.ripple * np.sin(2.1 * s + 0.7 * leg)
    return w_star * np.exp(alpha * (lnq - np.log(v_laid / v_ref)))


# ------------------------------------------------------------------------------------------------ fixture encoding
UPDATE_COLS = ("n_upd", "k", "win", "leg", "on_a", "dz", "s_tip", "state", "reason", "valid_elp", "valid_b1", "n_bins",
               "rhat", "u", "ov_pub", "src", "forced", "sat", "n_corr", "n_switch")
UPDATE_FLOATS = ("dz", "s_tip", "rhat", "u")


def rle(seq):
    """[[value..., count]] runs of equal consecutive items (tuples)."""
    out = []
    for x in seq:
        x = list(x) if isinstance(x, tuple) else [x]
        if out and out[-1][:-1] == x:
            out[-1][-1] += 1
        else:
            out.append(x + [1])
    return out


def encode(r: Run) -> dict:
    """The compared content of a run in a compact form (frames and steps as runs, updates row by row)."""
    frames = []
    for j, skip, leg, dz, s_tip in r.frames:
        key = [skip, leg, None if skip else dz]
        if frames and frames[-1][:3] == key and frames[-1][3] + frames[-1][4] == j:
            f = frames[-1]
            f[4] += 1
            if not skip:
                f[6], f[7] = s_tip, f[7] + s_tip
        else:
            frames.append(key + [j, 1] + ([s_tip, s_tip, s_tip] if not skip else [None, None, None]))
    ks = [int(round((t - T0) / PERIOD_S)) for t, _, _ in r.steps]
    steps = rle((ov, label) for _, ov, label in r.steps)
    ups = []
    for u in r.updates:
        u = dict(u, k=int(round((u["t"] - T0) / PERIOD_S)))
        ups.append([None if (c in UPDATE_FLOATS and math.isnan(u[c])) else u[c] for c in UPDATE_COLS])
    return dict(n_packets=r.n_packets, k_open=int(round((r.t_open - T0) / PERIOD_S)) if r.t_open else None,
                k_close=int(round((r.t_close - T0) / PERIOD_S)) if r.t_close else None, sat_alarm=r.sat_alarm,
                speed_alarm=r.speed_alarm, frame_cols=["skip", "leg", "dz", "j0", "n", "s_tip_first", "s_tip_last",
                                                       "s_tip_sum"],
                frames=frames, step_k=ks[:3], n_steps=len(ks), steps=steps, bridge=rle(r.bridge),
                update_cols=list(UPDATE_COLS), updates=ups)


# ------------------------------------------------------------------------------------------------ the golden plant
def _ramp(s, s0, width=30.0):
    return min(1.0, max(0.0, (s - s0) / width))


def golden_lnq(leg: int, s: float) -> float:
    """ln flow of the golden run: a per-leg level, a step inside legs 7 / 20 / 33, saturation (OvCmd at ov_max)
    on leg 2 (layer 1: no stop) from s 25 to 120 mm and on leg 50 (layer 25: the wall stop rule) from s 40 mm on.
    Leg 2 ends its saturation early enough for u to come back before leg 3: u is held off the A legs, and on the first
    leg of a layer >= 2 the ELP sees nothing valid for the first 50..80 mm (see the T12 PR), so a saturated u carried
    into leg 3 trips the rule there."""
    base = 0.12 * math.sin(1.3 * leg) + 0.06 * ((leg // 2) % 3 - 1)
    if leg in (7, 20, 33):
        base += 0.2 * _ramp(s, 90.0, 10.0)
    if leg == 2:
        base += (0.97 - base) * _ramp(s, 25.0, 10.0) * (1.0 - _ramp(s, 120.0, 5.0))
    if leg == 50:
        base += (0.97 - base) * _ramp(s, 40.0)
    return base


def golden_elp_off(leg: int, s_tip: float) -> bool:
    """ELP blackout on legs 10 and 23 (layers 5 and 12) for s_tip 70..120 mm: bore1 is valid but may not steer."""
    return leg in (10, 23) and 70.0 < s_tip < 120.0


GOLDEN_PLANT = Plant(lnq=golden_lnq, elp_off=golden_elp_off)
GOLDEN_PARAMS = dict(control_sources="elp")      # walls: only the ELP steers; sat_stop_s derived (2.0 s)
