"""Replays of session bduo (bduo_data) through the legacy cage-2 controller and through the port, with the same
inputs in the same order and the same step schedule.

Schedule: before every controller step at time t, every RSI packet (pc_ts_perf) and every row-store push (push time)
with time <= t is applied, packets first on a tie; at a logged update time the step is a law update (legacy
Controller._update / port Controller.update, both end with the command), in between it is the command alone (legacy
_command / port command()). The OvCmd of a step is what the legacy step() publishes: ov_default while the controller
sees "outside", else its OvCmd.

`explain_live` runs the legacy controller and compares every update with the live ctrl log; where they differ it
retries the update with the next bore push advanced to before it (module docstring of test_golden_bduo.py).
"""
import copy
import math

import numpy as np

import bduo_data as B
import legacy_import

NAN = math.nan


class NoLock:
    """Stand-in for threading.Lock in a copied legacy controller (copy.deepcopy cannot copy a lock)."""

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


def _unlock(*objs):
    for o in objs:
        if hasattr(o, "_lock"):
            o._lock = NoLock()
    return objs


class _Ro:
    """What the legacy row stores read from a Readout: leg, s_tip and rows (d, s_dep, w)."""

    def __init__(self, r):
        self.leg, self.s_tip, self.rows = r.leg, r.s_tip, np.column_stack([r.d, r.s, r.w])


class Legacy:
    """The legacy cage-2 controller (realtime_cl_cage2_semi_261103) configured as session_meta.json "realtime"."""

    def __init__(self, realtime=None):
        rt = realtime or B.meta()["realtime"]
        with legacy_import.cage2() as L:
            rc, lm = L["rt_controller"], L["rt_law_v3"]
            cd = dict(rt["cfg"])
            cd["step_seq"] = tuple(cd["step_seq"])
            self.cfg = rc.CtrlCfg(**cd)
            ld = dict(rt["law"])
            ld["tgt_seq"], ld["ol_sched_mm_s"] = tuple(ld["tgt_seq"]), tuple(ld["ol_sched_mm_s"])
            lc = lm.LawCfg(**ld)
            root = B.REPO / "src" / "width_readout_elp" / "config" / "pattern"
            self.pat = L["rt_readout"].Pattern(B.PATTERN, root)
            pl = lm.plateaus_at_ov(lm.pattern_speed_info(str(B.PATTERN), str(root)), self.cfg.ov_base)
            self.ctrl = rc.Controller(self.cfg, self.pat, rt["mode"], lc, pl)
            self.ring = L["rsi_ring"].RsiRing(tau_v=self.cfg.tau_v)
        self.ctrl.on_switch = None          # switches are read from ctrl.switches
        _unlock(self.ctrl, self.ring, *self.ctrl.st.values())

    def push_packet(self, t, ipoc, xyz):
        self.ring.push(t, ipoc, xyz)
        self.ctrl.win.update(t, xyz)

    def push(self, r):
        c = self.ctrl
        fn = {"elp": c.push_rows, "elp_b": c.push_rows_b, "bore1": c.push_bore1, "bore2": c.push_bore2}[r.src]
        fn(_Ro(r), r.t)

    def update(self, t):
        self.ctrl._update(t, self.ring)

    def command(self):
        self.ctrl._command(self.ring)

    def published(self):
        c = self.ctrl
        return (c.cfg.ov_default if c.out_src == "outside" else c.out_ov), c.out_src

    def row(self, now):
        """run_cl.py log_update (cage 2), unrounded, without t_wall."""
        c = self.ctrl
        i, L, nb = c.info, c.law_act, c.law_b
        r = i.get("row") or {}
        logk = i.get("lnk_ff", 0.0) + i.get("lnk_tgt", 0.0) + i.get("lnk_ol", 0.0) + L.u
        raw = c.cfg.ov_base * math.exp(logk)
        w = c.where or {}
        return dict(t_perf=now, n_upd=c.n_updates, win=c.win.state, leg=i["leg"], on_a=int(bool(i["on_a"])),
                    s_tip=i["s_tip"], n_bins=i["n_bins"], veff=i["veff"], w_est=i["w_est"], rhat=i["rhat"],
                    logk=logk, slow=L.slow, have_est=int(c.have_est), k=math.exp(logk), ov_raw=raw,
                    ov_round=int(round(raw)), ov_pub=self.published()[0], src=c.out_src,
                    rows_age_s=now - c.last_rows_now, forced=int(c.force_default), plan=c.law.cfg.plan,
                    state=r.get("state", ""), reason=r.get("reason", ""), gate=r.get("gate", ""),
                    valid_elp=r.get("valid_elp", ""), why_elp=r.get("why_elp", ""),
                    rhat_elp=r.get("rhat_elp", NAN), lag_elp=r.get("lag_elp", NAN),
                    valid_b1=r.get("valid_bore1", ""), why_b1=r.get("why_bore1", ""),
                    rhat_b1=r.get("rhat_bore1", NAN), lag_b1=r.get("lag_bore1", NAN), e=r.get("e", NAN),
                    du=r.get("du", NAN), u=L.u, lnk_ff=i.get("lnk_ff"), v_tip=i.get("v_tip"),
                    n_corr=c.law.n_corr + (nb.n_corr if nb else 0), n_switch=c.law.n_switch + (nb.n_switch if nb else 0),
                    w_target=r.get("w_target", NAN), lnk_tgt=i.get("lnk_tgt"), nobed_s=r.get("nobed_s", NAN),
                    hold_bed=r.get("hold_bed", ""), lnk_ol=i.get("lnk_ol"), law=i.get("law", "A"),
                    kind=i.get("kind", ""), dir=w.get("dir", ""), vx=w.get("vx", NAN), src_active=c.src_active,
                    src_why=c.src_why)

    @property
    def switches(self):
        return self.ctrl.switches


class Port:
    """cl_controller.Controller with the session's parameters (bduo_data.PARAMS)."""

    def __init__(self, params=None):
        from cl_controller.controller import Controller
        from cl_controller.params import build_configs
        from cl_controller.pattern import load_pattern
        self.pat = load_pattern(B.PATTERN)
        self.cfg, self.law_cfg, pl = build_configs(dict(params or B.PARAMS), self.pat)
        self.ctrl = Controller(self.cfg, self.pat, self.law_cfg, pl, mode="closed")

    def push_packet(self, t, ipoc, xyz):
        self.ctrl.push_state(t, ipoc, xyz)

    def push(self, r):
        self.ctrl.push_rows(r.src, r.leg, r.s_tip, r.d, r.s, r.w, r.t)

    def update(self, t):
        self.ctrl.update(t)

    def command(self):
        self.ctrl.command()

    def published(self):
        return self.ctrl.published_ov(), self.ctrl.out_src

    def row(self, now):
        return self.ctrl.log_row(now)

    @property
    def switches(self):
        return self.ctrl.switches


def run(ctl, steps, pk, ros):
    """Apply the schedule (module docstring). Returns (update rows, step (t, ov, out_src) list)."""
    pt, pi, px = pk
    ip = ir = 0
    rows, out = [], []
    for t, is_upd in steps:
        while True:
            t_p = pt[ip] if ip < len(pt) else math.inf
            t_r = ros[ir].t if ir < len(ros) else math.inf
            if min(t_p, t_r) > t:
                break
            if t_p <= t_r:
                ctl.push_packet(float(t_p), float(pi[ip]), px[ip])
                ip += 1
            else:
                ctl.push(ros[ir])
                ir += 1
        if is_upd:
            ctl.update(t)
            rows.append(ctl.row(t))
        else:
            ctl.command()
        out.append((t, *ctl.published()))
    return rows, out


ADVANCE_MAX_S = 0.003       # a bore push logged at most this long after the update stamp may have been read by it


def _differs(row, live, k, u_tol):
    return (row["ov_pub"] != int(live["ov_pub"][k]) or row["n_bins"] != int(live["n_bins"][k])
            or abs(row["u"] - float(live["u"][k])) > u_tol)


def explain_live(live, pk, ros, u_tol=6e-5):
    """Legacy replay at the logged update times, compared with the live ctrl log (`live`: DataFrame) at every update.
    Returns (rows, advanced, unexplained): `advanced` = {update index: (source, push time - update time)} where the
    live log is reproduced only with the next bore push applied before the update."""
    L = Legacy()
    tu = live["t_perf"].to_numpy(float)
    pt, pi, px = pk
    ip = ir = 0
    rows, advanced, unexplained = [], {}, []
    skip = set()
    for k, t in enumerate(tu):
        while True:
            t_p = pt[ip] if ip < len(pt) else math.inf
            while ir < len(ros) and ir in skip:
                ir += 1
            t_r = ros[ir].t if ir < len(ros) else math.inf
            if min(t_p, t_r) > t:
                break
            if t_p <= t_r:
                L.push_packet(float(t_p), float(pi[ip]), px[ip])
                ip += 1
            else:
                L.push(ros[ir])
                ir += 1
        snap = copy.deepcopy((L.ctrl, L.ring))
        L.update(t)
        row = L.row(t)
        if _differs(row, live, k, u_tol):
            nxt = next((j for j in range(ir, len(ros)) if j not in skip and ros[j].src in ("bore1", "bore2")), None)
            ok = False
            if nxt is not None and ros[nxt].t - t <= ADVANCE_MAX_S:
                L.ctrl, L.ring = copy.deepcopy(snap)
                L.push(ros[nxt])
                L.update(t)
                row2 = L.row(t)
                if not _differs(row2, live, k, u_tol):
                    ok, row = True, row2
                    advanced[k] = (ros[nxt].src, ros[nxt].t - t)
                    skip.add(nxt)
                else:
                    L.ctrl, L.ring = snap
                    L.update(t)
                    row = L.row(t)
            if not ok:
                unexplained.append(k)
        rows.append(row)
    return rows, advanced, unexplained, L
