"""Offline replay of a recorded session through rt_readout + rt_controller (shadow mode, causal).

ELP frames are iterated in pc_ts_perf order; before each frame only the RSI rows with pc_ts_perf <= the frame time are
pushed into the RsiRing (and the controller's per-packet window logic), then gate -> decode -> readout -> controller
step, exactly the calls the live threads make. No hardware, no network (the optional --loopback test talks to a
listener on 127.0.0.1 only).

Reports (written to --out as smoke_<tag>.json / .md, trace_<tag>.csv, ov_trace_<tag>.png):
  (a) per (frame, d) match against the offline r1pass_rig rows (int_<session>): r, slope, median |diff|, fraction matched
  (b) per-frame compute time: decode, gate, readout, controller; share of the 11.7 ms frame interval
  (c) shadow OvCmd trace and the fraction of steady-state time at the clamps

python replay_smoke.py --data <Y4F root with data/> --session 20261017_185445 --root <Y4F root> \
    --rig <rig json> --pattern <pattern json> --int-dir <int_<session> folder> --segment 3 --out <dir>
"""
import argparse
import json
import socket
import sys
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(1, str(HERE / "bore1"))   # v3 (line C 261021): the controller uses bore1_rt.RowStore
from rsi_ring import RsiRing  # noqa: E402
from rt_controller import Controller, CtrlCfg  # noqa: E402
from rt_readout import ROW_COLS, RtReadout  # noqa: E402

FRAME_INTERVAL_MS = 11.7
STEADY_FROM_LEG = 4         # print order >= 3 (cl_replay "steady"), pattern leg numbers start at 1


class Reader:
    """r1pass_rig.Reader: sequential decode with short grab-skips, seek for long jumps."""

    def __init__(self, fr, vid):
        self.vid = vid
        self.loc = {int(i): (f, int(k)) for i, f, k in zip(fr.frame_idx, fr.video_file, fr.frame_in_segment)}
        self.file, self.cap, self.pos = None, None, 0

    def get(self, idx):
        if idx not in self.loc:
            return None
        f, k = self.loc[idx]
        if f != self.file:
            if self.cap is not None:
                self.cap.release()
            self.cap, self.file, self.pos = cv2.VideoCapture(str(self.vid / f)), f, 1
        if k != self.pos:
            if k < self.pos or k - self.pos > 8:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, k - 1)
                self.pos = k
            while self.pos < k:
                self.cap.grab()
                self.pos += 1
        ok, im = self.cap.read()
        self.pos += 1
        return im if ok else None


def stats_ms(x):
    x = np.asarray(x, float) * 1e3
    if not len(x):
        return dict(n=0)
    return dict(n=int(len(x)), mean=float(x.mean()), p50=float(np.percentile(x, 50)),
                p95=float(np.percentile(x, 95)), p99=float(np.percentile(x, 99)))


def load_offline(int_dir, files):
    dfs = []
    for vf in files:
        f = Path(int_dir) / f"r1_frames_{vf.rsplit('.', 1)[0]}.csv"
        if not f.exists():
            continue
        try:
            d = pd.read_csv(f)
        except pd.errors.EmptyDataError:
            continue
        dfs.append(d)
    return pd.concat(dfs, ignore_index=True) if dfs else pd.DataFrame(columns=["frame", "d", "w"])


def compare(mine, off):
    """(frame, d) match on rows d 12..25 with a valid offline width."""
    off = off.assign(d=pd.to_numeric(off.d, errors="coerce"), w=pd.to_numeric(off.w, errors="coerce"),
                     frame=pd.to_numeric(off.frame, errors="coerce"))   # concat of empty / partial csv -> object dtype
    mine = mine.assign(w=pd.to_numeric(mine.w, errors="coerce"))
    off = off[(off.d >= 12) & (off.d < 25)].copy()
    off["d"] = off.d.round(2)
    mine = mine.copy()
    mine["d"] = mine.d.round(2)
    m = off[["frame", "d", "w"]].merge(mine[["frame", "d", "w"]], on=["frame", "d"], how="left", suffixes=("_off", "_rt"))
    vo = m.w_off.notna()
    both = vo & m.w_rt.notna()
    res = dict(rows_offline_valid=int(vo.sum()), rows_matched=int(both.sum()),
               frac_matched=float(both.sum() / max(vo.sum(), 1)),
               frames_offline=int(off.frame.nunique()), frames_rt=int(mine.frame.nunique()),
               frames_offline_not_rt=int(len(set(off.frame) - set(mine.frame))),
               frames_rt_not_offline=int(len(set(mine.frame) - set(off.frame))))
    if both.sum() > 2:
        a, b = m.w_off[both].to_numpy(), m.w_rt[both].to_numpy()
        res.update(r=float(np.corrcoef(a, b)[0, 1]), slope=float(np.polyfit(a, b, 1)[0]),
                   median_abs_diff_mm=float(np.median(np.abs(a - b))), max_abs_diff_mm=float(np.max(np.abs(a - b))))
    rt_valid_off_nan = int((m.w_off.isna() & m.w_rt.notna()).sum())
    res["rows_valid_rt_nan_offline"] = rt_valid_off_nan
    return res


def plot_trace(tr, path, tag):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    ax[0].plot(tr.t_rel, tr.w_est, ".", ms=2)
    ax[0].set_ylabel("width estimate (window, mm)")
    ax[0].set_title(f"shadow replay {tag}")
    ax[1].step(tr.t_rel, tr.ov, where="post")
    ax[1].set_ylabel("OvCmd (%)")
    ax[1].set_xlabel("time since print window opened (s)")
    ax[1].set_ylim(45, 105)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def loopback_test(pipeline_dir, port=59353):
    """Listener on 127.0.0.1 only: zero reply == original build_zero_reply bytes, ov reply has the OvCmd tag."""
    sys.path.insert(1, str(pipeline_dir))
    from rsi_listener import RsiListener, build_zero_reply
    from rsi_ov_listener import RsiOvListener, build_ov_reply
    tmp = Path(tempfile.mkdtemp(prefix="rt_loop_"))
    xml = (b'<Rob Type="KUKA"><RIst X="945.0" Y="-530.0" Z="1.5" A="0" B="90" C="0"/>'
           b'<RSol X="945.0" Y="-530.0" Z="1.5" A="0" B="90" C="0"/><Delay D="0"/><IPOC>1507778</IPOC></Rob>')
    out = {}
    for reply, expect in (("zero", build_zero_reply("1507778")), ("ov", build_ov_reply("1507778", 80))):
        lst = RsiOvListener(str(tmp / f"rsi_{reply}.csv"), RsiRing(), str(tmp / f"ov_{reply}.csv"),
                            provider=lambda t, xyz: (80, "control"), reply=reply, host="127.0.0.1", port=port)
        lst.start()
        time.sleep(0.3)
        c = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        c.settimeout(2.0)
        c.sendto(xml, ("127.0.0.1", port))
        data, _ = c.recvfrom(4096)
        c.close()
        lst.stop()
        lst.join(3.0)
        head = (tmp / f"rsi_{reply}.csv").read_text(encoding="utf-8").splitlines()[0]
        out[reply] = dict(reply_ok=data == expect, reply=data.decode(), header_ok=head.split(",") == RsiListener.HEADER)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="folder holding data/raw_video/<ses> and data/roboter/<ses>")
    ap.add_argument("--session", required=True)
    ap.add_argument("--rig", required=True)
    ap.add_argument("--pattern", required=True)
    ap.add_argument("--root", default=None, help="base for relative paths inside the rig / pattern json")
    ap.add_argument("--int-dir", default=None, help="offline r1pass_rig rows (int_<session>) to compare with")
    ap.add_argument("--segment", type=int, nargs="*", default=None, help="video segment index(es); default all")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--tag", default=None)
    ap.add_argument("--loopback", default=None, metavar="PIPELINE_DIR", help="also run the 127.0.0.1 listener test")
    a = ap.parse_args()

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    tag = a.tag or (a.session + ("_seg" + "-".join(map(str, a.segment)) if a.segment else "_full"))
    vid = Path(a.data) / "data" / "raw_video" / a.session
    fr = pd.read_csv(vid / f"elp_frames_{a.session}.csv").sort_values("pc_ts_perf").reset_index(drop=True)
    sel = fr[fr.segment_idx.isin(a.segment)] if a.segment else fr
    if a.max_frames:
        sel = sel.iloc[:a.max_frames]
    files = sorted(sel.video_file.unique())
    rs = pd.read_csv(Path(a.data) / "data" / "roboter" / a.session / f"rsi_data_{a.session}.csv")
    tr_t, tr_ip = rs.pc_ts_perf.to_numpy(), rs.ipoc.to_numpy().astype(float)
    tr_P = rs[["rist_x_mm", "rist_y_mm", "rist_z_mm"]].to_numpy()

    cfg = CtrlCfg()
    ro = RtReadout(a.rig, a.pattern, a.root)
    ring = RsiRing(tau_v=cfg.tau_v)
    from rt_law_v3 import LawCfg, plateaus_from_pattern_json   # v3: plan A defaults, plateaus from the pattern
    ctrl = Controller(cfg, ro.pat, "shadow", LawCfg(plan="A"), plateaus_from_pattern_json(a.pattern, a.root))
    reader = Reader(fr, vid)

    idxs, tfs = sel.frame_idx.to_numpy(), sel.pc_ts_perf.to_numpy()
    T = {k: [] for k in ("decode", "gate", "readout", "push", "step", "all")}
    t_pk, n_pk, ptr, n_missing = 0.0, 0, 0, 0
    mine_rows, trace = [], []
    skips = {}
    t_wall0 = time.time()
    pc = time.perf_counter
    for i in range(len(sel)):
        tf, idx = tfs[i], int(idxs[i])
        j = int(np.searchsorted(tr_t, tf, side="right"))
        t0 = pc()
        for k in range(ptr, j):
            ring.push(tr_t[k], tr_ip[k], tr_P[k])
            ctrl.on_packet(tr_t[k], tr_P[k])
        t_pk += pc() - t0
        n_pk += j - ptr
        ptr = j
        t0 = pc()
        g = ro.gate(tf, ring)
        t_gate = pc() - t0
        t_dec = t_rd = t_push = 0.0
        if g is None:
            skips[ro.last_skip] = skips.get(ro.last_skip, 0) + 1
        else:
            t0 = pc()
            im = reader.get(idx)
            t_dec = pc() - t0
            if im is None:
                n_missing += 1
            else:
                res = ro.read(im, g, tf, idx)
                t_rd = res.compute_s
                t0 = pc()
                ctrl.push_rows(res, tf)
                t_push = pc() - t0
                mine_rows.append(np.column_stack([np.full(len(res.rows), idx), np.full(len(res.rows), res.leg),
                                                  np.full(len(res.rows), res.s_tip), res.rows]))
                T["decode"].append(t_dec)
                T["readout"].append(t_rd)
        t0 = pc()
        ctrl.step(tf, ring)
        t_step = pc() - t0
        T["gate"].append(t_gate)
        T["push"].append(t_push)
        T["step"].append(t_step)
        T["all"].append(t_gate + t_rd + t_push + t_step)
        ov, src = ctrl.last_reply
        t_start = ctrl.win.t_start
        trace.append((tf, tf - t_start if t_start is not None else np.nan, idx, ctrl.win.state, ctrl.info["leg"],
                      ctrl.info["on_a"], ctrl.info["s_tip"], ctrl.info["rhat"], ctrl.info["w_est"], ov, src))
    wall = time.time() - t_wall0

    tr_df = pd.DataFrame(trace, columns=["t_perf", "t_rel", "frame", "win", "leg", "on_a", "s_tip", "rhat", "w_est",
                                         "ov", "source"])
    tr_df.to_csv(out / f"trace_{tag}.csv", index=False, float_format="%.4f")
    M = np.vstack(mine_rows) if mine_rows else np.zeros((0, 3 + len(ROW_COLS)))
    mine = pd.DataFrame(M, columns=["frame", "leg", "s_tip", *ROW_COLS])
    mine["frame"] = mine.frame.astype(int)

    S = dict(tag=tag, session=a.session, files=files, frames=int(len(sel)), frames_readout=int(len(T["readout"])),
             frames_gated_out=skips, frames_missing_image=n_missing, wall_s=wall,
             machine_note="timings measured on the desktop (shared pinned cores), not on the laptop")
    if a.int_dir:
        S["match"] = compare(mine, load_offline(a.int_dir, files))
    S["time_ms"] = {k: stats_ms(v) for k, v in T.items()}
    allm = np.mean(T["all"]) * 1e3 if T["all"] else float("nan")
    rd_frames = np.array(T["readout"]) + np.array(T["decode"])
    S["time_share"] = dict(
        mean_per_frame_all_ms=float(allm), share_of_11p7ms_all_frames=float(allm / FRAME_INTERVAL_MS),
        readout_frames_total_ms_p50=float(np.percentile(rd_frames * 1e3 + 0, 50)) if len(rd_frames) else None,
        readout_frames_total_ms_p95=float(np.percentile(rd_frames * 1e3, 95)) if len(rd_frames) else None,
        rsi_push_per_packet_us=float(1e6 * t_pk / max(n_pk, 1)), packets=int(n_pk))
    inside = tr_df[tr_df.win == "inside"]
    steady = inside[(inside.leg >= STEADY_FROM_LEG) & inside.on_a & (inside.source == "control")]
    S["ov"] = dict(frames_inside=int(len(inside)), source_counts=inside.source.value_counts().to_dict(),
                   ov_min_seen=int(inside.ov.min()) if len(inside) else None,
                   ov_max_seen=int(inside.ov.max()) if len(inside) else None,
                   steady_frames=int(len(steady)),
                   steady_frac_at_clamp=float(((steady.ov <= cfg.ov_min) | (steady.ov >= cfg.ov_max)).mean())
                   if len(steady) else None,
                   steady_ov_p5_p50_p95=[float(np.percentile(steady.ov, q)) for q in (5, 50, 95)] if len(steady) else None,
                   note="fraction of frames (uniform in time) on -X legs 4.. with a control estimate")
    if len(inside):
        plot_trace(inside, out / f"ov_trace_{tag}.png", tag)
    if a.loopback:
        S["loopback"] = loopback_test(a.loopback)
    (out / f"smoke_{tag}.json").write_text(json.dumps(S, indent=2, default=str), encoding="utf-8")
    print(json.dumps(S, indent=2, default=str))


if __name__ == "__main__":
    main()
