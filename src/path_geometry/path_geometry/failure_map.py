"""Angle failure map of one print: bore1, bore2, ELP-A and ELP-B readings placed at the local tangent angle where
the camera looked. Plain Python (numpy, pandas), no ROS.

Port of ``legacy/realtime_cl_cage2_semi_261103/semi_failure_map_261103.py`` on :mod:`path_geometry.geometry` (both
the semi and the fan geometry). The three per-reading tables are the legacy ones (same columns and rows):

- ``failuremap_bore_<ses>.csv``: one row per gated (``gate == ok``) bore frame, geometry at ``s_tip`` (``tip_*``)
  and at ``s_dep`` = s_tip - 5 (``obs_*``);
- ``failuremap_elp_<ses>.csv``: one row per stored ELP row, geometry at the row's ``s_dep`` (``obs_*``);
- ``failuremap_elpframes_<ses>.csv``: one row per gated ELP frame, geometry at ``s_tip`` (``tip_*``).

The summary ``failuremap_summary_<ses>.csv`` has a column ``basis``:

- ``obs``, bores: as legacy (bin ``obs_delta``; group by cam, bin, ``obs_curv``, ``obs_R``; ``n``, ``bore_pushed``,
  ``bore_sd_med``, ``w_med``);
- ``obs``, ELPs: every stored ELP row binned by its own ``obs_delta``; group by cam, bin, ``obs_curv``, ``obs_R``;
  ``n`` = stored rows, ``n_frames`` = distinct frames, ``w_med``, ``w_q25``, ``w_q75``. (The legacy summary binned the
  ELP by the angle at the nozzle tip, 12-25 mm ahead of where the ELP reads: known issue 1 of the legacy README.)
- ``tip_frames``, ELPs: the legacy frame-availability rows (``elp_frames_with_rows``, ``elp_rows_med``, ``w_med``,
  binned by ``tip_delta``), kept for comparison only. A frame has no single observed angle.
"""
from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd

from .geometry import Pattern

FAM = {"bore1": "A", "bore2": "B", "elpA": "A", "elpB": "B"}
BORES = (("bore1", "raw_video", "bore1_rt_{ses}"), ("bore2", "raw_video", "bore2_rt_{ses}"))
ELPS = (("elpA", ""), ("elpB", "B"))
BIN = 15.0

SUMMARY_COLS = ["cam", "basis", "delta_lo", "delta_hi", "curv", "R", "n", "n_frames", "bore_pushed", "bore_sd_med",
                "w_med", "w_q25", "w_q75", "elp_frames_with_rows", "elp_rows_med"]


def geo_cols(pat: Pattern, kind: str, legs, s, prefix: str):
    """line name and ``<prefix>delta / curv / R / lobe`` per (leg, s); NaN / 0 where leg or s is missing"""
    out = {f"{prefix}delta": np.full(len(s), np.nan), f"{prefix}curv": np.zeros(len(s), int),
           f"{prefix}R": np.full(len(s), np.nan), f"{prefix}lobe": np.zeros(len(s), int)}
    row = np.empty(len(s), object)
    for i, (lg, si) in enumerate(zip(legs, s)):
        ln = pat.lines.get(kind, int(lg)) if np.isfinite(lg) else None
        if ln is None or not np.isfinite(si):
            continue
        row[i] = ln.name
        out[f"{prefix}delta"][i] = ln.delta_at(si)
        out[f"{prefix}curv"][i] = ln.curv_at(si)
        out[f"{prefix}R"][i] = ln.R_at(si)
        out[f"{prefix}lobe"][i] = ln.lobe_at(si)
    return row, out


# ------------------------------------------------------------------------------------------------ session files
def find_csv(folder: Path, stem: str) -> Optional[Path]:
    """``<stem>.csv`` or ``<stem>.csv.gz`` in folder, else None"""
    for ext in (".csv", ".csv.gz"):
        p = Path(folder) / (stem + ext)
        if p.exists():
            return p
    return None


class Session:
    """The logs of one print in the layout of ``data/``: ``<dir>/raw_video/`` and ``<dir>/roboter/`` (the legacy
    layout ``<dir>/raw_video/<ses>/`` and ``<dir>/roboter/<ses>/`` is accepted too), ``.csv`` or ``.csv.gz``.
    ``session_meta.json`` is read from ``raw_video/`` (or ``raw_video/<ses>/``), else from ``<dir>`` itself (the
    print_session bringup's session folder)."""

    def __init__(self, root, ses: Optional[str] = None):
        self.root = Path(root)
        meta = self._meta_path(ses)
        self.meta = None
        self.meta_path = meta
        if meta is not None:
            import json
            self.meta = json.loads(meta.read_text(encoding="utf-8"))
        if ses is None:     # legacy run_cl: session_ts; print_session's session node: print_session.session
            ses = (self.meta or {}).get("session_ts") or ((self.meta or {}).get("print_session") or {}).get("session")
        if ses is None:
            raise ValueError(f"{self.root}: no session_meta.json with session_ts; give the session id")
        self.ses = str(ses)
        self.vdir = self._dir("raw_video")
        self.rdir = self._dir("roboter")

    def _meta_path(self, ses):
        # raw_video/ (the data/ and exported layouts), else the session root (where the print_session bringup writes it)
        cands = [self.root / "raw_video" / "session_meta.json"]
        if ses is not None:
            cands.append(self.root / "raw_video" / str(ses) / "session_meta.json")
        cands += sorted((self.root / "raw_video").glob("*/session_meta.json"))
        cands.append(self.root / "session_meta.json")
        for c in cands:
            if c.exists():
                return c
        return None

    def _dir(self, sub):
        nested = self.root / sub / self.ses
        return nested if nested.is_dir() else self.root / sub

    def csv(self, sub: str, stem: str) -> Optional[Path]:
        return find_csv(self.vdir if sub == "raw_video" else self.rdir, stem.format(ses=self.ses))

    def pattern_name(self) -> str:
        try:
            return str(self.meta["realtime"]["pattern"])
        except (TypeError, KeyError) as exc:
            raise ValueError(f"{self.root}: session_meta.json has no realtime.pattern; give --pattern") from exc


# ------------------------------------------------------------------------------------------------ tables
def bores(pat: Pattern, sess: Session) -> pd.DataFrame:
    frames = []
    for cam, sub, stem in BORES:
        f = sess.csv(sub, stem)
        if f is None:
            continue
        b = pd.read_csv(f)
        b = b[b.gate == "ok"].copy()
        if b.empty:
            continue
        b.insert(0, "cam", cam)
        row, g_tip = geo_cols(pat, FAM[cam], b.leg.to_numpy(float), b.s_tip.to_numpy(float), "tip_")
        _, g_obs = geo_cols(pat, FAM[cam], b.leg.to_numpy(float), b.s_dep.to_numpy(float), "obs_")
        b["row"] = row
        for k, v in {**g_tip, **g_obs}.items():
            b[k] = v
        frames.append(b)
    return pd.concat(frames, ignore_index=True) if frames else pd.DataFrame()


def elps(pat: Pattern, sess: Session) -> Tuple[pd.DataFrame, pd.DataFrame]:
    """(ELP rows, ELP frames)"""
    rows_all, fr_all = [], []
    for cam, tag in ELPS:
        ff, fr = sess.csv("roboter", f"readout{tag}_{{ses}}"), sess.csv("roboter", f"readout{tag}_rows_{{ses}}")
        if ff is None:
            continue
        F = pd.read_csv(ff)
        F = F[F.leg.notna()].copy()
        F.insert(0, "cam", cam)
        row, g = geo_cols(pat, FAM[cam], F.leg.to_numpy(float), F.s_tip.to_numpy(float), "tip_")
        F["row"] = row
        for k, v in g.items():
            F[k] = v
        fr_all.append(F)
        if fr is not None:
            Rw = pd.read_csv(fr).merge(F[["frame", "leg", "s_tip", "t_frame_perf"]], on="frame", how="left")
            Rw.insert(0, "cam", cam)
            row, g = geo_cols(pat, FAM[cam], Rw.leg.to_numpy(float), Rw.s_dep.to_numpy(float), "obs_")
            Rw["row"] = row
            for k, v in g.items():
                Rw[k] = v
            rows_all.append(Rw)

    def cat(L):
        return pd.concat(L, ignore_index=True) if L else pd.DataFrame()
    return cat(rows_all), cat(fr_all)


def binned(d, width: float = BIN):
    """lower edge of the angle bin of d (deg): bins of ``width`` from -90, the outer bins open to +-90 and beyond"""
    b = np.floor((np.asarray(d, float) + 90.0 + 1e-9) / width) * width - 90.0
    return np.clip(b, -90.0, 90.0 - width)


def summary_bore_obs(B: pd.DataFrame, width: float = BIN) -> pd.DataFrame:
    out = []
    if not B.empty:
        B = B[B.obs_delta.notna()].copy()
        B["bin"] = binned(B.obs_delta.to_numpy(), width)
        for (cam, b, c, R), g in B.groupby(["cam", "bin", "obs_curv", "obs_R"], dropna=False):
            out.append(dict(cam=cam, basis="obs", delta_lo=b, delta_hi=b + width, curv=c, R=R, n=len(g),
                            bore_pushed=float((g.pushed == 1).mean()) if "pushed" in g else np.nan,
                            bore_sd_med=float(g.sd.median()), w_med=float(g.w.median())))
    return pd.DataFrame(out)


def summary_elp_obs(E: pd.DataFrame, width: float = BIN) -> pd.DataFrame:
    """every stored ELP row binned by its own observed angle (``obs_delta`` at the row's ``s_dep``)"""
    out = []
    if not E.empty:
        E = E[E.obs_delta.notna()].copy()
        E["bin"] = binned(E.obs_delta.to_numpy(), width)
        for (cam, b, c, R), g in E.groupby(["cam", "bin", "obs_curv", "obs_R"], dropna=False):
            out.append(dict(cam=cam, basis="obs", delta_lo=b, delta_hi=b + width, curv=c, R=R, n=len(g),
                            n_frames=int(g.frame.nunique()), w_med=float(g.w.median()),
                            w_q25=float(g.w.quantile(0.25)), w_q75=float(g.w.quantile(0.75))))
    return pd.DataFrame(out)


def summary_elp_tip_frames(E: pd.DataFrame, F: pd.DataFrame, width: float = BIN) -> pd.DataFrame:
    """the legacy ELP summary rows: frames binned by the angle at the nozzle tip (for comparison only)"""
    out = []
    if not F.empty:
        F = F[F.tip_delta.notna()].copy()
        F["bin"] = binned(F.tip_delta.to_numpy(), width)
        for (cam, b, c, R), g in F.groupby(["cam", "bin", "tip_curv", "tip_R"], dropna=False):
            ew = E[(E.cam == cam) & E.frame.isin(g.frame)] if not E.empty else E
            out.append(dict(cam=cam, basis="tip_frames", delta_lo=b, delta_hi=b + width, curv=c, R=R, n=len(g),
                            elp_frames_with_rows=float((g.n_rows > 0).mean()),
                            elp_rows_med=float(g.n_rows.median()),
                            w_med=float(ew.w.median()) if len(ew) else np.nan))
    return pd.DataFrame(out)


def summary(B: pd.DataFrame, E: pd.DataFrame, F: pd.DataFrame, width: float = BIN) -> pd.DataFrame:
    parts = [p for p in (summary_bore_obs(B, width), summary_elp_obs(E, width), summary_elp_tip_frames(E, F, width))
             if not p.empty]
    S = pd.concat(parts, ignore_index=True) if parts else pd.DataFrame(columns=SUMMARY_COLS)
    for c in SUMMARY_COLS:
        if c not in S:
            S[c] = np.nan
    S = S[SUMMARY_COLS]
    S["n_frames"] = S["n_frames"].astype("Int64")
    return S


GATE_LOG_CAMERAS = ("bore1", "bore2", "elp", "elp_b")     # the camera parameter of the ROS readout nodes


def gate_log_files(sess: Session) -> List[Path]:
    """The gate logs of a session: the legacy ``roboter/semi_gate_<ses>.csv``, else the per-camera files
    ``semi_gate_<camera>.csv`` the ROS readout nodes write (``camera`` in GATE_LOG_CAMERAS; next to session_meta.json,
    or in roboter/ or raw_video/), each ``.csv`` or ``.csv.gz``."""
    legacy = sess.csv("roboter", "semi_gate_{ses}")
    if legacy is not None:
        return [legacy]
    dirs: List[Path] = []
    for d in (sess.root, sess.rdir, sess.vdir, sess.meta_path.parent if sess.meta_path else None):
        if d is not None and Path(d) not in dirs:
            dirs.append(Path(d))
    out = []
    for cam in GATE_LOG_CAMERAS:
        p = next((p for p in (find_csv(d, f"semi_gate_{cam}") for d in dirs) if p is not None), None)
        if p is not None:
            out.append(p)
    return out


def gate_log_check(B: pd.DataFrame, sess: Session) -> Optional[pd.DataFrame]:
    """bore rows joined on (cam, t) with the gate log (``gate_log_files``: the legacy ``semi_gate_<ses>.csv`` or the
    per-camera ``semi_gate_<camera>.csv``; None without a gate log)"""
    files = gate_log_files(sess)
    if not files or B.empty:
        return None
    logs = [pd.read_csv(f) for f in files]
    # a header-only log (a camera with no gated frame) would make every column of the concat dtype object
    G = pd.concat([g for g in logs if not g.empty] or logs[:1], ignore_index=True)
    return B.merge(G[G.cam.isin(["bore1", "bore2"])], left_on=["cam", "t_frame"], right_on=["cam", "t"], how="inner")


def failure_map(pat: Pattern, sess: Session, width: float = BIN) -> Dict[str, pd.DataFrame]:
    B = bores(pat, sess)
    E, F = elps(pat, sess)
    return {"bore": B, "elp": E, "elpframes": F, "summary": summary(B, E, F, width)}


def write_tables(tables: Dict[str, pd.DataFrame], out: Path, ses: str) -> Dict[str, Path]:
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    paths = {}
    for key in ("bore", "elp", "elpframes", "summary"):
        p = out / f"failuremap_{key}_{ses}.csv"
        tables[key].to_csv(p, index=False)
        paths[key] = p
    return paths
