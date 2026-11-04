"""The two 2026-11-03 sessions for the gate tests: RSI packets, the logged frames of every camera, the legacy gates,
and a sweep that evaluates gates on the RSI ring as it was after the first k packets.

Ring: the legacy ``rsi_ring.RsiRing`` filled from ``rsi_data`` (``pc_ts_perf``, IPOC, RIst x, y, z) in packet order.
A query (k, key) sees the ring after the first k packets; the queries are sorted by k and the packets pushed once, so
one pass serves every frame and every cut. A cut at time c means the packets with ``pc_ts_perf <= c``.
"""
import json
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pandas as pd

from legacy_paths import (ARC2, ARC2_JSON, ARC2_SES, FAN1, FAN1_JSON, FAN1_SES, LEGACY_DIR, LEGACY_PATTERNS,
                          import_legacy)

SESSIONS = {"fan1": (FAN1, FAN1_SES, FAN1_JSON, "fan_geom"), "arc2": (ARC2, ARC2_SES, ARC2_JSON, "semi_geom")}
CAMS = {"bore1": ("A", "raw_video/bore1_rt", "t_frame", "t_done"),
        "bore2": ("B", "raw_video/bore2_rt", "t_frame", "t_done"),
        "elpA": ("A", "roboter/readout", "t_frame_perf", "t_done_perf"),
        "elpB": ("B", "roboter/readoutB", "t_frame_perf", "t_done_perf")}
RIGS = {"A": LEGACY_DIR / "rig" / "cage2opt_elpA" / "rig_cage2opt_elpA_261102_m2.json",
        "B": LEGACY_DIR / "rig" / "cage2opt_elpB" / "rig_cage2opt_elpB_261102.json"}
BORES, ELPS = ("bore1", "bore2"), ("elpA", "elpB")


def available() -> bool:
    return all((d / "roboter" / f"rsi_data_{s}.csv.gz").exists() for d, s, _, _ in SESSIONS.values())


def lat(kind) -> float:
    return float(json.loads(RIGS[kind].read_text(encoding="utf-8"))["lat_s"])


@dataclass
class Session:
    name: str
    ses: str
    T: np.ndarray       # pc_ts_perf
    IP: np.ndarray      # ipoc
    P: np.ndarray       # RIst xyz
    pattern_json: Path  # the port's copy (path_geometry/patterns)
    legacy_mod: object  # fan_geom or semi_geom
    legacy_pat: object
    frames: dict        # cam -> DataFrame of the log

    def k_at(self, t) -> np.ndarray:
        """number of packets with pc_ts_perf <= t"""
        return np.searchsorted(self.T, np.asarray(t, float), side="right")

    def sweep(self, queries, fn):
        """{key: fn(ring, key)} for queries (k, key), each on the legacy ring after the first k packets"""
        rsi_ring = import_legacy("rsi_ring")
        ring, n, out = rsi_ring.RsiRing(), 0, {}
        for k, key in sorted(queries, key=lambda q: q[0]):
            while n < k:
                ring.push(self.T[n], self.IP[n], self.P[n])
                n += 1
            out[key] = fn(ring, key)
        return out


@lru_cache(maxsize=None)
def session(name) -> Session:
    from path_geometry.geometry import patterns_dir
    d, ses, js, mod = SESSIONS[name]
    rsi = pd.read_csv(d / "roboter" / f"rsi_data_{ses}.csv.gz")
    M = import_legacy(mod)
    cls = M.SemiPattern if mod == "semi_geom" else M.FanPattern
    frames = {}
    for cam, (_, f, _, _) in CAMS.items():
        frames[cam] = pd.read_csv(d / f"{f}_{ses}.csv.gz", keep_default_na=False, na_values=[""],
                                  dtype={"gate": str})
    return Session(name, ses, rsi.pc_ts_perf.to_numpy(float), rsi.ipoc.to_numpy(float),
                   rsi[["rist_x_mm", "rist_y_mm", "rist_z_mm"]].to_numpy(float), patterns_dir() / js, M,
                   cls(LEGACY_PATTERNS / js), frames)


@lru_cache(maxsize=None)
def port_pattern(name):
    from path_geometry.geometry import load_pattern
    return load_pattern(session(name).pattern_json)


# ------------------------------------------------------------------------------------------------ legacy gates
def legacy_bore_gate(S: Session, kind, max_deg=90.0):
    """make_fan_bore_gate of the session's legacy module (fan_geom / semi_geom), as run_cl.py installs it"""
    return S.legacy_mod.make_fan_bore_gate(kind, max_deg)


def legacy_elp(S: Session, kind):
    """the legacy FanReadout / FanReadoutB (fan_geom: fan, semi_geom: semi) gate without its image half: the gate
    reads only rig.lat and pat"""
    cls = S.legacy_mod.FanReadout if kind == "A" else S.legacy_mod.FanReadoutB
    obj = cls.__new__(cls)
    obj.rig, obj.pat, obj.last_skip = SimpleNamespace(lat=lat(kind)), S.legacy_pat, ""
    return obj


def legacy_e(S: Session, kind, tip) -> float:
    return float(S.legacy_pat.lines.locate(tip, (kind,)).e)


def frame_times(S: Session, cam):
    _, _, tf, td = CAMS[cam]
    F = S.frames[cam]
    return F[tf].to_numpy(float), F[td].to_numpy(float)


def live(S: Session, cam):
    """(gated, reason or '', leg or -1, s_tip or NaN) per logged frame"""
    F = S.frames[cam]
    if cam in BORES:
        why = F.gate.to_numpy(str)
        gated = why == "ok"
    else:
        gated = F.leg.notna().to_numpy()
        why = np.where(gated, "ok", "")
    leg = np.where(gated, F.leg.fillna(-1).to_numpy(float), -1).astype(int)
    return gated, why, leg, np.where(gated, F.s_tip.to_numpy(float), np.nan)
