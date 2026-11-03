"""Offline replay of a logged session through the bore1 row logic (no ROS; pandas and numpy).

Inputs (the files of a legacy session folder):
- ``raw_video/borescope_frames_<ses>.csv(.gz)``: the frame times (``pc_ts_perf`` by ``frame_idx``);
- ``raw_video/bore1_rt_<ses>.csv(.gz)``: which frames the online thread processed, and when (the schedule), and the
  head outputs for the ``replay`` head;
- ``roboter/rsi_data_<ses>.csv(.gz)``: the RSI packets (``pc_ts_perf``, RIst xyz).

Schedule. The online thread took the newest frame, waited for an RSI packet at or after its time, gated it on the ring
and ran the head. The ring was filled from the RSI process through a queue, so what it held at the gate is not logged.
The replay gates frame k with exactly the packets received (``pc_ts_perf``) at or before the gate time
``t_gate = t_done - ms / 1000`` of its logged row (``ms`` = 0 for a rejected frame): the end of the row minus the head
time. That is the ring without the queue lag. On E1 it reproduces the logged gate columns of 12676 of the 12878
frames; on 184 of the others the online ring had not drained the newest packet yet (received less than 2.2 ms before
the gate), and 18 differ by one unit of the last logged decimal (rounding of the 1 us frame times).
``test/test_golden.py`` checks these numbers.
"""
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterator, List, Optional

import numpy as np


@dataclass
class Step:
    frame: int          # the camera's frame index (legacy column frame)
    t_frame: float      # frame time (PC receive time, perf clock)
    t_gate: float       # the packets received at or before this time are in the ring at the gate


def load_rsi(path):
    """(pc_ts_perf, ipoc, RIst xyz) arrays of an rsi_data csv(.gz)."""
    import pandas as pd
    r = pd.read_csv(path)
    return (r["pc_ts_perf"].to_numpy(float), r["ipoc"].to_numpy(float),
            r[["rist_x_mm", "rist_y_mm", "rist_z_mm"]].to_numpy(float))


def frame_times(frames_csv) -> dict:
    """{frame_idx: pc_ts_perf} of a borescope_frames csv(.gz)."""
    import pandas as pd
    f = pd.read_csv(frames_csv)
    return dict(zip(f["frame_idx"].astype(int), f["pc_ts_perf"].astype(float)))


def schedule(bore1_rt_csv, frames_csv=None) -> List[Step]:
    """One Step per logged bore1_rt row, in log order. Frame times from ``frames_csv`` when given (they equal the
    logged t_frame on E1), else the logged t_frame."""
    import pandas as pd
    b = pd.read_csv(bore1_rt_csv, usecols=["frame", "t_frame", "t_done", "ms"])
    ms = pd.to_numeric(b["ms"], errors="coerce").fillna(0.0).to_numpy(float)
    t_gate = b["t_done"].to_numpy(float) - ms / 1e3
    tf = b["t_frame"].to_numpy(float)
    if frames_csv is not None:
        ft = frame_times(frames_csv)
        tf = np.array([ft[int(i)] for i in b["frame"]], float)
    return [Step(int(i), float(t), float(g)) for i, t, g in zip(b["frame"], tf, t_gate)]


def replay(readout, ring_push: Callable, ring, steps: List[Step], pk_t: np.ndarray, pk_ipoc: np.ndarray,
           pk_xyz: np.ndarray, frame_of: Optional[Callable] = None) -> Iterator:
    """Yield ``readout.process`` for every step, with the packets up to the step's t_gate pushed first.
    ``ring_push(t, ipoc, xyz)``; ``frame_of(step)`` gives the image when the head needs one (None otherwise).
    t_done of a replayed row is the step's t_gate."""
    ptr = 0
    for st in steps:
        j = int(np.searchsorted(pk_t, st.t_gate, side="right"))
        for k in range(ptr, j):
            ring_push(pk_t[k], pk_ipoc[k], pk_xyz[k])
        ptr = max(ptr, j)
        frame = (lambda s=st: frame_of(s)) if frame_of is not None else None
        yield readout.process(st.t_frame, ring, st.frame, frame, clock=lambda s=st: s.t_gate)


def session_files(session_dir, ses: Optional[str] = None) -> dict:
    """The three input files of a legacy session folder (``.../e1_<ses>`` or any folder ending in the session name)."""
    import re
    d = Path(session_dir)
    if ses is None:
        m = re.search(r"(\d{8}_\d{6})$", d.name)
        if not m:
            raise ValueError(f"{d}: give the session name (the folder name does not end in yyyymmdd_hhmmss)")
        ses = m.group(1)

    def find(sub, stem):
        for ext in (".csv.gz", ".csv"):
            p = d / sub / f"{stem}_{ses}{ext}"
            if p.exists():
                return p
        raise FileNotFoundError(d / sub / f"{stem}_{ses}.csv(.gz)")
    return dict(ses=ses, bore1_rt=find("raw_video", "bore1_rt"), frames=find("raw_video", "borescope_frames"),
                rsi=find("roboter", "rsi_data"))
