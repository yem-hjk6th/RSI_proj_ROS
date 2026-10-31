"""Offline replay: run a readout on a folder of frames plus the RSI log, causally, as the online pipeline did.

For each frame (in frame order) the RSI packets received up to the frame time are pushed into the ring, then the
readout is called. Works with the port and, duck-typed, with the legacy ``rt_readout.RtReadout`` / ``RsiRing``.
No ROS imports.
"""
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

FRAME_FMT = "frame_{:06d}.jpg"


def load_rsi(path):
    """(t_perf, ipoc, xyz) arrays from an rsi_data csv(.gz)."""
    rs = pd.read_csv(path)
    return (rs.pc_ts_perf.to_numpy(), rs.ipoc.to_numpy().astype(float),
            rs[["rist_x_mm", "rist_y_mm", "rist_z_mm"]].to_numpy())


def frame_times(index_csv, frame_idx):
    """Frame time (pc_ts_perf) of each frame index, from an elp_frames csv(.gz)."""
    fr = pd.read_csv(index_csv)
    t = dict(zip(fr.frame_idx.astype(int), fr.pc_ts_perf.astype(float)))
    return np.array([t[int(i)] for i in frame_idx])


def frames_in(folder):
    """Sorted frame indices of the frame_NNNNNN.jpg files in folder."""
    return sorted(int(p.stem.split("_")[1]) for p in Path(folder).glob("frame_*.jpg"))


def replay(readout, ring_push, ring, frames_dir, index_csv, rsi_csv, frame_idx=None):
    """Yield (frame_idx, t_frame, readout result or None, skip reason) for each frame.

    ring_push(t, ipoc, xyz) pushes one packet into ring (so the port's and the legacy ring signatures both fit).
    """
    if frame_idx is None:
        frame_idx = frames_in(frames_dir)
    tf = frame_times(index_csv, frame_idx)
    order = np.argsort(tf, kind="stable")
    tr_t, tr_ip, tr_P = load_rsi(rsi_csv)
    ptr = 0
    for i in order:
        idx, t = int(frame_idx[i]), float(tf[i])
        j = int(np.searchsorted(tr_t, t, side="right"))
        for k in range(ptr, j):
            ring_push(tr_t[k], tr_ip[k], tr_P[k])
        ptr = j
        g = readout.gate(t, ring)
        if g is None:
            yield idx, t, None, readout.last_skip
            continue
        im = cv2.imread(str(Path(frames_dir) / FRAME_FMT.format(idx)), cv2.IMREAD_COLOR)
        if im is None:
            raise FileNotFoundError(f"frame {idx} not readable in {frames_dir}")
        yield idx, t, readout.read(im, g, t, idx), ""
