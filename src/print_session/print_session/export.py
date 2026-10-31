"""Recorded session messages -> the legacy CSV logs. No ROS imports: messages are duck-typed (attribute access).

Output layout (what legacy eval_v3_print.py --data <out> --session <ses> reads):
    <out>/roboter/<ses>/rsi_data_<ses>.csv, ov_cmd_<ses>.csv, ctrl_<ses>.csv, readout_<ses>.csv,
    readout_rows_<ses>.csv
    <out>/raw_video/<ses>/session_meta.json   (copied, when given)
Columns, order and number formats follow legacy_cols. Columns no topic carries are filled as legacy_cols.NOT_CARRIED
says.
"""
import csv
import shutil
import statistics
from collections import OrderedDict
from pathlib import Path
from typing import Dict, Iterable, List, Optional

from . import topics as T
from .legacy_cols import (CTRL_BOOL, CTRL_COLS, CTRL_ND, CTRL_TRI, OV_COLS, READOUT_COLS, ROWS_COLS, RSI_COLS, f3, f6,
                          fnd)


def stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _int_if_whole(v):
    v = float(v)
    return int(v) if v.is_integer() else v


def rsi_rows(states: Iterable, replies: Iterable) -> List[list]:
    """rsi_data rows: one per KrcState, joined with its ReplyRecord (same ipoc) for delay and pc_ts_perf."""
    rep = {int(r.ipoc): r for r in replies}
    out = []
    for s in states:
        r = rep.get(int(s.ipoc))
        perf = f6(r.pc_ts_perf) if r is not None else ""
        out.append([perf, f6(stamp_s(s.stamp)), int(s.ipoc), *(f3(v) for v in s.rist), "", "", "", "", "", "",
                    int(r.delay) if r is not None else "", int(s.override_pct), _int_if_whole(s.vel_act_mm_s),
                    int(s.rpm_ext), perf])
    return out


def ov_rows(replies: Iterable) -> List[list]:
    return [[f6(r.pc_ts_perf), int(r.ipoc), int(r.ov_cmd), r.source] for r in replies]


def ctrl_row(m) -> list:
    row = []
    for c in CTRL_COLS:
        if c == "t_wall":
            row.append(f6(stamp_s(m.stamp)))
        elif c == "t_perf":
            row.append(f6(m.t_perf))
        elif c in CTRL_ND:
            row.append(fnd(float(getattr(m, c)), CTRL_ND[c]))
        elif c in CTRL_BOOL:
            row.append(int(bool(getattr(m, c))))
        elif c in CTRL_TRI:
            v = int(getattr(m, c))
            row.append("" if v < 0 else v)
        else:
            row.append(getattr(m, c))
    return row


def wall_minus_perf(states: Iterable, replies: Iterable) -> Optional[float]:
    """Median of KrcState.stamp - ReplyRecord.pc_ts_perf over the packets that have both (None if none)."""
    perf = {int(r.ipoc): float(r.pc_ts_perf) for r in replies}
    d = [stamp_s(s.stamp) - perf[int(s.ipoc)] for s in states if int(s.ipoc) in perf]
    return statistics.median(d) if d else None


def readout_rows(widths: Iterable, offset: Optional[float]):
    """(readout rows, readout_rows rows) from the Width messages of one camera. One frame per distinct stamp
    (the readout publishes every row of a frame with the frame's stamp), numbered 1, 2, ... in stamp order."""
    frames: "OrderedDict[tuple, list]" = OrderedDict()
    for m in widths:
        frames.setdefault((m.stamp.sec, m.stamp.nanosec), []).append(m)
    ro, rows = [], []
    for i, key in enumerate(sorted(frames), start=1):
        ms = frames[key]
        used = [m for m in ms if m.valid]
        t = key[0] + key[1] * 1e-9
        w_med = statistics.median([float(m.width_mm) for m in used]) if used else None
        ro.append([i, f6(t - offset) if offset is not None else "", "", "", "", int(ms[0].leg), "", "", len(used),
                   fnd(w_med, 4)])
        rows.extend([i, round(float(m.d_mm), 2), round(float(m.s_mm), 3), round(float(m.width_mm), 4)] for m in used)
    return ro, rows


def _write(path: Path, cols, rows) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(cols)
        w.writerows(rows)


def export(msgs: Dict[str, list], out_dir, session: str, meta_path=None, elp_topic: str = T.WIDTH_ELP) -> dict:
    """Write the five legacy CSVs (and copy session_meta.json). `msgs` maps topic -> messages in bag order.
    Returns {file name: number of data rows}."""
    rd = Path(out_dir) / "roboter" / session
    rd.mkdir(parents=True, exist_ok=True)
    st, rp = msgs.get(T.STATE, []), msgs.get(T.REPLY, [])
    off = wall_minus_perf(st, rp)
    ro, rr = readout_rows(msgs.get(elp_topic, []), off)
    files = {
        f"rsi_data_{session}.csv": (RSI_COLS, rsi_rows(st, rp)),
        f"ov_cmd_{session}.csv": (OV_COLS, ov_rows(rp)),
        f"ctrl_{session}.csv": (CTRL_COLS, [ctrl_row(m) for m in msgs.get(T.CTRL, [])]),
        f"readout_{session}.csv": (READOUT_COLS, ro),
        f"readout_rows_{session}.csv": (ROWS_COLS, rr),
    }
    for name, (cols, rows) in files.items():
        _write(rd / name, cols, rows)
    if meta_path is not None and Path(meta_path).exists():
        vd = Path(out_dir) / "raw_video" / session
        vd.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(meta_path, vd / "session_meta.json")
    return {name: len(rows) for name, (_, rows) in files.items()}
