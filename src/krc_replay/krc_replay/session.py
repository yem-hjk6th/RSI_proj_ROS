"""A logged session as replay events. No ROS imports (pandas and numpy only).

Inputs, ordering and timing follow T2's golden replay (``src/cl_controller/test/e1_data.py`` and
``test/golden/make_e1_fixture.py``):

- RSI packets: ``roboter/rsi_data_<ses>.csv.gz``, at ``pc_ts_perf``;
- ELP rows: frames of ``roboter/readout_<ses>.csv.gz`` with ``n_used > 0`` and their rows in
  ``readout_rows_<ses>.csv.gz``, pushed at ``t_done_perf``;
- bore1 rows: ``raw_video/bore1_rt_<ses>.csv.gz`` with ``pushed = 1``, one row at d = 5 mm, pushed at ``t_done``;
- controller updates: ``roboter/ctrl_<ses>.csv.gz`` ``t_perf``.

Events are merged in time order; on a tie a packet goes before a readout, and everything at or before an update time
goes before that update.
"""
import math
import re
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

BORE1_D_MM = 5.0


@dataclass
class Readout:
    """One readout (frame) of one source, as the legacy thread pushed it to the controller's row store."""
    t_push: float        # time the row store received it
    t_frame: float       # frame time
    src: str             # elp | bore1
    leg: int
    s_tip: float
    d: np.ndarray
    s: np.ndarray
    w: np.ndarray


def sec_ns(t: float) -> Tuple[int, int]:
    """(sec, nanosec) of a builtin_interfaces/Time for t seconds."""
    sec = int(math.floor(t))
    ns = int(round((t - sec) * 1e9))
    if ns >= 1_000_000_000:
        sec, ns = sec + 1, ns - 1_000_000_000
    return sec, ns


def sec_of(stamp) -> float:
    """Seconds of a Time message, as the nodes compute it."""
    return stamp.sec + stamp.nanosec * 1e-9


def ses_of(session_dir) -> str:
    """'20261022_165128' from a folder named like 'e1_20261022_165128'."""
    m = re.search(r"(\d{8}_\d{6})$", Path(session_dir).name)
    if not m:
        raise ValueError(f"{session_dir}: folder name must end in <yyyymmdd_hhmmss>")
    return m.group(1)


class Session:
    def __init__(self, session_dir):
        import pandas as pd
        self.dir = Path(session_dir)
        self.ses = ses_of(self.dir)
        rb, rv = self.dir / "roboter", self.dir / "raw_video"
        rsi = pd.read_csv(rb / f"rsi_data_{self.ses}.csv.gz")
        self.pk_t = rsi["pc_ts_perf"].to_numpy(float)
        self.pk_ipoc = rsi["ipoc"].to_numpy(np.int64)
        self.pk_xyz = rsi[["rist_x_mm", "rist_y_mm", "rist_z_mm"]].to_numpy(float)
        self.pk_abc = rsi[["rist_a_deg", "rist_b_deg", "rist_c_deg"]].to_numpy(float)
        self.pk_override = rsi["override"].to_numpy(np.int64)
        self.pk_vel_act = rsi["vel_act"].to_numpy(float)
        self.pk_rpm = rsi["rpm_ext"].to_numpy(np.int64)
        self.t_updates = pd.read_csv(rb / f"ctrl_{self.ses}.csv.gz", usecols=["t_perf"])["t_perf"].to_numpy(float)
        ov = pd.read_csv(rb / f"ov_cmd_{self.ses}.csv.gz")
        self.ov_ipoc = ov["ipoc"].to_numpy(np.int64)
        self.ov_cmd = ov["ov_cmd"].to_numpy(np.int64)
        self.ov_source = ov["source"].astype(str).to_numpy()
        self.readouts = self._readouts(pd, rb, rv)

    def _readouts(self, pd, rb, rv) -> List[Readout]:
        out = []
        ro = pd.read_csv(rb / f"readout_{self.ses}.csv.gz")
        ro = ro[ro["n_used"] > 0]
        rows = pd.read_csv(rb / f"readout_rows_{self.ses}.csv.gz")
        grp = {f: g for f, g in rows.groupby("frame")}
        for fr, tf, t, leg, s_tip in zip(ro["frame"], ro["t_frame_perf"], ro["t_done_perf"], ro["leg"], ro["s_tip"]):
            g = grp.get(fr)
            if g is None:
                continue
            out.append(Readout(float(t), float(tf), "elp", int(leg), float(s_tip), g["d"].to_numpy(float),
                               g["s_dep"].to_numpy(float), g["w"].to_numpy(float)))
        b1 = pd.read_csv(rv / f"bore1_rt_{self.ses}.csv.gz")
        b1 = b1[b1["pushed"] == 1]
        for tf, t, leg, s_tip, s_dep, w in zip(b1["t_frame"], b1["t_done"], b1["leg"], b1["s_tip"], b1["s_dep"],
                                               b1["w"]):
            out.append(Readout(float(t), float(tf), "bore1", int(leg), float(s_tip), np.array([BORE1_D_MM]),
                               np.array([float(s_dep)]), np.array([float(w)])))
        out.sort(key=lambda r: r.t_push)       # stable: ELP before bore1 on equal times, as in the golden replay
        return out

    # ---------------------------------------------------------------- lock-step schedule
    def lockstep_steps(self):
        """Per controller update k: (t_update, packet index range, readouts) with everything at or before t_update
        that was not fed before. Same merge as the golden replay (packet first on a tie)."""
        ip = ir = 0
        n_pk, ros = len(self.pk_t), self.readouts
        for tu in self.t_updates:
            p0, r0 = ip, ir
            while True:
                t_p = self.pk_t[ip] if ip < n_pk else math.inf
                t_r = ros[ir].t_push if ir < len(ros) else math.inf
                if min(t_p, t_r) > tu:
                    break
                if t_p <= t_r:
                    ip += 1
                else:
                    ir += 1
            yield float(tu), (p0, ip), ros[r0:ir]

    # ---------------------------------------------------------------- real-time keying
    def row_of_ipoc(self, ipoc: int) -> Optional[int]:
        i = int(np.searchsorted(self.pk_ipoc, ipoc))
        return i if i < len(self.pk_ipoc) and self.pk_ipoc[i] == ipoc else None

    def readouts_until(self, i0: int, t: float) -> int:
        """Index one past the last readout pushed at or before t, starting the search at i0."""
        ros = self.readouts
        i = i0
        while i < len(ros) and ros[i].t_push <= t:
            i += 1
        return i


@lru_cache(maxsize=2)
def load(session_dir) -> Session:
    return Session(str(session_dir))
