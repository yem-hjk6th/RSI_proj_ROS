"""ELP readout of the fan (FAN1) and semicircle (ARC2) patterns: the path gate and the path-following grid.

Port of ``FanReadout`` / ``FanReadoutB`` (``fan_geom.py``) and ``SemiReadout`` / ``SemiReadoutB`` (``semi_geom.py``)
of ``legacy/realtime_cl_cage2_semi_261103/``, for a pattern json with ``"geometry"`` ``fan`` or ``semi``:

- ``gate``: ``path_geometry.gates.elp_gate`` (the legacy order of checks and reasons; tip at ``t_frame - lat``);
- ``read``: the chroma of ``readout.sample_chroma_grid`` on ``path_geometry.gates.elp_grid`` (rows at ``s_tip - d``
  along the line, columns along its local normal), the profile math of ``readout.rows_from_chroma`` with the MDF band
  of the line's ``mdf_side``; ``s_dep = s_tip - d``, rows with ``s_dep >= 0.5`` kept (as legacy);
- semi patterns: one gate log row per gated frame (``path_geometry.gates.GateLog``), as the legacy ``_SemiInit.gate``.

Every other pattern runs ``readout.RtReadout`` unchanged. No ROS imports.
"""
import time
from typing import Optional

import numpy as np

from path_geometry.gates import elp_gate, elp_grid, mdf_band, needs_gate_log
from path_geometry.geometry import load_pattern

from .readout import Readout, Rig, rows_from_chroma, sample_chroma_grid

CAMS = {"A": "elpA", "B": "elpB"}


class PathReadout:
    """gate() (RSI only) then read() (image); ``last_skip`` names the reason of a None, as ``RtReadout``.

    ``kind``: ``A`` (ELP-A, the A legs) or ``B`` (ELP-B, the B legs). ``gate_log``: a ``GateLog`` (semi patterns only;
    ignored for fan patterns, as legacy)."""

    def __init__(self, rig_json, pattern_json, root=None, kind: str = "A", gate_log=None):
        if kind not in CAMS:
            raise ValueError(f"kind {kind!r}: one of {list(CAMS)}")
        self.rig = Rig(rig_json, root)
        self.pat = load_pattern(pattern_json)
        self.kind, self.cam = kind, CAMS[kind]
        if not any(ln.kind == kind for ln in self.pat.lines):
            raise ValueError(f"{pattern_json}: no kind-{kind} legs")
        self.gate_log = gate_log if needs_gate_log(self.pat) else None
        self.last_skip = ""

    def gate(self, t_frame: float, ring):
        g, why = elp_gate(self.pat, ring, t_frame, self.kind, self.rig.lat)
        if g is None:
            self.last_skip = why
            return None
        if self.gate_log is not None:
            self.gate_log.row(self.cam, t_frame, g)
        return g

    def grid(self, g):
        return elp_grid(g.line, g.tip)

    def read(self, im: np.ndarray, g, t_frame: float, frame_idx: int) -> Optional[Readout]:
        t0 = time.perf_counter()
        gx, gy = self.grid(g)
        r = rows_from_chroma(sample_chroma_grid(self.rig, im, g.tip, gx, gy), mdf_band(g.line))
        s_dep = g.s_tip - r[:, 0]
        keep = s_dep >= 0.5
        rows = np.column_stack([r[:, 0], s_dep, r[:, 1:]])[keep]
        return Readout(frame_idx, t_frame, g.leg, g.s_tip, g.tip, rows, time.perf_counter() - t0)

    def __call__(self, im, t_frame: float, ring, frame_idx: int = -1) -> Optional[Readout]:
        g = self.gate(t_frame, ring)
        return None if g is None else self.read(im, g, t_frame, frame_idx)
