"""Per-packet override decision of the RSI reply loop. No ROS imports.

Ported from the decision block of ``legacy/realtime_cl_261023/rsi_proc.py`` (``_child``):

- outside the print window -> ``ov_outside``;
- forced (latched, never cleared) -> ``ov_default``;
- step mode -> ``step_seq`` with ``step_hold_s`` holds from the window opening, then ``ov_default``;
- closed / shadow -> the controller's value, or ``ov_default`` when its heartbeat is older than ``wd_s``;
- forced and watchdog ramp linearly from the last value sent inside the window under control / default to
  ``ov_default`` over ``fallback_ramp_s`` (no ramp when nothing was sent inside yet; 0 s = jump);
  the watchdog ramp ends as soon as the heartbeat is fresh again;
- every value is clamped to ``[ov_min, ov_max]``, where ``ov_min`` already includes the speed floor;
- any exception in the decision -> ``clamp(ov_default)`` ("fallback").

Not ported: the step test's ``--limit-probe``. It deliberately sent an unclamped value and would break
safety invariant 2.
"""
import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

from .window import STATE_CODE, PatternGeom, PrintWindow

# Source codes as in the legacy sidecar ov_cmd_<ses>.csv (column "source").
SRC_OUT = ["outside", "forced", "step", "default", "control", "watchdog", "fallback", "none"]
SRC_OUTSIDE, SRC_FORCED, SRC_STEP, SRC_DEFAULT, SRC_CONTROL, SRC_WATCHDOG, SRC_FALLBACK, SRC_NONE = range(8)

# Controller-side source codes written into the shared value (legacy rt_controller.publish).
CTRL_DEFAULT, CTRL_CONTROL, CTRL_WATCHDOG = 0, 1, 2

MODES = ("closed", "shadow", "step")


@dataclass
class DeciderConfig:
    """Bridge-side override limits. Defaults are the legacy ``CtrlCfg`` defaults."""

    mode: str = "closed"
    ov_min: int = 1                 # outer lower limit; the effective limit also includes the speed floor
    ov_max: int = 100
    ov_outside: int = 100
    ov_default: int = 50
    v_min_mm_s: float = 3.4         # lowest commanded print speed; 0 = no floor
    fallback_ramp_s: float = 2.0
    wd_s: float = 1.0
    window_xy_tol: float = 1.5
    z_tol: float = 0.5
    window_ignore_z: bool = False
    step_seq: Tuple[int, ...] = field(default=(50, 0, 100, 50, 28, 90, 50, 28, 90))
    step_hold_s: float = 6.0


def ov_floor(v_min_mm_s: float, v100_min_mm_s: Optional[float]) -> int:
    """Speed floor: OvCmd >= ceil(v_min x 100 / v100) on the slowest straight segment (legacy run_cl.speed_rules)."""
    if not v_min_mm_s or v_min_mm_s <= 0:
        return 0
    if not v100_min_mm_s or v100_min_mm_s <= 0:
        raise ValueError("speed floor needs the pattern's speed at OvCmd 100 (segs column v100_mm_s or json ov_ref)")
    return int(math.ceil(v_min_mm_s * 100.0 / v100_min_mm_s - 1e-9))


def effective_limits(cfg: DeciderConfig, pat: PatternGeom) -> Tuple[int, int, int]:
    """(ov_min including the floor, ov_max, floor). Raises ValueError on an unusable window."""
    floor = ov_floor(cfg.v_min_mm_s, pat.v100_min_mm_s)
    lo, hi = max(int(cfg.ov_min), floor), int(cfg.ov_max)
    lo_allowed = 0 if cfg.mode == "step" else 1
    if not (lo_allowed <= lo <= hi <= 100):
        raise ValueError(f"override window {lo}..{hi} (floor {floor}): need {lo_allowed} <= ov_min <= ov_max <= 100")
    if cfg.mode != "step" and not (lo <= int(cfg.ov_default) <= hi):
        raise ValueError(f"ov_default {cfg.ov_default} outside the window {lo}..{hi}")
    if cfg.mode not in MODES:
        raise ValueError(f"mode {cfg.mode!r}: need one of {MODES}")
    if not (math.isfinite(cfg.fallback_ramp_s) and cfg.fallback_ramp_s >= 0):
        raise ValueError(f"fallback_ramp_s {cfg.fallback_ramp_s}: need >= 0")
    if not (math.isfinite(cfg.wd_s) and cfg.wd_s > 0):
        raise ValueError(f"wd_s {cfg.wd_s}: need > 0")
    return lo, hi, floor


class OverrideDecider:
    """Stateful per-packet decision. ``decide`` never raises."""

    def __init__(self, cfg: DeciderConfig, pat: PatternGeom):
        self.cfg = cfg
        self.lo, self.hi, self.floor = effective_limits(cfg, pat)
        self.win = PrintWindow(pat, cfg.window_xy_tol, cfg.z_tol, cfg.window_ignore_z)
        self.ramp_s = float(cfg.fallback_ramp_s) if cfg.fallback_ramp_s and cfg.fallback_ramp_s > 0 else 0.0
        self.fb: Optional[Tuple[float, int]] = None      # (t0, ov_from) of the running forced / watchdog ramp
        self.ov_last_in: Optional[int] = None            # last value sent inside under control / default
        self.step_t0: Optional[float] = None
        self.errors = 0

    def clamp(self, v) -> int:
        return min(max(int(v), self.lo), self.hi)

    @property
    def win_state(self) -> int:
        return STATE_CODE.get(self.win.state, 0)

    def decide(self, t: float, xyz, forced: bool, hb: float, ov_ctrl: int, ov_src: int) -> Tuple[int, int]:
        """(OvCmd, source code) for one packet received at perf time ``t`` with RIst position ``xyz``.

        ``hb`` is the perf time of the controller's last heartbeat, ``ov_ctrl`` / ``ov_src`` its last command.
        """
        cfg = self.cfg
        try:
            st = self.win.update(t, xyz)
            if st != "inside":
                ov, src = cfg.ov_outside, SRC_OUTSIDE
            elif forced:
                ov, src = cfg.ov_default, SRC_FORCED
            elif cfg.mode == "step":
                if self.step_t0 is None:
                    self.step_t0 = t
                k = int((t - self.step_t0) / cfg.step_hold_s)
                ov, src = (cfg.step_seq[k] if k < len(cfg.step_seq) else cfg.ov_default), SRC_STEP
            elif t - hb > cfg.wd_s:
                ov, src = cfg.ov_default, SRC_WATCHDOG
            else:
                ov, src = ov_ctrl, SRC_DEFAULT + ov_src
            if src in (SRC_FORCED, SRC_WATCHDOG) and self.ramp_s > 0.0 and self.ov_last_in is not None:
                if self.fb is None:
                    self.fb = (t, self.ov_last_in)
                f = min(1.0, (t - self.fb[0]) / self.ramp_s)
                ov = int(round(self.fb[1] + f * (cfg.ov_default - self.fb[1])))
            elif src not in (SRC_FORCED, SRC_WATCHDOG):
                self.fb = None
            ov = self.clamp(ov)
            if src in (SRC_DEFAULT, SRC_CONTROL):
                self.ov_last_in = ov
            return ov, src
        except Exception:  # noqa: BLE001 - the reply must go out
            self.errors += 1
            return self.clamp(cfg.ov_default), SRC_FALLBACK
