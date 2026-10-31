"""Override supervisor: clamp, speed floor, heartbeat watchdog and linear fallback ramp.

Pure Python, no ROS. Same rules as the RSI bridge (legacy ``rsi_proc.py``, 261022):

- every value that leaves the bridge is clamped to ``[max(ov_min, ov_floor), ov_max]``;
- the controller's ``OverrideCmd.seq`` is the heartbeat: a command refreshes it only when ``seq`` changes;
- while the heartbeat is fresh (younger than ``wd_s``) the controller's value goes out (source ``control``);
- when the heartbeat goes stale, the output ramps linearly from the last value sent under control to
  ``ov_default`` over ``fallback_ramp_s`` (source ``watchdog``); ``fallback_ramp_s = 0`` jumps at once;
- with no command ever received, ``ov_default`` goes out (source ``default``), no ramp;
- a fresh heartbeat ends the ramp immediately (control again).

All times are seconds on one monotonic clock chosen by the caller.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class OverrideConfig:
    ov_min: int = 1             # outer lower limit [%]
    ov_max: int = 100           # outer upper limit [%]
    ov_floor: int = 0           # speed floor [%]; the effective lower limit is max(ov_min, ov_floor)
    ov_default: int = 50        # fallback value [%]
    wd_s: float = 1.0           # heartbeat timeout [s]
    fallback_ramp_s: float = 2.0  # ramp duration to ov_default [s]; 0 = jump

    def __post_init__(self) -> None:
        if not 0 <= int(self.ov_min) <= int(self.ov_max) <= 100:
            raise ValueError("need 0 <= ov_min <= ov_max <= 100")
        if int(self.ov_floor) > int(self.ov_max):
            raise ValueError("ov_floor must not exceed ov_max")
        if not self.wd_s > 0:
            raise ValueError("wd_s must be > 0")
        if self.fallback_ramp_s < 0 or math.isnan(self.fallback_ramp_s):
            raise ValueError("fallback_ramp_s must be >= 0")

    @property
    def lo(self) -> int:
        return max(int(self.ov_min), int(self.ov_floor))

    @property
    def hi(self) -> int:
        return int(self.ov_max)


def clamp_override(value: float, cfg: OverrideConfig) -> int:
    """Round to an integer percent and clamp to ``[max(ov_min, ov_floor), ov_max]``.

    NaN or infinite values map to the clamped ``ov_default``.
    """
    try:
        v = float(value)
    except (TypeError, ValueError):
        v = float("nan")
    if not math.isfinite(v):
        v = float(cfg.ov_default)
    return min(max(int(round(v)), cfg.lo), cfg.hi)


class OverrideSupervisor:
    """Decides the override the bridge sends, from the controller's commands and the clock."""

    def __init__(self, cfg: OverrideConfig) -> None:
        self.cfg = cfg
        self._ov_ctrl: Optional[int] = None    # last commanded value (clamped)
        self._seq: Optional[int] = None
        self._hb: Optional[float] = None       # time of the last heartbeat
        self._last_ctrl_out: Optional[int] = None  # last value sent under control (ramp start)
        self._ramp: Optional[Tuple[float, int]] = None  # (t0, ov_from) of the running ramp

    def on_command(self, value_pct: float, seq: int, t: float) -> None:
        """Store a command. Only a changed ``seq`` refreshes the heartbeat."""
        self._ov_ctrl = clamp_override(value_pct, self.cfg)
        seq = int(seq)
        if self._seq is None or seq != self._seq:
            self._seq = seq
            self._hb = float(t)

    def heartbeat_fresh(self, t: float) -> bool:
        return self._hb is not None and (t - self._hb) <= self.cfg.wd_s

    def output(self, t: float) -> Tuple[int, str]:
        """Return ``(override_pct, source)`` for time ``t``. The value is always clamped."""
        cfg = self.cfg
        if self._ov_ctrl is not None and self.heartbeat_fresh(t):
            self._ramp = None
            ov = clamp_override(self._ov_ctrl, cfg)
            self._last_ctrl_out = ov
            return ov, "control"
        if self._last_ctrl_out is None:
            return clamp_override(cfg.ov_default, cfg), "default"
        # stale heartbeat after control: linear ramp to ov_default
        if cfg.fallback_ramp_s <= 0.0:
            return clamp_override(cfg.ov_default, cfg), "watchdog"
        if self._ramp is None:
            # the ramp starts where the heartbeat went stale, not at the first call after it
            t0 = self._hb + cfg.wd_s if self._hb is not None else t
            self._ramp = (min(t0, t), self._last_ctrl_out)
        t0, ov_from = self._ramp
        f = min(1.0, max(0.0, (t - t0) / cfg.fallback_ramp_s))
        return clamp_override(ov_from + f * (cfg.ov_default - ov_from), cfg), "watchdog"
