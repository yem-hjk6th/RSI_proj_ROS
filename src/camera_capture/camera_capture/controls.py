"""V4L2 controls: set from parameters in the given order, read every one back (no ROS imports).

``ElpCam.open`` applied its controls after every open, read each one back and logged requested and read-back values.
Here the controls are V4L2 controls by their ``v4l2-ctl`` names, e.g. for the ELP (E1 profile on DirectShow:
``CAP_PROP_AUTO_EXPOSURE`` 0.25 = manual, ``CAP_PROP_EXPOSURE`` -7 = 2^-7 s = 7.81 ms, gain 0, auto white balance off,
4600 K)::

    auto_exposure=1                  # V4L2 menu: 1 = manual
    exposure_time_absolute=78        # unit 100 us: 7.8 ms (2^-7 s = 7.8125 ms)
    gain=0
    white_balance_automatic=0
    white_balance_temperature=4600

Order matters: an absolute exposure or white balance is inactive (not settable) while its automatic mode is on.

Borescopes never get an exposure control: on the legacy (DirectShow) setup, touching the NTG100's exposure stopped
its driver until the USB plug was pulled (``borescope.py``). ``check_allowed`` refuses that for ``kind: borescope``.

The backend is ``v4l2-ctl`` (v4l-utils), run once per control at open, never in the capture loop.
"""
import re
import shutil
import subprocess
from typing import Dict, List, Optional, Sequence, Tuple

Control = Tuple[str, int]

_NAME = re.compile(r"^[a-z0-9_]+$")


def exposure_units_100us(seconds: float) -> int:
    """V4L2 ``exposure_time_absolute`` (100 us units) for an exposure time in seconds, rounded."""
    return int(round(seconds / 100e-6))


def parse_controls(items: Sequence[str]) -> List[Control]:
    """``["name=value", ...]`` -> ``[(name, int value), ...]`` in the given order; names as ``v4l2-ctl`` prints them."""
    out: List[Control] = []
    seen = set()
    for it in items:
        it = str(it).strip()
        if not it:
            continue
        if "=" not in it:
            raise ValueError(f"control {it!r}: expected name=value")
        name, val = (s.strip() for s in it.split("=", 1))
        if not _NAME.match(name):
            raise ValueError(f"control {it!r}: name must be a v4l2-ctl control name (lower case, digits, _)")
        if name in seen:
            raise ValueError(f"control {name} given twice")
        seen.add(name)
        try:
            v = int(val)
        except ValueError:
            raise ValueError(f"control {it!r}: value must be an integer (V4L2 units)") from None
        out.append((name, v))
    return out


def check_allowed(kind: str, controls: Sequence[Control]) -> None:
    """Refuse exposure controls on a borescope (see the module docstring)."""
    if kind == "borescope":
        bad = [n for n, _ in controls if "exposure" in n]
        if bad:
            raise ValueError(f"borescope: never set exposure ({', '.join(bad)}); legacy borescope.py: touching the "
                             f"NTG100's exposure stopped its driver until a USB replug")


def parse_get_ctrl(text: str) -> Dict[str, int]:
    """``v4l2-ctl --get-ctrl`` output (``name: value`` per line) -> {name: value}."""
    out = {}
    for line in text.splitlines():
        if ":" not in line:
            continue
        k, v = (s.strip() for s in line.split(":", 1))
        try:
            out[k] = int(v.split()[0])
        except (ValueError, IndexError):
            continue
    return out


def parse_list_ctrls(text: str) -> Dict[str, dict]:
    """``v4l2-ctl --list-ctrls`` -> {name: {"type": .., "min": .., "max": .., "default": .., "value": .., ...}}.
    Lines look like ``exposure_time_absolute 0x009a0902 (int) : min=1 max=5000 step=1 default=156 value=156``."""
    out = {}
    for line in text.splitlines():
        m = re.match(r"^\s*([a-z0-9_]+)\s+0x[0-9a-f]+\s+\((\w+)\)\s*:\s*(.*)$", line)
        if not m:
            continue
        d: dict = {"type": m.group(2)}
        for k, v in re.findall(r"(\w+)=(\S+)", m.group(3)):
            try:
                d[k] = int(v)
            except ValueError:
                d[k] = v
        out[m.group(1)] = d
    return out


class V4l2Ctl:
    """Controls through the ``v4l2-ctl`` command."""

    def __init__(self, exe: str = "v4l2-ctl", timeout: float = 5.0):
        self.exe = shutil.which(exe) or exe
        self.timeout = timeout

    def _run(self, *args) -> subprocess.CompletedProcess:
        return subprocess.run([self.exe, *args], capture_output=True, text=True, timeout=self.timeout)

    def available(self) -> bool:
        return shutil.which(self.exe) is not None

    def list_ctrls(self, device: str) -> str:
        r = self._run("-d", device, "--list-ctrls")
        return r.stdout if r.returncode == 0 else ""

    def set(self, device: str, name: str, value: int) -> Optional[str]:
        """None on success, else the error text."""
        r = self._run("-d", device, f"--set-ctrl={name}={value}")
        return None if r.returncode == 0 and not r.stderr.strip() else (r.stderr.strip() or f"exit {r.returncode}")

    def get(self, device: str, name: str) -> Optional[int]:
        r = self._run("-d", device, f"--get-ctrl={name}")
        if r.returncode != 0:
            return None
        return parse_get_ctrl(r.stdout).get(name)


def apply_controls(backend, device: str, controls: Sequence[Control], log=print) -> dict:
    """Set every control in order, then read every one back. Returns
    ``{"requested": {..}, "readback": {..}, "errors": {name: text}, "mismatch": [names]}`` and logs requested and
    read-back values, as ``ElpCam.open`` did."""
    requested = {n: v for n, v in controls}
    errors = {}
    for n, v in controls:
        err = backend.set(device, n, v)
        if err is not None:
            errors[n] = err
            log(f"WARN set {n}={v} failed: {err}")
    readback = {n: backend.get(device, n) for n, _ in controls}
    mismatch = [n for n, v in controls if readback[n] != v]
    log(f"controls requested: {requested}")
    log(f"controls read back: {readback}")
    for n in mismatch:
        log(f"WARN {n} reads back {readback[n]}, requested {requested[n]}")
    return dict(requested=requested, readback=readback, errors=errors, mismatch=mismatch)
