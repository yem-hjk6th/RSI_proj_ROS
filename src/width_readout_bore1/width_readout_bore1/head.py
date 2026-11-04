"""bore1 width head: the interface, the frame preprocessing and quality terms, the weight check, and the replay head.

Port of ``legacy/realtime_cl_261023/bore1/bore1_head.py``. A head is called once per gated frame:

    pred = head(frame_bgr, t_frame)          # Bore1Pred(w, w_seeds, sd, q_ok, q, ms)

Two implementations:

- ``torch`` (``torch_head.TorchHead``): the legacy ``Bore1Head``, three frozen ConvNeXt-T seeds on the GPU, loaded from
  a weight directory after the sha256 check below. It ignores ``t_frame``. Needs torch (CUDA build) and timm.
- ``replay`` (``ReplayHead``): the head outputs logged in a ``bore1_rt_<ses>.csv(.gz)``, looked up by frame time; for
  tests and offline replays. It ignores the image (``needs_frame`` is false).

The weight check (``verify_weights``) needs no torch: every ``cnxt_t_deploy_s<seed>.pt`` must match the sha256 list
committed in this package (a copy of the legacy ``weights/weights_sha256.json``), and a list in the weight directory, if
there is one, must be that same list. A missing file or a mismatch refuses (``WeightError``); there is no switch to
skip it. The legacy checked only against the list next to the weights.

bore2 (cage 2, T15; ``legacy/realtime_cl_cage2_semi_261103/bore1/bore2_rt.py`` 24-45): the same head with the bore2
crop x 544-1120, y 0-576 and a horizontal flip before the resize (``crop288(frame, "bore2")``); its weights are checked
against the packaged bore2 sha256 list (``config.sha_json_for``).

No ROS imports. numpy and OpenCV (crop and quality terms).
"""
import csv
import gzip
import hashlib
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

from .config import default_quality_json, default_sha_json, sha_json_for

CROP = (160, 736, 0, 576)       # x0, x1, y0, y1 on the full-res 1280 x 720 frame (legacy line B build_dataset.CROP)
CROP2 = (544, 1120, 0, 576)     # bore2 (legacy bore2_rt.CROP2), then a horizontal flip
CROPS = {"bore1": CROP, "bore2": CROP2}
SIZE = 288
SEEDS = (0, 1, 2)
Q_KEYS = ("bright", "sat", "sharp", "diff")
HEAD_KINDS = ("torch", "replay")


class WeightError(RuntimeError):
    """The bore1 weights are missing or do not match the committed sha256 list."""


def weight_name(seed: int) -> str:
    return f"cnxt_t_deploy_s{seed}.pt"


def sha256(p, chunk=1 << 20) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(chunk), b""):
            h.update(b)
    return h.hexdigest().upper()


def _upper(d: dict) -> Dict[str, str]:
    return {str(k): str(v).upper() for k, v in d.items()}


def verify_weights(weight_dir, sha_json=None, seeds: Sequence[int] = SEEDS) -> List[Path]:
    """Paths of the seed weights in ``weight_dir``, each checked against the committed sha256 list; else WeightError."""
    if not str(weight_dir or ""):
        raise WeightError("bore head: weight_dir is empty (the folder with cnxt_t_deploy_s0.pt .. s2.pt)")
    wd = Path(weight_dir)
    if not wd.is_dir():
        raise WeightError(f"bore1 head: weight_dir {wd} is not a directory")
    want = _upper(json.loads(Path(sha_json or default_sha_json()).read_text(encoding="utf-8")))
    side = wd / "weights_sha256.json"
    if side.exists():
        got = _upper(json.loads(side.read_text(encoding="utf-8")))
        if got != want:
            raise WeightError(f"bore1 head: {side} differs from the committed sha256 list {sha_json or default_sha_json()}")
    paths = []
    for s in seeds:
        p = wd / weight_name(s)
        if p.name not in want:
            raise WeightError(f"bore1 head: {p.name} is not in the sha256 list")
        if not p.is_file():
            raise WeightError(f"bore1 head: {p} is missing")
        if sha256(p) != want[p.name]:
            raise WeightError(f"bore1 head: weight sha256 mismatch: {p}")
        paths.append(p)
    return paths


def crop288(frame_bgr: np.ndarray, camera: str = "bore1") -> np.ndarray:
    """uint8 288 x 288 x 3 RGB crop exactly as the training crops (legacy crop288; bore2: crop288_b2, flipped)."""
    import cv2
    x0, x1, y0, y1 = CROPS[camera]
    c = cv2.resize(frame_bgr[y0:y1, x0:x1], (SIZE, SIZE), interpolation=cv2.INTER_AREA)
    if camera == "bore2":
        c = cv2.flip(c, 1)
    return cv2.cvtColor(c, cv2.COLOR_BGR2RGB)


@dataclass
class QualityGate:
    """Per-frame image checks on the 288 crop (legacy QualityGate). Limits from the training crops (quality_limits.json,
    p0.5-p99.5 of the 7 training prints), widened by ``widen`` x span; a frame outside any limit gets q_ok False
    (logged, and not pushed when drop_bad_q). diff = mean |crop - previous crop| (0 = frozen / repeated frame)."""
    lim: dict = field(default_factory=dict)
    widen: float = 0.25
    prev: Optional[np.ndarray] = None

    @classmethod
    def load(cls, path=None, widen: float = 0.25) -> "QualityGate":
        return cls(lim=json.loads(Path(path or default_quality_json()).read_text(encoding="utf-8")), widen=widen)

    def __call__(self, rgb: np.ndarray):
        import cv2
        g = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        q = dict(bright=float(g.mean()), sat=float((g >= 250).mean()),
                 sharp=float(cv2.Laplacian(g, cv2.CV_32F).var()),
                 diff=float(np.abs(g.astype(np.int16) - self.prev).mean()) if self.prev is not None else np.nan)
        self.prev = g.astype(np.int16)
        ok = True
        for k, (lo, hi) in self.lim.items():
            v = q.get(k)
            if v is None or not np.isfinite(v):
                continue
            span = hi - lo
            if v < lo - self.widen * span or v > hi + self.widen * span:
                ok = False
        if q["diff"] == 0.0:            # exact repeat of the previous frame
            ok = False
        return ok, q


@dataclass
class Bore1Pred:
    w: float                 # mm, mean of the 3 seeds (deploy value)
    w_seeds: tuple           # mm, per seed
    sd: float                # mm, sd of the 3 seeds (ddof 0)
    q_ok: bool
    q: dict                  # bright, sat, sharp, diff
    ms: float                # compute time of this call (crop + quality + 3 models + sync), ms


def no_pred() -> Bore1Pred:
    """What the replay head returns for a frame without a logged head output: NaN, q_ok False (so it is not pushed)."""
    nan = math.nan
    return Bore1Pred(nan, (nan, nan, nan), nan, False, {k: nan for k in Q_KEYS}, nan)


def _f(v: str) -> float:
    return math.nan if v in ("", None) else float(v)


def _open_text(path):
    p = Path(path)
    return gzip.open(p, "rt", encoding="utf-8", newline="") if p.suffix == ".gz" else open(p, encoding="utf-8",
                                                                                           newline="")


class ReplayHead:
    """The head outputs of a logged ``bore1_rt_<ses>.csv(.gz)``, by frame time.

    Only rows with ``gate == ok`` carry head outputs. A call returns the logged prediction whose ``t_frame`` is nearest
    to the given frame time when it is within ``tol_s`` (logged times have 6 decimals), else ``no_pred()`` (counted in
    ``n_miss``). The image is not used. ``ms`` is the logged head time."""
    kind = "replay"
    needs_frame = False

    def __init__(self, csv_path, tol_s: float = 1e-6):
        self.path, self.tol_s = Path(csv_path), float(tol_s)
        t, preds = [], []
        with _open_text(self.path) as f:
            for r in csv.DictReader(f):
                if r["gate"] != "ok":
                    continue
                t.append(float(r["t_frame"]))
                preds.append(Bore1Pred(_f(r["w"]), (_f(r["w_s0"]), _f(r["w_s1"]), _f(r["w_s2"])), _f(r["sd"]),
                                       r["q_ok"] == "1", {k: _f(r[k]) for k in Q_KEYS}, _f(r["ms"])))
        order = np.argsort(np.asarray(t), kind="stable")
        self.t = np.asarray(t, float)[order]
        self.preds = [preds[i] for i in order]
        self.n_calls = self.n_hit = self.n_miss = 0

    def __len__(self) -> int:
        return len(self.preds)

    def lookup(self, t_frame: float) -> Optional[Bore1Pred]:
        if not len(self.t):
            return None
        i = int(np.searchsorted(self.t, t_frame))
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(self.t) and abs(self.t[j] - t_frame) <= self.tol_s:
                if best is None or abs(self.t[j] - t_frame) < abs(self.t[best] - t_frame):
                    best = j
        return None if best is None else self.preds[best]

    def __call__(self, frame_bgr, t_frame: float) -> Bore1Pred:
        self.n_calls += 1
        p = self.lookup(float(t_frame))
        if p is None:
            self.n_miss += 1
            return no_pred()
        self.n_hit += 1
        return Bore1Pred(p.w, tuple(p.w_seeds), p.sd, p.q_ok, dict(p.q), p.ms)

    def warmup(self, n: int = 0) -> None:
        pass

    def info(self) -> dict:
        return dict(kind=self.kind, replay_csv=self.path.name, logged_predictions=len(self), tol_s=self.tol_s,
                    calls=self.n_calls, hits=self.n_hit, misses=self.n_miss)


def make_head(kind: str, weight_dir: str = "", replay_csv: str = "", quality_json: str = "", device: str = "cuda",
              mem_frac: float = 0.0, replay_tol_s: float = 1e-6, warmup: int = 5, camera: str = "bore1",
              weights: str = ""):
    """The head named by ``kind`` (torch | replay), ready to use (the torch head is warmed up). ``camera`` picks the
    crop, ``weights`` (with the camera) the sha256 list (``config.sha_json_for``)."""
    if kind == "replay":
        if not replay_csv:
            raise ValueError("head replay needs replay_csv (a bore1_rt_<ses>.csv(.gz))")
        return ReplayHead(replay_csv, replay_tol_s)
    if kind == "torch":
        sha = sha_json_for(camera, weights)
        paths = verify_weights(weight_dir, sha)         # before torch is imported: a refusal needs no GPU
        from .torch_head import TorchHead
        h = TorchHead(weight_dir, device=device, mem_frac=mem_frac or None,
                      quality=QualityGate.load(quality_json or None), verified=paths, camera=camera)
        h.warmup(warmup)
        return h
    raise ValueError(f"head {kind!r}: one of {', '.join(HEAD_KINDS)}")
