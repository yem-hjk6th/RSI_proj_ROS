"""bore1 width head for live use: one bore1 frame -> line width at 5 mm behind the nozzle + seed spread + frame quality.

Model = the frozen deploy version (1018 line PF, FREEZE_bore1.md): timm convnext_tiny.dinov3_lvd1689m, 3 seeds, deploy
value = mean of the 3 predictions. Preprocessing is predict_bore1.py / line B build_dataset.extract() verbatim:
full-res frame x 160-736, y 0-576 -> INTER_AREA 288 x 288 -> BGR->RGB -> /255 -> (x - mean) / std of the checkpoint;
prediction * ys + ym; bf16 autocast on cuda.
The weights are checked against weights_sha256.json before loading (refuses on a mismatch).

    head = Bore1Head(weight_dir, device="cuda")
    p = head(frame_bgr)          # Bore1Pred(w, w_seeds, sd, q_ok, q)
"""
import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
import torch

CROP = (160, 736, 0, 576)       # x0, x1, y0, y1 on the full-res 1280 x 720 frame (line B build_dataset.CROP)
SIZE = 288
SEEDS = (0, 1, 2)


def sha256(p, chunk=1 << 20):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(chunk), b""):
            h.update(b)
    return h.hexdigest().upper()


def crop288(frame_bgr):
    """uint8 288 x 288 x 3 RGB crop exactly as the training crops (int/crops_<tag>.npy)."""
    x0, x1, y0, y1 = CROP
    return cv2.cvtColor(cv2.resize(frame_bgr[y0:y1, x0:x1], (SIZE, SIZE), interpolation=cv2.INTER_AREA),
                        cv2.COLOR_BGR2RGB)


@dataclass
class QualityGate:
    """Per-frame image checks on the 288 crop. Limits come from the training crops (quality_limits.json, p0.5-p99.5
    of the 7 training prints, widened by `widen`); a frame outside any limit gets q_ok False (logged, not dropped,
    unless the caller drops it). diff = mean |crop - previous crop| (0 = frozen / repeated frame)."""
    lim: dict = field(default_factory=dict)
    widen: float = 0.25
    prev: np.ndarray = None

    @classmethod
    def load(cls, path, widen=0.25):
        return cls(lim=json.loads(Path(path).read_text(encoding="utf-8")), widen=widen)

    def __call__(self, rgb):
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
    q: dict
    ms: float                # compute time of this call (crop + quality + 3 models + sync), ms


class Bore1Head:
    def __init__(self, weight_dir, device="cuda", mem_frac=None, quality=None, check_sha=True, use_graph=True):
        import timm
        self.dev = device
        wd = Path(weight_dir)
        want = json.loads((wd / "weights_sha256.json").read_text(encoding="utf-8")) if check_sha else {}
        if device == "cuda":
            if mem_frac:
                torch.cuda.set_per_process_memory_fraction(mem_frac)
            torch.backends.cudnn.benchmark = True
        self.models, ym, ys, means, stds = [], [], [], [], []
        for s in SEEDS:
            p = wd / f"cnxt_t_deploy_s{s}.pt"
            if check_sha and sha256(p) != want[p.name].upper():
                raise SystemExit(f"weight sha256 mismatch: {p}")
            ck = torch.load(p, map_location="cpu", weights_only=False)
            m = timm.create_model(ck["timm_ref"], pretrained=False, num_classes=1).to(memory_format=torch.channels_last)
            m.load_state_dict(ck["state_dict"])
            self.models.append(m.to(device).eval())
            ym.append(ck["ym"]), ys.append(ck["ys"])
            means.append(ck["mean"]), stds.append(ck["std"])
            assert int(ck["size"]) == SIZE
        assert all(np.allclose(means[0], m) for m in means) and all(np.allclose(stds[0], s) for s in stds)
        self.ym, self.ys = np.array(ym), np.array(ys)
        self.mean = torch.tensor(means[0], device=device).view(1, 3, 1, 1)
        self.std = torch.tensor(stds[0], device=device).view(1, 3, 1, 1)
        self.quality = quality or QualityGate()
        self.graph = None
        if device == "cuda" and use_graph:
            self._build_graph()

    def _fwd_u8(self, u8):
        """uint8 (n, 288, 288, 3) cuda -> (n, 3) normalised-space outputs (same ops as predict_rgb)."""
        xb = u8.permute(0, 3, 1, 2)
        xb = (((xb.float() / 255.0) - self.mean) / self.std).contiguous(memory_format=torch.channels_last)
        with torch.autocast("cuda", dtype=torch.bfloat16, cache_enabled=False):
            return torch.stack([m(xb).reshape(-1).float() for m in self.models], 1)

    @torch.no_grad()
    def _build_graph(self):
        """single-frame CUDA graph of crop-normalise + 3 models: the batch-1 cost on Windows is kernel-launch
        overhead (~8 ms / model eager on the 5080), the graph replays all launches at once."""
        self.g_u8 = torch.zeros((1, SIZE, SIZE, 3), dtype=torch.uint8, device="cuda")
        s = torch.cuda.Stream()
        s.wait_stream(torch.cuda.current_stream())
        with torch.cuda.stream(s):
            for _ in range(3):
                self._fwd_u8(self.g_u8)
        torch.cuda.current_stream().wait_stream(s)
        self.graph = torch.cuda.CUDAGraph()
        with torch.cuda.graph(self.graph):
            self.g_out = self._fwd_u8(self.g_u8)
        self.h_u8 = torch.zeros((1, SIZE, SIZE, 3), dtype=torch.uint8).pin_memory()

    @torch.no_grad()
    def predict_one(self, rgb):
        """uint8 (288, 288, 3) RGB -> (3,) mm per seed; CUDA graph when built, else predict_rgb."""
        if self.graph is None:
            return self.predict_rgb(rgb[None])[0]
        self.h_u8[0].copy_(torch.from_numpy(rgb))
        self.g_u8.copy_(self.h_u8, non_blocking=True)
        self.graph.replay()
        return self.g_out[0].cpu().numpy() * self.ys + self.ym

    @torch.no_grad()
    def predict_rgb(self, rgb_batch):
        """uint8 (n, 288, 288, 3) RGB -> (n, 3) mm per seed."""
        xb = torch.from_numpy(np.ascontiguousarray(rgb_batch)).to(self.dev, non_blocking=True).permute(0, 3, 1, 2)
        xb = (((xb.float() / 255.0) - self.mean) / self.std).contiguous(memory_format=torch.channels_last)
        out = []
        with torch.autocast(self.dev, dtype=torch.bfloat16, enabled=self.dev == "cuda"):
            for m in self.models:
                out.append(m(xb).reshape(-1).float())
        y = torch.stack(out, 1).cpu().numpy()
        return y * self.ys[None, :] + self.ym[None, :]

    def __call__(self, frame_bgr):
        import time
        t0 = time.perf_counter()
        rgb = crop288(frame_bgr)
        ok, q = self.quality(rgb)
        y = self.predict_one(rgb)
        return Bore1Pred(float(y.mean()), tuple(float(v) for v in y), float(y.std()), ok, q,
                         1e3 * (time.perf_counter() - t0))
