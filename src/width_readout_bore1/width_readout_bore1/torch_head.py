"""The torch bore1 head: port of ``Bore1Head`` in ``legacy/realtime_cl_261023/bore1/bore1_head.py``.

Model = the frozen deploy version: timm ``convnext_tiny.dinov3_lvd1689m`` (``pretrained=False``; the weights come from
the ``.pt``), 3 seeds, deploy value = mean of the 3 predictions. Preprocessing as in training: full-res frame
x 160-736, y 0-576 -> INTER_AREA 288 x 288 -> BGR->RGB -> /255 -> (x - mean) / std of the checkpoint;
prediction * ys + ym; bf16 autocast on cuda; one CUDA graph for crop-normalise + 3 models per frame.

Differences from legacy: the sha256 check is ``head.verify_weights`` (against the list committed in this package, and
it cannot be switched off; ``verified`` takes the paths of a check already done, so that ``make_head`` hashes once); a refusal raises ``WeightError`` instead of ``SystemExit``; ``__call__`` takes the frame
time (unused) so that it has the head interface of ``head.py``; ``camera`` bore2 = the legacy ``Bore2Head`` (crop
x 544-1120 and a horizontal flip, ``head.crop288``); ``warmup`` is the warm-up of the legacy
``bore1_rt.load_head``. Everything else, including the order of the operations, is the legacy code.

Needs torch (a CUDA build; RTX 50 series needs the cu128 wheels) and timm, see the legacy ``requirements_bore1.txt``.
Not imported by anything else in this package unless ``head: torch`` is chosen.
"""
import time

import numpy as np
import torch

from .head import SEEDS, SIZE, Bore1Pred, QualityGate, crop288, verify_weights


class TorchHead:
    kind = "torch"
    needs_frame = True

    def __init__(self, weight_dir, device="cuda", mem_frac=None, quality=None, use_graph=True, sha_json=None,
                 verified=None, camera="bore1"):
        import timm
        paths = verified if verified is not None else verify_weights(weight_dir, sha_json)
        self.weight_dir = str(weight_dir)
        self.camera = camera                    # bore1 | bore2: the crop (head.crop288)
        self.dev = device
        if device == "cuda":
            if mem_frac:
                torch.cuda.set_per_process_memory_fraction(mem_frac)
            torch.backends.cudnn.benchmark = True
        self.models, ym, ys, means, stds = [], [], [], [], []
        for s, p in zip(SEEDS, paths):
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
        """single-frame CUDA graph of crop-normalise + 3 models: the batch-1 cost is kernel-launch overhead, the graph
        replays all launches at once."""
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

    def __call__(self, frame_bgr, t_frame=None) -> Bore1Pred:
        t0 = time.perf_counter()
        rgb = crop288(frame_bgr, self.camera)
        ok, q = self.quality(rgb)
        y = self.predict_one(rgb)
        return Bore1Pred(float(y.mean()), tuple(float(v) for v in y), float(y.std()), ok, q,
                         1e3 * (time.perf_counter() - t0))

    def warmup(self, n: int = 5) -> None:
        """legacy bore1_rt.load_head: n dummy frames (cudnn autotune), then forget the quality gate's previous crop."""
        dummy = np.zeros((720, 1280, 3), np.uint8)
        for _ in range(n):
            self(dummy)
        self.quality.prev = None

    def info(self) -> dict:
        return dict(kind=self.kind, camera=self.camera, device=self.dev, cuda_graph=self.graph is not None,
                    torch=torch.__version__, gpu=torch.cuda.get_device_name(0) if self.dev == "cuda" else "")
