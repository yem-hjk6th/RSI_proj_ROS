"""Preview: decimation, downscaling and the 2x2 tiled window (no ROS imports).

The legacy ``run_cl.py`` showed its preview at 5 Hz (``PREVIEW_PERIOD_S = 0.2``) with 640 px wide tiles
(``show_preview``). Here every camera publishes a decimated, downscaled preview topic, and one window tiles them
2x2: the first two topics form the left column (pair 1, top and bottom), the next two the right column (pair 2).
"""
import math
from typing import List, Optional, Sequence

PREVIEW_HZ = 5.0
PREVIEW_WIDTH = 640


class Decimator:
    """Lets a frame through when its time enters a new period of ``1 / hz`` (on the frame's own clock): exactly
    ``hz`` frames per second on average for any input rate above ``hz``, none dropped below it.

    ``anchored``: the very first frame, which can sit anywhere inside its period, is not let through; passes start at
    the first period boundary. Every pass is then the first frame of its period, so two passes are never closer than
    one period minus one frame interval (the camera node's previews)."""

    def __init__(self, hz: float = PREVIEW_HZ, anchored: bool = False):
        if hz <= 0:
            raise ValueError("preview rate must be > 0")
        self.period = 1.0 / float(hz)
        self.anchored = bool(anchored)
        self.last_slot: Optional[int] = None
        self.passed = 0

    def take(self, t: float) -> bool:
        slot = math.floor(t / self.period)
        if self.last_slot is None and self.anchored:
            self.last_slot = slot               # the first, partial period: wait for its end
            return False
        if self.last_slot is not None and slot <= self.last_slot:
            return False
        self.last_slot = slot
        self.passed += 1
        return True


def downscale(frame, width: int = PREVIEW_WIDTH):
    """Resize to ``width`` (aspect kept, INTER_AREA); frames already that narrow pass unchanged."""
    import cv2
    h, w = frame.shape[:2]
    if w <= width:
        return frame
    return cv2.resize(frame, (int(width), max(1, round(h * width / w))), interpolation=cv2.INTER_AREA)


def fit(frame, tile_w: int, tile_h: int):
    """Scale into a ``tile_w`` x ``tile_h`` tile, letterboxed (legacy ``pipeline._fit_tile``)."""
    import cv2
    h, w = frame.shape[:2]
    s = min(tile_w / w, tile_h / h)
    f = cv2.resize(frame, (max(1, round(w * s)), max(1, round(h * s))), interpolation=cv2.INTER_AREA)
    dx, dy = tile_w - f.shape[1], tile_h - f.shape[0]
    return cv2.copyMakeBorder(f, dy // 2, dy - dy // 2, dx // 2, dx - dx // 2, cv2.BORDER_CONSTANT)


def _label(img, lines: Sequence[str]):
    import cv2
    for i, txt in enumerate(lines):
        y = 22 + 22 * i
        cv2.putText(img, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
        cv2.putText(img, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 1)


def tile_2x2(frames: Sequence, labels: Sequence[Sequence[str]] = (), tile_w: int = PREVIEW_WIDTH,
             tile_h: int = 400):
    """One image of four tiles: ``frames[0]`` top left, ``frames[1]`` bottom left, ``frames[2]`` top right,
    ``frames[3]`` bottom right (pair 1 on the left). A missing frame (None) is a black tile."""
    import numpy as np
    tiles: List = []
    for i in range(4):
        f = frames[i] if i < len(frames) else None
        t = np.zeros((tile_h, tile_w, 3), np.uint8) if f is None else fit(f, tile_w, tile_h)
        if i < len(labels) and labels[i]:
            _label(t, labels[i])
        tiles.append(t)
    left = np.vstack([tiles[0], tiles[1]])
    right = np.vstack([tiles[2], tiles[3]])
    return np.hstack([left, right])


SLOTS = ("top left", "bottom left", "top right", "bottom right")
