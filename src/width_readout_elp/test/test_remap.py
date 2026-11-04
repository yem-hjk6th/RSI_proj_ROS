"""readout.remap_linear: the OpenCV 4 float-map bilinear remap, bit for bit, on any OpenCV version.

OpenCV 4 rounds a float map to 1/32 pixel (half to even), splits it into the integer pixel and the 5-bit fraction,
weights the four neighbours with (32 - a or a) * (32 - b or b) * 32 (sum 2^15), adds 2^14 and shifts by 15; a
neighbour outside the image counts as 0 (BORDER_CONSTANT). OpenCV 5 interpolates float maps at the full position
instead, which moved the E1 goldens by up to 0.007 mm (README, "OpenCV versions"). ``model`` below is that OpenCV 4
arithmetic in numpy; the test asserts remap_linear equals it exactly, so it holds on OpenCV 4 and 5 alike.
"""
import cv2
import numpy as np
import pytest

from legacy_paths import FRAMES
from width_readout_elp.readout import remap_linear


def model(im, u, v):
    H, W = im.shape[:2]
    X = np.rint(np.asarray(u, np.float32) * np.float32(32)).astype(np.int64)
    Y = np.rint(np.asarray(v, np.float32) * np.float32(32)).astype(np.int64)
    x0, y0, ax, ay = X >> 5, Y >> 5, X & 31, Y & 31
    f = im.astype(np.int64)
    acc = np.zeros(np.shape(u) + (im.shape[2],), np.int64)
    for dy, wy in ((0, 32 - ay), (1, ay)):
        for dx, wx in ((0, 32 - ax), (1, ax)):
            yy, xx = y0 + dy, x0 + dx
            inside = (yy >= 0) & (yy < H) & (xx >= 0) & (xx < W)
            px = f[np.clip(yy, 0, H - 1), np.clip(xx, 0, W - 1)]
            acc += np.where(inside[..., None], px, 0) * (wy * wx * 32)[..., None]
    return np.clip((acc + (1 << 14)) >> 15, 0, 255).astype(np.uint8)


@pytest.mark.parametrize("seed", range(3))
def test_remap_linear_is_the_opencv4_fixed_point_remap(seed):
    rng = np.random.default_rng(seed)
    im = rng.integers(0, 256, (60, 80, 3), dtype=np.uint8)
    u = rng.uniform(-3, 83, (130, 211))           # the readout grid's shape; points outside the image included
    v = rng.uniform(-3, 63, (130, 211))
    u[0, :64] = 5 + (np.arange(64) + 0.5) / 32    # exact half steps of 1/32 px: rounding half to even
    v[1, :64] = 5 + (np.arange(64) + 0.5) / 32
    assert np.array_equal(remap_linear(im, u, v), model(im, u, v))


def test_remap_linear_on_an_e1_frame():
    im = cv2.imread(str(sorted(FRAMES.glob("frame_*.jpg"))[0]), cv2.IMREAD_COLOR)
    rng = np.random.default_rng(7)
    u = 300 + 0.989 * np.arange(211)[None, :] + rng.uniform(0, 1, (130, 1))   # ~1 px per 0.1 mm, as on E1
    v = 200 + 0.989 * np.arange(130)[:, None] + rng.uniform(0, 1, (1, 211))
    got = remap_linear(im, u, v)
    assert got.shape == (130, 211, 3) and got.dtype == np.uint8
    assert np.array_equal(got, model(im, u, v))
