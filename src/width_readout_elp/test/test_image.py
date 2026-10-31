from types import SimpleNamespace

import numpy as np
import pytest

from width_readout_elp.image import image_to_bgr


def _msg(a, encoding, pad=0):
    h, w, c = a.shape
    rows = np.concatenate([a.reshape(h, w * c), np.zeros((h, pad), np.uint8)], 1)
    return SimpleNamespace(height=h, width=w, step=w * c + pad, encoding=encoding, data=rows.tobytes())


@pytest.mark.parametrize("pad", [0, 5])
def test_encodings(pad):
    bgr = np.random.default_rng(0).integers(0, 256, (4, 6, 3), dtype=np.uint8)
    alpha = np.full((4, 6, 1), 255, np.uint8)
    assert np.array_equal(image_to_bgr(_msg(bgr, "bgr8", pad)), bgr)
    assert np.array_equal(image_to_bgr(_msg(bgr[..., ::-1].copy(), "rgb8", pad)), bgr)
    assert np.array_equal(image_to_bgr(_msg(np.concatenate([bgr, alpha], 2), "bgra8", pad)), bgr)
    assert np.array_equal(image_to_bgr(_msg(np.concatenate([bgr[..., ::-1], alpha], 2), "rgba8", pad)), bgr)


def test_unsupported():
    with pytest.raises(ValueError):
        image_to_bgr(_msg(np.zeros((2, 2, 3), np.uint8), "mono8"))
