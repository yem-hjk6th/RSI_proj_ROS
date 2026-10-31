"""sensor_msgs/Image payload -> BGR numpy array, without cv_bridge and without ROS imports (duck-typed message)."""
import numpy as np

_CHANNELS = {"bgr8": (3, (0, 1, 2)), "rgb8": (3, (2, 1, 0)), "bgra8": (4, (0, 1, 2)), "rgba8": (4, (2, 1, 0))}


def image_to_bgr(msg) -> np.ndarray:
    """Image with encoding bgr8, rgb8, bgra8 or rgba8 -> HxWx3 uint8 BGR array (row padding in step is dropped)."""
    if msg.encoding not in _CHANNELS:
        raise ValueError(f"unsupported image encoding {msg.encoding!r}")
    ch, order = _CHANNELS[msg.encoding]
    a = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch]
    return np.ascontiguousarray(a.reshape(msg.height, msg.width, ch)[..., list(order)])
