"""sensor_msgs/Image <-> BGR numpy, without cv_bridge (avoids its numpy/OpenCV ABI issues)."""
import numpy as np

_CHANNELS = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4, "mono8": 1}


def image_to_bgr(msg):
    """Convert a ZED/ROS Image message (bgra8 is the ZED default) to an HxWx3 uint8 BGR array."""
    enc = msg.encoding.lower()
    if enc not in _CHANNELS:
        raise ValueError(f"unsupported image encoding '{msg.encoding}'")
    ch = _CHANNELS[enc]
    h, w = int(msg.height), int(msg.width)
    if h <= 0 or w <= 0:
        raise ValueError("image width and height must be positive")
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    step = int(msg.step) or w * ch
    if step < w * ch:
        raise ValueError("image step smaller than width*channels")
    if raw.size < h * step:
        raise ValueError("image buffer smaller than height*step")
    arr = raw[: h * step].reshape(h, step)[:, : w * ch].reshape(h, w, ch)
    if enc in ("bgr8", "bgra8"):
        out = arr[:, :, :3]
    elif enc in ("rgb8", "rgba8"):
        out = arr[:, :, [2, 1, 0]]
    else:  # mono8
        out = np.repeat(arr, 3, axis=2)
    return np.ascontiguousarray(out)


def fill_bgr_image(msg, frame, header):
    """Populate an (empty) sensor_msgs/Image with a BGR frame."""
    msg.header = header
    msg.height, msg.width = int(frame.shape[0]), int(frame.shape[1])
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(frame, dtype=np.uint8).tobytes()
    return msg
