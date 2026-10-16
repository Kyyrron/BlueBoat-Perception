"""Message <-> plain data conversions (the node's only ROS-typed helpers).

Contract: ``docs/topics.md`` §1 (images), §3 (``Detection2DArray``).
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

FRAME = "odom"   # the world frame of /blueboat/odom (a label: no module reads TF)


def stamp_from_seconds(t: float) -> Tuple[int, int]:
    """(sec, nanosec) of a float time in seconds (>= 0)."""
    if not math.isfinite(t) or t <= 0.0:
        return 0, 0
    sec = int(math.floor(t))
    nsec = int(round((t - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec, nsec = sec + 1, nsec - 1_000_000_000
    return sec, nsec


def image_to_array(msg: Image) -> np.ndarray:
    """``sensor_msgs/Image`` (32FC1 or mono8, little-endian) -> numpy [H, W]."""
    if msg.encoding == "32FC1":
        dtype, size = np.dtype("<f4") if not msg.is_bigendian else np.dtype(">f4"), 4
    elif msg.encoding == "mono8":
        dtype, size = np.dtype(np.uint8), 1
    else:
        raise ValueError(f"unsupported encoding {msg.encoding!r}")
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if buf.size < h * step or step < w * size:
        raise ValueError(f"image buffer {buf.size} B too small for {h}x{w} step {step}")
    rows = buf[: h * step].reshape(h, step)[:, : w * size]
    arr = np.ascontiguousarray(rows).view(dtype).reshape(h, w)
    return arr.astype(np.float32 if size == 4 else np.uint8, copy=False)


def detections_to_msg(detections: Optional[List[dict]], t_image_s: float) -> Detection2DArray:
    """One ``Detection2DArray`` per image (possibly empty)."""
    out = Detection2DArray()
    out.header.stamp.sec, out.header.stamp.nanosec = stamp_from_seconds(t_image_s)
    out.header.frame_id = FRAME
    for d in detections or []:
        m = Detection2D()
        m.header.stamp.sec, m.header.stamp.nanosec = stamp_from_seconds(d["t_row_s"])
        m.header.frame_id = FRAME
        m.id = str(d["uid"])
        x, y = d["world_xy"]
        m.bbox.center.position.x = float(x)
        m.bbox.center.position.y = float(y)
        m.bbox.center.theta = float(d["boat_heading_rad"])
        m.bbox.size_x = float(d["extent_m"][0])
        m.bbox.size_y = float(d["extent_m"][1])
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(d["class_name"])
        hyp.hypothesis.score = float(d["confidence"])
        hyp.pose.pose.position.x = float(x)
        hyp.pose.pose.position.y = float(y)
        hyp.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        (sxx, sxy), (syx, syy) = d["covariance_xy"]
        cov[0], cov[1], cov[6], cov[7] = float(sxx), float(sxy), float(syx), float(syy)
        hyp.pose.covariance = cov
        m.results.append(hyp)
        out.detections.append(m)
    return out
