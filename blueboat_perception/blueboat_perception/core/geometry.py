"""Pixel -> world and the per-row / per-side geometry of a seabed image.

One function, two conventions, read from the image's ``georef``:

* column ``j`` of width ``W``: ``k = W/2 − 1 − j`` (port, ``j < W/2``) or
  ``j − W/2`` (starboard); slant ``s = (k + 0.5)·bin_pitch_m``;
* **convention 1** (every session before the Q6 fix, ``georef`` absent):
  ``y = ±sqrt(max(s² − altitude_m², 0))``;
* **convention 2** (Q6): ``y = ±(lever_side + sqrt(max(s² − bottom_m², 0)))``;
* both: ``wx = x − sin(yaw)·y``, ``wy = y_row + cos(yaw)·y`` -- the
  flat-seabed projection (Blondel 2009 eq. 3.2; Burguera & Oliver 2016
  eq. 2 with hp = 0).

The grids are computed with exactly the operations, shapes and dtypes of
the GCS (``models/sonar.py`` ``slant_col_to_y_local`` and
``seabed_imager.waterfall_pixel_to_world``: float64 throughout, the row
pose as an (H, 1) column, cast to float32 at the end), so they reproduce
the station's ``_world.npz`` grids bit for bit on the same numpy. A pixel's
world position is a lookup in those grids, as the station's own dummy did.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from .image import SeabedImageInput

TWO_PI = 2.0 * math.pi


def wrap_pi(a):
    """Wrap angle(s) to (−π, π]."""
    a = np.asarray(a, dtype=np.float64)
    w = np.mod(a + math.pi, TWO_PI) - math.pi
    w = np.where(w == -math.pi, math.pi, w)
    return w if w.ndim else float(w)


def column_slant(width: int, pitch_m: float) -> Tuple[np.ndarray, np.ndarray]:
    """(is_port[W], slant_m[W]) of every column (GCS layout)."""
    col = np.arange(width, dtype=np.float64)
    half = width / 2.0
    port = col < half
    k = np.where(port, half - 1.0 - col, col - half)
    return port, (k + 0.5) * pitch_m


def row_altitude(img: SeabedImageInput) -> np.ndarray:
    """The altitude the image's convention uses per row (>= 0)."""
    h = img.rows.bottom if img.georef.altitude_key == "bottom_m" else img.rows.altitude
    return np.maximum(np.asarray(h, np.float64), 0.0)


def lateral_offset(img: SeabedImageInput, slant, port, h) -> np.ndarray:
    """Signed lateral offset y (+ port) of slant ranges under the
    image's convention; broadcasting ``slant``/``port`` against ``h``."""
    ground = np.sqrt(np.maximum(slant * slant - np.square(h), 0.0))
    if img.georef.convention >= 2:
        return np.where(port, img.georef.lever_y_port_m + ground,
                        -(img.georef.lever_y_stbd_m + ground))
    return np.where(port, ground, -ground)


def world_grids(img: SeabedImageInput) -> Tuple[np.ndarray, np.ndarray]:
    """Per-pixel (world_x, world_y), float32 [H, W] -- the station's grids."""
    H, W = img.shape
    port, s = column_slant(W, img.bin_pitch_m)
    h = row_altitude(img)[:, None]
    y_local = lateral_offset(img, s[None, :], port[None, :], h)
    x = img.rows.x[:, None]
    y = img.rows.y[:, None]
    yaw = img.rows.yaw[:, None]
    wx = x - np.sin(yaw) * y_local
    wy = y + np.cos(yaw) * y_local
    return wx.astype(np.float32), wy.astype(np.float32)


def ground_range(img: SeabedImageInput, slant: float, h: float, port: bool) -> float:
    """|y| of one slant range: the ground range from the boat's centre
    line (the lever arm included under convention 2)."""
    return float(abs(lateral_offset(img, np.float64(slant), bool(port), np.float64(h))))


def look_azimuths(yaw: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """(port, starboard) look directions, boat -> seabed, ENU CCW from +x."""
    yaw = np.asarray(yaw, np.float64)
    return wrap_pi(yaw + math.pi / 2.0), wrap_pi(yaw - math.pi / 2.0)


def grazing_angle(slant: float, h: float) -> Optional[float]:
    """Flat-floor grazing angle ``asin(min(h/s, 1))`` (Burguera & Oliver
    2016 eq. 1); None for a non-positive slant."""
    if not slant > 0.0:
        return None
    return float(math.asin(min(max(h, 0.0) / slant, 1.0)))


def derived_yaw_rate(t: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    """Finite-difference yaw rate of the rows (unwrapped yaw over row time);
    NaN where the rows' times do not advance."""
    n = t.size
    if n < 2:
        return np.full(n, np.nan)
    order = np.argsort(t, kind="stable")
    ts, ys = t[order], np.unwrap(yaw[order])
    with np.errstate(divide="ignore", invalid="ignore"):
        dt = np.diff(ts)
        if np.any(dt <= 0.0):
            rate_sorted = np.full(n, np.nan)
            good = dt > 0.0
            d = np.where(good, np.diff(ys) / np.where(good, dt, 1.0), np.nan)
            rate_sorted[:-1] = d
            rate_sorted[-1] = d[-1]
        else:
            rate_sorted = np.gradient(ys, ts)
    out = np.empty(n)
    out[order] = rate_sorted
    return out


def smooth_in_time(t: np.ndarray, w: np.ndarray, window_s: float) -> np.ndarray:
    """Mean of ``w`` over the rows within ``±window_s/2`` of each row's time
    (rows in any order; NaN ignored, NaN where a window holds nothing)."""
    t = np.asarray(t, np.float64)
    w = np.asarray(w, np.float64)
    if t.size == 0 or window_s <= 0.0:
        return w.copy()
    order = np.argsort(t, kind="stable")
    ts, ws = t[order], w[order]
    fin = np.isfinite(ws)
    cs = np.concatenate([[0.0], np.cumsum(np.where(fin, ws, 0.0))])
    cn = np.concatenate([[0], np.cumsum(fin)])
    lo = np.searchsorted(ts, ts - window_s / 2.0, "left")
    hi = np.searchsorted(ts, ts + window_s / 2.0, "right")
    n = cn[hi] - cn[lo]
    mean = np.where(n > 0, (cs[hi] - cs[lo]) / np.maximum(n, 1), np.nan)
    out = np.empty_like(mean)
    out[order] = mean
    return out


def turn_geometry(speed: np.ndarray, yaw_rate: np.ndarray, range_m: np.ndarray):
    """Per row: (turn radius r* = v/|ω| [m] or NaN, inner side +1 port / −1
    starboard / 0, backscan start [m] = r* when r* < range else NaN).

    A seabed point on the inner side at lateral distance y moves along-track at
    ``v − ω·y``, so it is scanned backwards beyond ``y = v/ω`` (Cobra et al. 1992
    §I.A, backscan). Positive ω (ENU, CCW) turns left: the inner side is port."""
    v = np.asarray(speed, np.float64)
    w = np.asarray(yaw_rate, np.float64)
    ok = np.isfinite(v) & np.isfinite(w) & (v > 0.05) & (np.abs(w) > 1e-3)
    rstar = np.where(ok, v / np.where(ok, np.abs(w), 1.0), np.nan)
    side = np.where(ok, np.sign(w), 0.0).astype(np.int64)
    back = np.where(ok & (rstar < np.asarray(range_m, np.float64)), rstar, np.nan)
    return rstar, side, back


def side_spans(img: SeabedImageInput) -> Dict[str, Dict[str, np.ndarray]]:
    """Per row and side, the valid-seabed slant span.

    Valid seabed = a finite sample at a slant range not below the
    convention's altitude. Returns, per side, ``near``/``far`` slant [m]
    (NaN where the row has no valid sample on that side) and ``present``
    (the side has at least one finite sample in that row)."""
    H, W = img.shape
    port, s = column_slant(W, img.bin_pitch_m)
    h = row_altitude(img)[:, None]
    finite = np.isfinite(img.db)
    valid = finite & (s[None, :] >= h)
    out = {}
    for name, mask in (("port", port), ("starboard", ~port)):
        v = valid[:, mask]
        ss = np.broadcast_to(s[mask][None, :], v.shape)
        near = np.where(v, ss, np.inf).min(axis=1)
        far = np.where(v, ss, -np.inf).max(axis=1)
        has = v.any(axis=1)
        out[name] = {"near": np.where(has, near, np.nan),
                     "far": np.where(has, far, np.nan),
                     "present": finite[:, mask].any(axis=1)}
    return out


def edge_points(img: SeabedImageInput, rows_idx: List[int], slant: np.ndarray,
                port: bool) -> List[List[float]]:
    """World points of one slant range per row (NaN rows skipped)."""
    pts = []
    h = row_altitude(img)
    for i in rows_idx:
        si = slant[i]
        if not np.isfinite(si):
            continue
        yl = float(lateral_offset(img, np.float64(si), bool(port), np.float64(h[i])))
        yaw = img.rows.yaw[i]
        pts.append([float(img.rows.x[i] - math.sin(yaw) * yl),
                    float(img.rows.y[i] + math.cos(yaw) * yl)])
    return pts


def decimated_rows(n: int, step: int) -> List[int]:
    """Every ``step``-th row, first and last always kept."""
    if n <= 0:
        return []
    step = max(int(step), 1)
    idx = list(range(0, n, step))
    if idx[-1] != n - 1:
        idx.append(n - 1)
    return idx
