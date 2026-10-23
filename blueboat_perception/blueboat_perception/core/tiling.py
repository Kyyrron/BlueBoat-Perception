"""Across-track tiling of the detector-input picture (decided 2026-10-23).

A YOLO model takes a square input; a seabed picture is 256 rows × W slant
bins (W = 1200–2402). Squeezing it into one square would shrink a 100 px
object to ~27 px and fill ~89 % of the input with padding, so the picture is
cut into square tiles at **native resolution** (never resized):

* tile = ``tile`` × ``tile`` px (256), spanning the whole picture height;
  tiling is across track only (the GCS already overlaps consecutive pictures
  by 50 % along track);
* starts ``0, S, 2S, …`` while ``start + T ≤ W``, plus one final tile at
  ``W − T`` when the last regular tile does not reach ``W``, so no column is
  lost and every tile is ``T`` wide. With T = 256, S = 128 any interval up
  to ``T − S + 1`` = 129 px wide lies whole inside at least one tile;
* the nadir stays in the tiles: the strip is one continuous row of columns;
* a picture narrower than ``T`` is padded on the right, a picture shorter
  than ``T`` (the last of a run, 2–255 rows) at the bottom -- the oldest
  rows, row 0 being the newest -- with invalid zeros (``valid`` False);
* a tile is skipped only when its valid share is below
  ``min_valid_fraction`` (essentially empty); the nadir band is valid data
  and never a reason to skip. Every skip is counted.

The same functions cut the training tiles (``tools/make_tile_dataset.py``)
and the inference tiles (``detectors/yolo.py``), from the same grey picture
(``preprocess.FixedGray``), so training, live, offline and dataset mode all
see one function. Pure, deterministic, numpy + stdlib only.

Slicing images into overlapping patches for detection: Akyon et al. 2022
(SAHI) §3; Yu et al. 2021 §2.1 (320 × 320 windows sliding across track,
40 % overlap); Ünel et al. 2019 §4.2 (overlapping tiles in training and
inference) -- REFERENCES R13, R23, R25.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class TilingConfig:
    """One configuration shared by training and inference."""

    tile: int = 256
    stride: int = 128
    #: A tile whose valid share (over its T × T pixels) is below this is
    #: skipped. 0.001 ≈ 66 px: "essentially no valid pixels".
    min_valid_fraction: float = 0.001

    def __post_init__(self) -> None:
        if int(self.tile) <= 0:
            raise ValueError("tile must be > 0")
        if not 0 < int(self.stride) <= int(self.tile):
            raise ValueError("stride must be in (0, tile]")
        if not 0.0 <= float(self.min_valid_fraction) < 1.0:
            raise ValueError("min_valid_fraction must be in [0, 1)")

    def to_json(self) -> Dict[str, Any]:
        return {"tile": int(self.tile), "stride": int(self.stride),
                "min_valid_fraction": float(self.min_valid_fraction)}


def tile_starts(width: int, tile: int, stride: int) -> Tuple[int, ...]:
    """Column starts of the tiles of a ``width``-wide picture."""
    width, tile, stride = int(width), int(tile), int(stride)
    if width <= 0:
        return ()
    if width <= tile:
        return (0,)
    starts = list(range(0, width - tile + 1, stride))
    if starts[-1] + tile < width:
        starts.append(width - tile)
    return tuple(starts)


@dataclass(frozen=True, eq=False)
class Tile:
    """One square tile of a picture.

    ``c0`` = first picture column; ``cols`` / ``rows`` = picture columns and
    rows it really covers (the rest of ``gray`` / ``valid`` is padding)."""

    index: int
    c0: int
    cols: int
    rows: int
    gray: np.ndarray            # uint8 [T, T]
    valid: np.ndarray           # bool [T, T]
    valid_fraction: float
    skipped: bool


def cut_tiles(gray: np.ndarray, valid: Optional[np.ndarray],
              cfg: TilingConfig = TilingConfig()) -> List[Tile]:
    """Every tile of ``gray`` (uint8 [H, W]), padded to T × T, in column
    order. ``valid`` None = every pixel valid."""
    gray = np.asarray(gray)
    if gray.ndim != 2:
        raise ValueError("gray must be [H, W]")
    H, W = gray.shape
    if valid is None:
        valid = np.ones((H, W), bool)
    valid = np.asarray(valid, bool)
    if valid.shape != (H, W):
        raise ValueError(f"valid {valid.shape} != gray {(H, W)}")
    T = int(cfg.tile)
    rows = min(H, T)
    out: List[Tile] = []
    for i, c0 in enumerate(tile_starts(W, T, cfg.stride)):
        cols = min(T, W - c0)
        g = np.zeros((T, T), np.uint8)
        v = np.zeros((T, T), bool)
        g[:rows, :cols] = gray[:rows, c0:c0 + cols]
        v[:rows, :cols] = valid[:rows, c0:c0 + cols]
        frac = float(np.count_nonzero(v)) / float(T * T)
        out.append(Tile(index=i, c0=int(c0), cols=int(cols), rows=int(rows), gray=g,
                        valid=v, valid_fraction=frac,
                        skipped=frac < float(cfg.min_valid_fraction)))
    return out


def tile_box_to_picture(box_xyxy, tile: Tile) -> Optional[Tuple[float, float, float, float]]:
    """A tile box ``(x0, y0, x1, y1)`` (continuous tile pixels, pixel j spans
    [j, j + 1)) in picture coordinates, clipped to the real part of the tile;
    None when it lies entirely in padding (or is empty after clipping)."""
    x0, y0, x1, y1 = (float(v) for v in box_xyxy)
    x0, x1 = max(x0, 0.0), min(x1, float(tile.cols))
    y0, y1 = max(y0, 0.0), min(y1, float(tile.rows))
    if not (x1 > x0 and y1 > y0):
        return None
    return (x0 + tile.c0, y0, x1 + tile.c0, y1)


def _half_up(v: float) -> int:
    return int(math.floor(v + 0.5))


def to_inclusive_px(box_xyxy, height: int, width: int) -> Tuple[int, int, int, int]:
    """Continuous ``(x0, y0, x1, y1)`` → the inclusive pixel box
    ``(r0, c0, r1, c1)`` the pipeline takes (``RawDetection.bbox_px``):
    edges rounded half up, at least one pixel, clipped to the picture."""
    x0, y0, x1, y1 = (float(v) for v in box_xyxy)
    c0 = min(max(_half_up(x0), 0), width - 1)
    r0 = min(max(_half_up(y0), 0), height - 1)
    c1 = min(max(_half_up(x1) - 1, c0), width - 1)
    r1 = min(max(_half_up(y1) - 1, r0), height - 1)
    return r0, c0, r1, c1
