"""Detector protocol and the raw detection a detector returns."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Protocol, Tuple

from ..preprocess import DetectorInput


@dataclass(frozen=True)
class RawDetection:
    """One detector output, in pixels, before calibration and georeferencing.

    ``bbox_px`` = (r0, c0, r1, c1), inclusive; ``center_px`` = (row, col).
    ``score_raw`` is the detector's own score -- for a YOLO-type detector
    ``Pr(Object)·IOU``, **not a probability** (Redmon et al. 2016 §2)."""

    bbox_px: Tuple[int, int, int, int]
    center_px: Tuple[int, int]
    class_id: int
    class_name: str
    score_raw: float
    logit: Optional[float] = None


class Detector(Protocol):
    name: str
    version: str
    is_dummy: bool

    def detect(self, inp: DetectorInput) -> List[RawDetection]: ...
