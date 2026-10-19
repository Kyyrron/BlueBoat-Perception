"""Label sources for dataset mode.

``LabelSource.labels_for(img) -> list[YoloBox]``. Two sources exist:

- :class:`NullLabelSource` (no labels) — the default. Real data, or a dataset
  with no simulator world.
- ``sim_labels.SimGroundTruthLabelSource`` — dataset mode in a simulator world
  (2026-10-16). Boxes come from the simulator's per-ping ground truth, joined
  through each row's device ping numbers. The detection path never sees that
  ground truth: only the dataset writer calls a label source.

Optional methods a source may add, which the dataset writer uses when they are
present:

- ``detail_for(img) -> dict`` — per-box detail for the image sidecar;
- ``ready(img) -> bool`` — whether the ground truth needed by the picture has
  arrived (the node defers the write until it has);
- ``info() -> dict``, ``stats() -> dict`` — for the manifest.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Protocol

from .image import SeabedImageInput


@dataclass(frozen=True)
class YoloBox:
    """One YOLO label: class id and the normalised centre / size."""

    class_id: int
    cx: float
    cy: float
    w: float
    h: float

    def line(self) -> str:
        return f"{self.class_id} {self.cx:.6f} {self.cy:.6f} {self.w:.6f} {self.h:.6f}"


class LabelSource(Protocol):
    name: str
    classes: Dict[int, str]

    def labels_for(self, img: SeabedImageInput) -> List[YoloBox]: ...


class NullLabelSource:
    name = "none"

    def __init__(self) -> None:
        self.classes: Dict[int, str] = {}

    def labels_for(self, img: SeabedImageInput) -> List[YoloBox]:
        return []
