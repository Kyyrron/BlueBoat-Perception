"""Label sources for dataset mode.

``LabelSource.labels_for(img) -> list[YoloBox]``. The default is
:class:`NullLabelSource` (no labels): dataset mode writes images with
**empty** label files. A future ground-truth labeller works **offline**
from the dataset's archival ``ground_truth_bag/`` and the world folder,
joined through the per-row device ping numbers -- never online
(the Perception node does not read simulator ground truth).
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
