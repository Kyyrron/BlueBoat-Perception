"""YOLO detector -- interface only (no ML runtime on this machine).

Constraints any adapter must respect (Redmon et al. 2016, read in full):

* YOLO resizes its input to a fixed square (fig. 1); our tiles are
  256 x W with W up to 2402, so the adapter must **tile across-track**
  (never squash) and merge boxes across tiles (NMS);
* boxes are YOLO-normalised ``xywh`` (§2.2);
* the score is ``Pr(Object)·IOU`` or ``Pr(Class)·IOU`` (§2, eq. 1), not a
  calibrated probability -- hence the calibration stage
  (``core/calibration.py``);
* it is weaker on small objects and on groups (§2.4).

Choosing a runtime (torch / ultralytics / onnxruntime) is a later
decision (``.claude/TODO.md``).
"""

from __future__ import annotations

from typing import List, Optional

from ..preprocess import DetectorInput
from .base import RawDetection


class YoloDetector:
    name = "yolo"
    version = "0"
    is_dummy = False

    def __init__(self, model_path: Optional[str] = None, input_size: int = 640,
                 tile_stride: Optional[int] = None, score_threshold: float = 0.25,
                 nms_iou: float = 0.5) -> None:
        self.model_path = model_path
        self.input_size = int(input_size)
        self.tile_stride = tile_stride
        self.score_threshold = float(score_threshold)
        self.nms_iou = float(nms_iou)

    def detect(self, inp: DetectorInput) -> List[RawDetection]:
        raise NotImplementedError(
            "YoloDetector is an interface stub: no ML runtime is installed "
            "(see BlueBoat-Perception/.claude/TODO.md)")
