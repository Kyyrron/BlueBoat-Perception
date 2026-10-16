"""The dummy detector: the GCS's former in-process ``dummy_center_analyzer``.

Since 2026-10-16 the station has one detection source, Perception, so the
dummy lives here. It reproduces the station's dummy exactly: the centre
pixel ``(H // 2, W // 2)``, class ``dummy_center``, score 0.5; the
pipeline georeferences it through the image's convention and stamps it
with its row time, as the station did. Every record and detection it
produces says ``"dummy": true``.
"""

from __future__ import annotations

from typing import List

from ..preprocess import DetectorInput
from .base import RawDetection


class DummyCenter:
    name = "dummy_center"
    version = "1"
    is_dummy = True

    def __init__(self, score: float = 0.5) -> None:
        self.score = float(score)

    def detect(self, inp: DetectorInput) -> List[RawDetection]:
        h, w = inp.db.shape
        row, col = h // 2, w // 2
        return [RawDetection(bbox_px=(row, col, row, col), center_px=(row, col),
                             class_id=0, class_name="dummy_center",
                             score_raw=self.score)]
