"""Merging the boxes of overlapping tiles into one list per picture.

Tiles overlap (``tiling.py``), so one object is seen in up to three tiles,
whole in one and cut by a tile edge in its neighbours. A highlight + shadow
box can also be wider than the overlap, then every tile holds only part of
it. Two rules, per class, both deterministic:

* ``nms`` -- plain non-maximum suppression: the highest-scored box of a
  group whose IoU with it is ``>= threshold`` is kept, the others dropped.
  This is the SAHI paper's merge (Akyon et al. 2022 §3, "boxes having higher
  IoU ratios than a predefined matching threshold Tm are matched", Tm = 0.5
  in §4). A box cut by a tile edge can have a low IoU with the whole box
  and survive as a duplicate;
* ``greedy_nmm`` -- greedy non-maximum *merging*, as in the SAHI software
  (``sahi/postprocess``, commit b5a153e, ``greedy_nmm_from_matrix`` and
  ``_apply_merge``; not described in the paper, so ``[engineering]``):
  boxes are visited by descending score; each unassigned box keeps every
  lower box of its class whose overlap with it is ``>= threshold`` (metric
  ``ios`` = intersection over the smaller box, by default, so a cut piece
  inside the whole box matches fully); the kept box then absorbs them one by
  one, each merge re-checking the overlap against the growing box. The
  merged box is the union; its **score is the maximum** of the group (the
  SAHI rule ``get_merged_score``) -- a written rule, not an average, so a
  merge never raises a score above what one tile saw.

Which rule the YOLO adapter uses is a configuration value; the comparison on
held-out simulated worlds is ``tools/evaluate_detector.py`` and the decision
is recorded in ``docs/DESIGN.md``.

**Order.** Inputs are sorted by ``(-score, x0, y0, class, tile, x1, y1)``
and outputs by the same key, so the detection index ``k`` (and with it the
station uid) is stable for the same input.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Dict, List, Sequence


@dataclass(frozen=True)
class Box:
    """A box in continuous picture pixels (pixel j spans [j, j + 1))."""

    x0: float
    y0: float
    x1: float
    y1: float
    score: float
    class_id: int
    tile: int = -1

    @property
    def area(self) -> float:
        return max(self.x1 - self.x0, 0.0) * max(self.y1 - self.y0, 0.0)


def sort_key(b: Box):
    return (-b.score, b.x0, b.y0, b.class_id, b.tile, b.x1, b.y1)


def ordered(boxes: Sequence[Box]) -> List[Box]:
    return sorted(boxes, key=sort_key)


def intersection(a: Box, b: Box) -> float:
    w = min(a.x1, b.x1) - max(a.x0, b.x0)
    h = min(a.y1, b.y1) - max(a.y0, b.y0)
    return w * h if w > 0 and h > 0 else 0.0


def iou(a: Box, b: Box) -> float:
    inter = intersection(a, b)
    union = a.area + b.area - inter
    return inter / union if union > 0 else 0.0


def ios(a: Box, b: Box) -> float:
    inter = intersection(a, b)
    small = min(a.area, b.area)
    return inter / small if small > 0 else 0.0


METRICS = {"iou": iou, "ios": ios}


def _by_class(boxes: Sequence[Box]) -> Dict[int, List[Box]]:
    out: Dict[int, List[Box]] = {}
    for b in ordered(boxes):
        out.setdefault(b.class_id, []).append(b)
    return out


def merge_nms(boxes: Sequence[Box], threshold: float = 0.5,
              metric: str = "iou") -> List[Box]:
    """Per-class NMS: keep the best box of every overlapping group."""
    m = METRICS[metric]
    kept: List[Box] = []
    for group in _by_class(boxes).values():
        keep_c: List[Box] = []
        for b in group:
            if all(m(k, b) < threshold for k in keep_c):
                keep_c.append(b)
        kept.extend(keep_c)
    return ordered(kept)


def merge_greedy_nmm(boxes: Sequence[Box], threshold: float = 0.5,
                     metric: str = "ios") -> List[Box]:
    """Per-class greedy non-maximum merging (module docstring)."""
    m = METRICS[metric]
    out: List[Box] = []
    for group in _by_class(boxes).values():
        taken = [False] * len(group)
        for i, keep in enumerate(group):
            if taken[i]:
                continue
            taken[i] = True
            members = []
            for j in range(i + 1, len(group)):
                if not taken[j] and m(keep, group[j]) >= threshold:
                    taken[j] = True
                    members.append(group[j])
            merged = keep
            for b in members:
                if m(merged, b) >= threshold:
                    merged = replace(merged, x0=min(merged.x0, b.x0), y0=min(merged.y0, b.y0),
                                     x1=max(merged.x1, b.x1), y1=max(merged.y1, b.y1),
                                     score=max(merged.score, b.score))
            out.append(merged)
    return ordered(out)


MERGES = {"nms": merge_nms, "greedy_nmm": merge_greedy_nmm}
#: Default overlap metric of each rule.
DEFAULT_METRIC = {"nms": "iou", "greedy_nmm": "ios"}


def merge(boxes: Sequence[Box], rule: str = "nms", threshold: float = 0.5,
          metric: str = "") -> List[Box]:
    try:
        fn = MERGES[rule]
    except KeyError:
        raise ValueError(f"unknown merge rule {rule!r}; known: {sorted(MERGES)}")
    return fn(boxes, threshold, metric or DEFAULT_METRIC[rule])
