"""Merging overlapping tiles' boxes (core/merge.py)."""

from __future__ import annotations

import random

import pytest

from blueboat_perception.core.merge import (Box, ios, iou, merge, merge_greedy_nmm,
                                            merge_nms, ordered)


def B(x0, x1, score=0.9, cls=1, tile=0, y0=10.0, y1=60.0):
    return Box(float(x0), float(y0), float(x1), float(y1), score, cls, tile)


def test_metrics():
    a, b = B(0, 10), B(5, 15)
    assert iou(a, b) == pytest.approx(5 / 15)
    assert ios(a, b) == pytest.approx(0.5)
    assert ios(B(0, 100), B(10, 20)) == pytest.approx(1.0)
    assert iou(B(0, 10), B(20, 30)) == 0.0


def test_a_box_split_across_two_tiles_becomes_one():
    # object [400, 620): tile A [256, 512) sees [400, 512), tile B [384, 640) all of it,
    # tile C [512, 768) sees [512, 620)
    whole = B(400, 620, 0.80, tile=1)
    left, right = B(400, 512, 0.60, tile=0), B(512, 620, 0.70, tile=2)
    out = merge_greedy_nmm([left, whole, right])
    assert len(out) == 1
    assert (out[0].x0, out[0].x1, out[0].score) == (400.0, 620.0, 0.80)
    # NMS keeps the right piece: IoU 108/220 < 0.5
    nms = merge_nms([left, whole, right])
    assert len(nms) == 2


def test_two_partial_halves_without_a_whole_box_are_joined():
    # a box wider than the overlap: no tile holds it whole
    a, b = B(300, 512, 0.5, tile=0), B(384, 700, 0.6, tile=1)
    out = merge_greedy_nmm([a, b])
    assert len(out) == 1 and (out[0].x0, out[0].x1, out[0].score) == (300.0, 700.0, 0.6)


def test_different_classes_never_merge_and_far_boxes_stay():
    out = merge_greedy_nmm([B(0, 50, cls=1), B(0, 50, cls=2), B(500, 550, cls=1)])
    assert len(out) == 3
    assert len(merge_nms([B(0, 50, cls=1), B(0, 50, cls=2)])) == 2


def test_merged_score_is_the_maximum():
    out = merge_greedy_nmm([B(0, 100, 0.3), B(10, 90, 0.9), B(20, 80, 0.5)])
    assert len(out) == 1 and out[0].score == 0.9


def test_merge_rechecks_against_the_growing_box():
    # SAHI's _apply_merge: each member must still match the merged box
    # (with IoU; under IoS the merged box only ever matches better)
    keep = B(0, 100, 0.9, y0=0, y1=100)
    m1 = B(0, 100, 0.8, y0=0, y1=200)        # IoU(keep, m1) = 0.5 -> member
    m2 = B(0, 100, 0.7, y0=-100, y1=100)     # IoU(keep, m2) = 0.5 -> member
    out = merge_greedy_nmm([keep, m1, m2], metric="iou")
    # after absorbing m1 the box is [0, 200): IoU with m2 = 1/3, so m2 is not
    # absorbed (and, being assigned to this group, not kept on its own either)
    assert len(out) == 1 and (out[0].y0, out[0].y1) == (0.0, 200.0)


def test_output_order_is_deterministic_and_independent_of_input_order():
    rng = random.Random(3)
    boxes = [B(rng.uniform(0, 2000), 0, rng.random(), rng.randint(0, 3), rng.randint(0, 17))
             for _ in range(60)]
    boxes = [Box(b.x0, b.y0, b.x0 + rng.uniform(5, 300), b.y1, b.score, b.class_id, b.tile)
             for b in boxes]
    ref = merge(boxes, "greedy_nmm")
    ref_nms = merge(boxes, "nms")
    for _ in range(5):
        rng.shuffle(boxes)
        assert merge(boxes, "greedy_nmm") == ref
        assert merge(boxes, "nms") == ref_nms
    assert ref == ordered(ref)


def test_unknown_rule():
    with pytest.raises(ValueError):
        merge([], "soft")
    assert merge([], "nms") == [] and merge([], "greedy_nmm") == []
