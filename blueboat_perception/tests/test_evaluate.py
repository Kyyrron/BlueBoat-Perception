"""The evaluation tool (tools/evaluate_detector.py): AP, matching, ignore
regions, and one run through the real pipeline."""

from __future__ import annotations

import importlib.util
import json
import sys

import numpy as np
import pytest

from conftest import PKG_ROOT

TOOLS = PKG_ROOT.parent / "tools"
NEEDS = [m for m in ("cv2", "yaml") if importlib.util.find_spec(m) is None]
pytestmark = pytest.mark.skipif(bool(NEEDS), reason=f"needs {NEEDS}")


def _ev():
    if str(TOOLS) not in sys.path:
        sys.path.insert(0, str(TOOLS))
    import evaluate_detector
    return evaluate_detector


def G(c0, c1, cls=0, ignore=False, slant=5.0):
    return {"box": [c0, 0.0, c1, 50.0], "class_id": cls, "ignore": ignore, "slant_m": slant}


def D(c0, c1, score, cls=0):
    return {"box": [c0, 0.0, c1, 50.0], "class_id": cls, "score": score}


def test_average_precision():
    ev = _ev()
    assert ev.average_precision([0.9, 0.8], [1, 1], 2) == pytest.approx(1.0)
    assert ev.average_precision([], [], 3) == 0.0
    assert np.isnan(ev.average_precision([0.5], [0], 0))
    # one hit then one miss at rank 2, 2 GT: recall 0.5 at precision 1 -> 51/101
    assert ev.average_precision([0.9, 0.8], [1, 0], 2) == pytest.approx(51 / 101)


def test_matching_with_an_ignore_region():
    ev = _ev()
    out, used, real = ev.match_picture([D(0, 100, 0.9), D(500, 600, 0.8), D(900, 950, 0.7)],
                                       [G(0, 100), G(500, 600, ignore=True)], 0.5)
    assert list(out) == [1, -1, 0] and list(used) == [True] and len(real) == 1


def test_evaluate_metrics():
    ev = _ev()
    pics = [{"dets": [D(0, 100, 0.9), D(300, 350, 0.05)],
             "gts": [G(0, 100, slant=4.0), G(300, 350, slant=12.0)], "latency_ms": 10.0},
            {"dets": [D(700, 760, 0.6)], "gts": [], "latency_ms": 20.0}]
    m = ev.evaluate(pics, ["a", "b"], [0, 5, 10, 20], 0.10)
    bands = m["operating_point"]["recall_by_slant_range_m"]
    assert bands[0]["recall"] == 1.0 and bands[2]["recall"] == 0.0 and bands[1]["n_gt"] == 0
    assert m["operating_point"]["false_detections_per_picture"]["mean"] == 0.5
    assert m["per_class"]["a"]["n_gt"] == 2 and 0 < m["ap50"] < 1
    assert m["latency_ms_per_picture"]["max"] == 20.0


def test_a_run_through_the_pipeline(tmp_path):
    ev = _ev()
    import test_tile_dataset as T
    db, boxes = T.scene()
    ds = T.make_dataset(tmp_path / "root", "w", boxes, db, world="/worlds/world_t")
    split = T.split_file(tmp_path, {"test": ["world_t"]})
    rc = ev.main([str(ds), "--split", str(split), "--on", "test", "--name", "e",
                  "--detector", "dummy_center", "--data-dir", str(tmp_path / "root")])
    assert rc == 0
    out = tmp_path / "root" / "data" / "Perception_eval" / "e"
    r = json.loads((out / "results.json").read_text())
    assert r["held_out_report_only"] is True and "simulated data" in r["statement"]
    assert r["metrics"]["gt_boxes"] == 1 and r["metrics"]["ignored_boxes"] == 1
    assert r["metrics"]["ap50"] == 0.0                         # the dummy finds nothing
    assert len((out / "per_picture.jsonl").read_text().splitlines()) == 1
    assert ev.main([str(ds), "--split", str(split), "--on", "val", "--name", "e2",
                    "--detector", "dummy_center", "--data-dir", str(tmp_path / "root")]) == 2
