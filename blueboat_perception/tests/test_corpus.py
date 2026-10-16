"""The station's own conversion and its own dummy, on every on-disk session.

* convention 1: Perception's grids equal the ``world_x``/``world_y`` of
  each ``_world.npz`` the GCS wrote (tolerance 1 float32 ulp; the exact
  share is asserted to be 100 % on this machine's numpy);
* the dummy: wherever the station's in-process ``dummy_center_analyzer``
  ran (live sessions: non-empty ``detections``), Perception's dummy
  reproduces pixel, world, class, confidence and row time exactly.

Read-only; skips without a data directory.
"""

from __future__ import annotations

import numpy as np
import pytest

from blueboat_perception.core import geometry as geo
from blueboat_perception.core.image import SeabedImageInput
from blueboat_perception.core.pipeline import Pipeline
from blueboat_perception.core.session import list_images
from conftest import corpus_image_dirs

DIRS = corpus_image_dirs()
pytestmark = pytest.mark.skipif(not DIRS, reason="no GCS sessions on this machine")


def _ulp(a, b):
    return int(np.max(np.abs(a.view(np.int32).astype(np.int64)
                             - b.view(np.int32).astype(np.int64)))) if a.size else 0


@pytest.mark.parametrize("folder", DIRS, ids=lambda p: p.parent.name + "/" + p.name)
def test_grids_equal_the_stations(folder):
    files = list_images(folder)
    exact = 0
    for f in files:
        img = SeabedImageInput.from_session_files(f.json, f.npz, f.png, stream_id="corpus")
        with np.load(f.npz) as z:
            gx, gy = z["world_x"], z["world_y"]
        wx, wy = geo.world_grids(img)
        assert _ulp(wx, gx) <= 1 and _ulp(wy, gy) <= 1, f.json
        exact += int(np.array_equal(wx, gx) and np.array_equal(wy, gy))
    assert exact == len(files), f"{exact}/{len(files)} bit-exact"


@pytest.mark.parametrize("folder", DIRS, ids=lambda p: p.parent.name + "/" + p.name)
def test_dummy_equals_the_stations_dummy(folder):
    pipeline = Pipeline()
    compared = 0
    for f in list_images(folder):
        img = SeabedImageInput.from_session_files(f.json, f.npz, f.png, stream_id="corpus")
        old = img.meta.get("detections") or []
        if not old:
            continue                    # offline export: the analyzer did not run
        _, dets = pipeline.run(img)
        assert len(dets) == len(old) == 1
        o, d = old[0], dets[0]
        assert d["pixel"] == o["pixel"]
        assert d["world_xy"] == o["world"]
        assert d["class_name"] == o["class_name"] == "dummy_center"
        assert d["confidence"] == o["confidence"] == 0.5
        assert d["t_row_s"] == o["t_s"]
        compared += 1
    if compared == 0:
        pytest.skip("no in-process detections stored in this folder")
