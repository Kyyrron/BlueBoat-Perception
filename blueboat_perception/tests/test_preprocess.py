"""Detector-input preprocessing (decisions 6.1, 6.4, 6.5, 6.6, 6.7).

* the originals are never modified; ``identity`` is the GCS display PNG as is;
* ``fixed_gray`` is deterministic, fixed (no per-image stretch) and the same
  from the wire and from the session files;
* the near-range mask switches on and off independently of the mapping, and
  its boundary column is kept;
* the disabled options raise instead of running.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from blueboat_perception.core import geometry as geo
from blueboat_perception.core.image import SeabedImageInput
from blueboat_perception.core.pipeline import Pipeline, PipelineConfig
from blueboat_perception.core.preprocess import (DEFERRED, FixedGray, Identity,
                                                 load_fixed_gray, make_preprocessor,
                                                 near_range_mask)
from blueboat_perception.core.session import iter_session
from conftest import envelope, make_db, make_metadata, write_session


def _img(png=True, seed=0, **kw):
    meta = make_metadata(**kw)
    db = make_db(meta["rows"], meta["cols"], seed=seed)
    p = np.nan_to_num(np.clip(db, 0, 60) * 4).astype(np.uint8) if png else None
    return SeabedImageInput.from_metadata(meta, db, p)


def test_identity_is_the_display_png_and_touches_nothing():
    img = _img()
    db0, png0 = img.db.copy(), img.png.copy()
    inp = Identity().run(img)
    np.testing.assert_array_equal(inp.gray, img.png)
    np.testing.assert_array_equal(img.db, db0)
    np.testing.assert_array_equal(img.png, png0)
    assert Identity().run(_img(png=False)).gray is None      # no PNG: no input


def test_the_packaged_mapping_is_the_fitted_one():
    m = load_fixed_gray()
    assert m["name"] == "fixed_gray_v1" and m["assumes_no_tvg_law"] is True
    assert m["window_db"]["hi"] > m["window_db"]["lo"]
    assert len(m["range_reference"]["db"]) == len(m["range_reference"]["log10_slant_m"])
    # fitted on real data only (CM-11): no simulated session among the sources
    assert all("sessions/" not in d["folder"] or "merged_sessions" in d["folder"]
               for d in m["training_data"])


def test_fixed_gray_is_the_documented_formula_and_never_modifies_its_input():
    img = _img(seed=3)
    db0 = img.db.copy()
    fg = FixedGray()
    gray = fg.run(img).gray
    np.testing.assert_array_equal(img.db, db0)
    assert gray.dtype == np.uint8 and gray.shape == img.db.shape
    port, s = geo.column_slant(img.shape[1], img.bin_pitch_m)
    lo, hi = fg.params["window_db"]
    for i, j in ((0, 0), (5, 11), (31, 39)):
        g = img.rows.gain_port[i] if port[j] else img.rows.gain_stbd[i]
        y = img.db[i, j] - g - fg.reference(np.array([s[j]]))[0]
        want = 0 if not np.isfinite(y) else int(np.rint(255 * min(max((y - lo) / (hi - lo), 0), 1)))
        assert gray[i, j] == want
    assert np.all(gray[~np.isfinite(img.db)] == 0)                 # NaN -> 0


def test_fixed_gray_is_fixed_not_stretched_per_image():
    """A pixel's grey level depends on its own dB, row gain and column only:
    changing the REST of the image (another scene, a brighter target) leaves
    it alone -- unlike a per-image min/max stretch or equalisation."""
    a, b = _img(seed=1), _img(seed=1)
    b.db[10:20, :] += 25.0                                         # a bright band elsewhere
    ga, gb = FixedGray().run(a).gray, FixedGray().run(b).gray
    np.testing.assert_array_equal(ga[:10], gb[:10])
    np.testing.assert_array_equal(ga[20:], gb[20:])


def test_fixed_gray_removes_the_receive_gain_step():
    a = _img(seed=2)
    meta = make_metadata()
    for r in meta["rows_data"]:
        r["gain_port_db"] = r["gain_stbd_db"] = 5.65
    db = a.db + np.float32(5.65)                                   # the device turned its gain up
    b = SeabedImageInput.from_metadata(meta, db)
    np.testing.assert_array_equal(FixedGray().run(a).gray, FixedGray().run(b).gray)


def test_wire_and_files_give_the_same_detector_input(tmp_path):
    meta, db = make_metadata(image_id=0), make_db(seed=7)
    write_session(tmp_path / "s", [meta], [db])
    live = SeabedImageInput.from_wire(envelope(json.loads(json.dumps(meta))), db.copy())
    off = next(iter_session(tmp_path / "s"))
    for pre in (FixedGray(), FixedGray(near_range_mask_m=0.75)):
        np.testing.assert_array_equal(pre.run(live).gray, pre.run(off).gray)


@pytest.mark.parametrize("pre", [Identity, FixedGray])
def test_the_mask_switches_independently_with_its_boundary_kept(pre):
    img = _img(pitch=0.05)                                          # slant (k+0.5)*0.05
    off, on = pre(near_range_mask_m=0.0).run(img), pre(near_range_mask_m=0.75).run(img)
    _port, s = geo.column_slant(img.shape[1], img.bin_pitch_m)
    masked = s < 0.75
    assert masked.any() and not masked.all()
    assert np.all(on.gray[:, masked] == 0) and not on.valid[:, masked].any()
    np.testing.assert_array_equal(on.gray[:, ~masked], off.gray[:, ~masked])
    # boundary: a column at exactly the mask distance is kept
    k_edge = 7                                                       # s = 0.375 m
    d = (k_edge + 0.5) * 0.05
    edge = pre(near_range_mask_m=d).run(img)
    cols = np.flatnonzero(np.isclose(s, d))
    assert cols.size == 2 and np.array_equal(edge.gray[:, cols], off.gray[:, cols])
    assert near_range_mask(img, 0.0) is None


def test_disabled_and_deferred_options_raise():
    with pytest.raises(NotImplementedError):
        FixedGray(roll_correction=True)
    for name in DEFERRED:
        with pytest.raises(NotImplementedError):
            make_preprocessor(name)
    with pytest.raises(ValueError):
        make_preprocessor("no_such")
    with pytest.raises(ValueError):
        FixedGray(near_range_mask_m=-1)


def test_records_say_which_input_the_detector_saw():
    rec, _ = Pipeline(PipelineConfig(near_range_mask_m=0.75)).run(_img())
    pre = rec["processing"]["preprocess"]
    assert pre["name"] == "fixed_gray" and pre["params"]["near_range_mask_m"] == 0.75
    assert pre["params"]["mapping"] == "fixed_gray_v1" and len(pre["params"]["mapping_sha256"]) == 64
    assert rec["summary"]["detector_input"]["gray"] is True
    base, _ = Pipeline(PipelineConfig(preprocess="identity")).run(_img())
    assert base["processing"]["preprocess"] == {"name": "identity", "params": {"near_range_mask_m": 0.0}}
    # the dummy's detections do not depend on the mapping
    a = Pipeline(PipelineConfig(preprocess="identity")).run(_img())[1]
    b = Pipeline(PipelineConfig(preprocess="fixed_gray")).run(_img())[1]
    assert a == b
