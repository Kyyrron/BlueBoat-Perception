"""Dataset mode: layout, exclusive names, never inside a session, sidecars."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

from blueboat_perception.core.dataroot import claim_dir, sss_data_dir
from blueboat_perception.core.dataset import (MANIFEST_SCHEMA, DatasetError,
                                              DatasetWriter, acquisition_meta)
from blueboat_perception.core.image import SeabedImageInput
from blueboat_perception.core.session import iter_session
from conftest import make_db, make_metadata, write_session

AUG = Path(__file__).resolve().parents[3] / "SSS-Dataset-Aug-Studio"
HAVE_PYDANTIC = importlib.util.find_spec("pydantic") is not None


def _img(i=0, png=True):
    meta = make_metadata(image_id=i)
    db = make_db(seed=i)
    p = np.nan_to_num(np.clip(db, 0, 60) * 4).astype(np.uint8) if png else None
    return SeabedImageInput.from_metadata(meta, db, p)


def test_layout_and_manifest(tmp_path):
    w = DatasetWriter.create(tmp_path, "2026_10_16-10_00_00-my note", world_dir="/w/x")
    assert w.dir == tmp_path / "data" / "Dataset_data" / "2026_10_16-10_00_00-my_note"
    stem = w.write(_img(0))
    w.write(_img(1))
    w.close()
    assert stem == "gcs-20261016T000000Z-abcd_00000"
    for rel in (f"images/{stem}.png", f"images/{stem}.json", f"labels/{stem}.txt",
                f"raw/{stem}_db.npz", "manifest.json", "data.yaml"):
        assert (w.dir / rel).is_file(), rel
    assert (w.dir / f"labels/{stem}.txt").read_text() == ""        # NullLabelSource
    m = json.loads((w.dir / "manifest.json").read_text())
    assert m["schema"] == MANIFEST_SCHEMA and m["image_count"] == 2
    assert m["world_dir"] == "/w/x" and m["label_source"] == "none"
    assert "R-3" in m["intensity_mapping"]
    with np.load(w.dir / f"raw/{stem}_db.npz") as z:
        np.testing.assert_array_equal(z["db"], _img(0).db)
    side = json.loads((w.dir / f"images/{stem}.json").read_text())
    assert side["layout"] == "dual" and "intensity_mapping" not in side
    assert "res_across_m" not in side
    assert side["gcs_metadata"]["image_id"] == 0
    assert "names: {}" in (w.dir / "data.yaml").read_text()


def test_names_are_claimed_exclusively(tmp_path):
    a = DatasetWriter.create(tmp_path, "n")
    b = DatasetWriter.create(tmp_path, "n")
    assert a.dir.name == "n" and b.dir.name == "n-2"
    a.write(_img(0))
    with pytest.raises(FileExistsError):         # never overwrites a file
        a.write(_img(0))


def test_never_inside_a_recording_session(tmp_path):
    inside = claim_dir(sss_data_dir(tmp_path) / "sessions" / "x", "ds")
    with pytest.raises(DatasetError):
        DatasetWriter(inside, name="ds", forbidden_roots=[sss_data_dir(tmp_path)])
    with pytest.raises(DatasetError):
        DatasetWriter.create(tmp_path, "")


def test_images_without_png_or_pixels_are_counted_not_invented(tmp_path):
    w = DatasetWriter.create(tmp_path, "n")
    assert w.write(_img(0, png=False)) is None
    assert w.skipped_no_png == 1 and w.count == 0
    meta = make_metadata()
    assert w.write(SeabedImageInput.from_metadata(meta, None)) is None


def test_offline_copies_the_session_png_verbatim(tmp_path, synthetic_session):
    w = DatasetWriter.create(tmp_path / "root", "off")
    img = next(iter_session(synthetic_session))
    stem = w.write(img)
    assert (w.dir / f"images/{stem}.png").read_bytes() == Path(img.png_path).read_bytes()


def test_acquisition_meta_stays_inside_the_aug_studio_bounds():
    meta = make_metadata(altitude=0.05)                     # below its 0.1 m floor
    m = acquisition_meta(SeabedImageInput.from_metadata(meta, make_db()))
    assert "altitude_m" not in m
    m = acquisition_meta(SeabedImageInput.from_metadata(make_metadata(), make_db()))
    assert m["altitude_m"] >= 0.1 and m["slant_range_m"] > 0
    assert 0.0 <= m["heading_deg"] < 360.0
    assert "timestamp" not in m                             # sim time, not a date


@pytest.mark.skipif(not (HAVE_PYDANTIC and (AUG / "sss_aug_studio").is_dir()),
                    reason="needs pydantic and the SSS-Dataset-Aug-Studio sibling")
def test_the_sidecar_validates_as_an_aug_studio_acquisition_meta(tmp_path):
    sys.path.insert(0, str(AUG))
    try:
        from sss_aug_studio.core.meta import AcquisitionMeta
    finally:
        sys.path.remove(str(AUG))
    w = DatasetWriter.create(tmp_path, "n")
    stem = w.write(_img(0))
    side = json.loads((w.dir / f"images/{stem}.json").read_text())
    am = AcquisitionMeta(**side)                             # extra="ignore"
    assert am.layout == "dual" and am.altitude_m == pytest.approx(side["altitude_m"])
    # the fixed-mapping sidecar declares log + its range, within the Aug Studio's bounds
    from blueboat_perception.core.pipeline import Pipeline
    pipe = Pipeline()
    w2 = DatasetWriter.create(tmp_path, "m", detector_input=pipe.detector_input_info())
    img = _img(1)
    rec, _d, inp = pipe.run_full(img)
    stem2 = w2.write(img, rec, inp)
    am2 = AcquisitionMeta(**json.loads((w2.dir / f"images/{stem2}.json").read_text()))
    assert am2.intensity_mapping == "log"
    lo, hi = pipe.preprocessor.params["window_db"]
    assert am2.log_range_db == pytest.approx(hi - lo)


def test_training_images_are_the_detector_input_and_the_display_is_kept(tmp_path):
    """Decision 6.4: training and inference see one mapping."""
    import cv2
    from blueboat_perception.core.pipeline import Pipeline
    pipe = Pipeline()                                            # fixed_gray by default
    w = DatasetWriter.create(tmp_path, "n", detector_input=pipe.detector_input_info())
    img = _img(0)
    _rec, _dets, inp = pipe.run_full(img)
    stem = w.write(img, _rec, inp)
    np.testing.assert_array_equal(cv2.imread(str(w.dir / f"images/{stem}.png"), 0), inp.gray)
    np.testing.assert_array_equal(cv2.imread(str(w.dir / f"display/{stem}.png"), 0), img.png)
    side = json.loads((w.dir / f"images/{stem}.json").read_text())
    lo, hi = pipe.preprocessor.params["window_db"]
    assert side["intensity_mapping"] == "log"
    assert side["log_range_db"] == pytest.approx(hi - lo)
    assert side["perception"]["training_image"] == "detector input"
    m = json.loads((w.dir / "manifest.json").read_text())
    assert m["training_images"] == "detector input" and m["detector_input"]["name"] == "fixed_gray"
    assert "log_range_db" in m["intensity_mapping"]


def test_the_identity_baseline_keeps_the_display_png_as_training_image(tmp_path):
    from blueboat_perception.core.pipeline import Pipeline, PipelineConfig
    pipe = Pipeline(PipelineConfig(preprocess="identity"))
    w = DatasetWriter.create(tmp_path, "n", detector_input=pipe.detector_input_info())
    img = _img(0)
    _rec, _dets, inp = pipe.run_full(img)
    stem = w.write(img, _rec, inp)
    assert (w.dir / f"images/{stem}.png").read_bytes() == (w.dir / f"display/{stem}.png").read_bytes()
    side = json.loads((w.dir / f"images/{stem}.json").read_text())
    assert "intensity_mapping" not in side                     # power transfer: R-3 gap


def test_a_ground_truth_label_source_fills_labels_data_yaml_and_sidecars(tmp_path):
    import test_sim_labels as T
    from blueboat_perception.core import sim_labels as SL
    o = T.obj()
    buf = SL.GroundTruthBuffer()
    T.feed(buf, [o])
    src = T.source([o], buf)
    w = DatasetWriter.create(tmp_path, "labelled", world_dir="/w/x", label_source=src)
    empty = T.picture(first_ping=5000, image_id=1, png=True)   # no ground truth for it
    stem = w.write(T.picture(png=True))
    stem_empty = w.write(empty)
    w.close()
    lines = (w.dir / "labels" / f"{stem}.txt").read_text().splitlines()
    assert len(lines) == 1
    cid, *box = lines[0].split()
    assert int(cid) == SL.load_class_map().index("concrete_block")
    assert all(0.0 < float(v) <= 1.0 for v in box)
    assert (w.dir / "labels" / f"{stem_empty}.txt").read_text() == ""
    data = (w.dir / "data.yaml").read_text()
    assert "train: images" in data and "val: images" in data and "nc: 29" in data
    assert "  0: tire_car" in data and "  28: shopping_trolley" in data
    side = json.loads((w.dir / "images" / f"{stem}.json").read_text())
    (b,) = side["perception"]["labels"]["boxes"]
    assert b["type"] == "concrete_block" and b["object_id"] == 7
    assert side["perception"]["labels"]["ground_truth_coverage"] == 1.0
    m = json.loads((w.dir / "manifest.json").read_text())
    assert m["label_source"] == "sim_ground_truth/1"
    assert m["labels"]["boxes"] == 1 and m["labels"]["images"] == 2
    assert m["labels"]["images_incomplete_coverage"] == 1
    assert m["label_source_info"]["objects"] == 1
