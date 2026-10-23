"""The YOLO adapter over tiles (core/detectors/yolo.py), with a fake runtime;
the real Ultralytics test skips when it is not installed."""

from __future__ import annotations

import importlib.util
import json

import numpy as np
import pytest

from blueboat_perception.core.detectors import make_detector
from blueboat_perception.core.detectors.yolo import (PARAM_DEFAULTS, YoloDetector,
                                                     detector_args)
from blueboat_perception.core.image import SeabedImageInput
from blueboat_perception.core.pipeline import (MAX_DETECTIONS, Pipeline, PipelineConfig,
                                               on_map, station_uid)
from blueboat_perception.core.records import canonical_json, comparable_json
from blueboat_perception.core.session import iter_session
from conftest import envelope, make_metadata, write_session

HAVE_ULTRALYTICS = (importlib.util.find_spec("ultralytics") is not None
                    and importlib.util.find_spec("torch") is not None)
H, W = 256, 1200
PITCH = 20.0 / (W // 2)
BRIGHT, DARK = 140.0, -60.0          # dB that fixed_gray_v1 maps to 255 / 0


class FakeRuntime:
    """Deterministic stand-in: one box around the pixels > 200 of each input,
    scored by how much of the object it sees (a whole object beats its cut
    pieces, as a trained model's would), plus a faint box at a fixed place
    (tests the map filter)."""

    def __init__(self, extra=0, faint=True):
        self.calls = []
        self.extra = extra
        self.faint = faint

    def info(self):
        return {"runtime": "fake", "channels": 3,
                "names": {i: f"c{i}" for i in range(29)}}

    def predict(self, images, imgsz, conf, iou, max_det):
        self.calls.append((len(images), imgsz, conf, iou, max_det))
        out = []
        for img in images:
            rows = []
            ys, xs = np.nonzero(img > 200)
            if xs.size:
                rows.append([xs.min(), ys.min(), xs.max() + 1, ys.max() + 1,
                             0.5 + xs.size / 1e6, 3])
            if self.faint:
                rows.append([0.0, 0.0, 8.0, 8.0, 0.05, 1])
            for k in range(self.extra):
                rows.append([k % 200, 0.0, k % 200 + 4.0, 4.0, 0.2 + k * 1e-5, 2])
            out.append(np.asarray(rows, np.float64).reshape(-1, 6))
        return out


def picture(image_id=0, rows=H, obj=(300, 470), obj_rows=(40, 90), seed=0):
    meta = make_metadata(H=rows, W=W, pitch=PITCH, image_id=image_id, t0=100.0 + image_id)
    db = np.full((rows, W), DARK, np.float32)
    db[:, W // 2 - 3: W // 2 + 3] = np.nan                    # nadir blank
    if obj is not None:
        db[obj_rows[0]:obj_rows[1], obj[0]:obj[1]] = BRIGHT
    return meta, db


def pipeline(runtime=None, **kw):
    det = YoloDetector(runtime=runtime or FakeRuntime(), **kw)
    return Pipeline(PipelineConfig(detector="yolo"), detector=det)


def run(meta, db, p=None):
    p = p or pipeline()
    return p.run(SeabedImageInput.from_wire(envelope(meta), db))


# ------------------------------------------------------------------ behaviour
def test_an_object_across_a_tile_edge_is_one_detection_in_picture_pixels():
    meta, db = picture(obj=(300, 470))          # tiles [256,512) [384,640) [128,384)
    rec, dets = run(meta, db, pipeline(merge="greedy_nmm"))
    real = [d for d in dets if d["class_id"] == 3]
    assert len(real) == 1
    assert real[0]["bbox_px"] == [40, 300, 89, 469]
    assert real[0]["pixel"] == [(40 + 89) // 2, (300 + 469) // 2]
    assert real[0]["class_name"] == "c3" and not real[0]["dummy"]
    # plain NMS leaves the cut pieces whose IoU with the whole box is < 0.5
    _, dets_nms = run(meta, db, pipeline(merge="nms"))
    assert len([d for d in dets_nms if d["class_id"] == 3]) > 1


def test_record_names_the_detector_tiling_and_merge_and_counts_tiles():
    meta, db = picture()
    rec, _ = run(meta, db)
    d = rec["processing"]["detector"]
    assert (d["name"], d["version"], d["dummy"], d["status"]) == ("yolo", "1", False, "ran")
    assert d["params"]["tiling"] == {"mode": "tiles", "tile": 256, "stride": 128,
                                     "min_valid_fraction": 0.001}
    assert d["params"]["merge"] == {"rule": "nms", "threshold": 0.5, "metric": "iou",
                                    "score": "max"}
    assert d["params"]["input_size"] == 256 and d["capped"] == 0
    st = rec["summary"]["detector_tiles"]
    assert st["tiles"] == 9 and st["tiles_skipped"] == 0 and st["ran"] is True
    assert st["boxes_before_merge"] >= st["boxes_after_merge"] > 0
    assert rec["processing"]["calibration"]["calibrated"] is False


def test_all_tiles_of_a_picture_go_in_one_batch_at_256():
    rt = FakeRuntime()
    meta, db = picture()
    run(meta, db, pipeline(rt))
    assert rt.calls == [(9, 256, 0.01, 0.7, 300)]


def test_offline_equals_live_byte_for_byte(tmp_path):
    metas, dbs = zip(*[picture(image_id=i, obj=(300 + 40 * i, 470 + 40 * i)) for i in range(3)])
    write_session(tmp_path / "s", metas, dbs, with_png=False)
    p = pipeline(merge="greedy_nmm")
    live = [comparable_json(run(m, d.copy(), p)[0]) for m, d in zip(metas, dbs)]
    off = [comparable_json(p.run(img)[0]) for img in iter_session(tmp_path / "s")]
    assert live == off
    again = [comparable_json(run(m, d.copy(), pipeline(merge="greedy_nmm"))[0])
             for m, d in zip(metas, dbs)]
    assert again == live


def test_detections_are_capped_at_999_and_uids_never_collide():
    meta, db = picture()
    rec, dets = run(meta, db, pipeline(FakeRuntime(extra=200), merge="nms",
                                       merge_threshold=0.99))
    assert len(dets) == MAX_DETECTIONS
    assert rec["processing"]["detector"]["capped"] > 0
    uids = [d["uid"] for d in dets]
    assert len(set(uids)) == len(uids)
    assert all(station_uid(0, k) not in {station_uid(1, j) for j in range(999)}
               for k in range(999))
    # kept in descending score order
    scores = [d["confidence_raw"] for d in dets]
    assert scores == sorted(scores, reverse=True)


def test_a_short_picture_is_padded_and_detections_stay_inside_it():
    meta, db = picture(rows=100, obj=(300, 470), obj_rows=(60, 100))
    rec, dets = run(meta, db)
    assert rec["input"]["rows"] == 100 and rec["detections"] is not None
    for d in dets:
        assert 0 <= d["bbox_px"][0] <= d["bbox_px"][2] <= 99


def test_no_detector_input_means_unknown_not_empty():
    meta, db = picture()
    img = SeabedImageInput.from_wire(envelope(meta), db)   # no PNG
    det = YoloDetector(runtime=FakeRuntime())
    p = Pipeline(PipelineConfig(preprocess="identity", detector="yolo"), detector=det)
    rec, dets = p.run(img)
    assert dets is None and rec["detections"] is None
    assert rec["processing"]["detector"]["status"] == "no_detector_input"
    assert rec["sides"] is not None


def test_map_filter_keeps_uids():
    meta, db = picture()
    rec, dets = run(meta, db)
    shown = on_map(dets, 0.10)
    assert [d["uid"] for d in shown] == [d["uid"] for d in dets if d["confidence"] >= 0.10]
    assert len(shown) < len(dets)                    # the faint 0.05 boxes are not drawn
    assert on_map(None, 0.1) == []


def test_full_mode_letterboxes_the_whole_strip():
    rt = FakeRuntime(faint=False)
    meta, db = picture()
    rec, dets = run(meta, db, pipeline(rt, tiling="full"))
    assert rt.calls == [(1, 640, 0.01, 0.7, 300)]
    assert rec["summary"]["detector_tiles"]["tiles"] == 1
    assert [d["bbox_px"] for d in dets] == [[40, 300, 89, 469]]


def test_dummy_records_carry_no_yolo_fields():
    meta, db = picture()
    rec, dets = Pipeline().run(SeabedImageInput.from_wire(envelope(meta), db))
    assert "params" not in rec["processing"]["detector"]
    assert "capped" not in rec["processing"]["detector"]
    assert "detector_tiles" not in rec["summary"]
    assert dets[0]["class_name"] == "dummy_center"


def test_parameters_map_to_the_adapter():
    assert detector_args("dummy_center", {"detector_model": "x"}) == {}
    a = detector_args("yolo", {"detector_model": "/m.pt", "merge": "greedy_nmm"})
    assert a["model_path"] == "/m.pt" and a["merge"] == "greedy_nmm"
    assert a["tile_stride"] == 128 and a["score_threshold"] == 0.01
    assert set(PARAM_DEFAULTS) >= {"detector_model", "tile_stride", "merge"}


def test_configuration_errors(tmp_path):
    with pytest.raises(ValueError):
        YoloDetector(runtime=FakeRuntime(), tiling="squash")
    with pytest.raises(ValueError):
        YoloDetector(runtime=FakeRuntime(), merge="soft")
    with pytest.raises(ValueError):
        make_detector("yolo")                                  # no weights
    with pytest.raises(FileNotFoundError):
        make_detector("yolo", model_path=str(tmp_path / "none.pt"))


def test_the_model_file_hash_and_card_are_recorded(tmp_path):
    w = tmp_path / "m" / "weights" / "best.pt"
    w.parent.mkdir(parents=True)
    w.write_bytes(b"weights")
    (tmp_path / "m" / "model.json").write_text(json.dumps(
        {"name": "x", "family": "yolo26s", "seed": 0, "secret": "not copied"}))
    det = YoloDetector(model_path=str(w), runtime=FakeRuntime())
    m = det.params["model"]
    assert m["file"] == "best.pt" and len(m["sha256"]) == 64
    assert m["card"]["family"] == "yolo26s" and "secret" not in m["card"]


# ------------------------------------------------------------ real Ultralytics
@pytest.mark.skipif(not HAVE_ULTRALYTICS, reason="needs ultralytics and torch")
@pytest.mark.parametrize("arch", ["yolo11n.yaml", "yolo26n.yaml"])
def test_real_ultralytics_cpu_offline_equals_live(tmp_path, monkeypatch, arch):
    monkeypatch.setenv("YOLO_CONFIG_DIR", str(tmp_path / "cfg"))
    monkeypatch.setenv("YOLO_OFFLINE", "1")
    import torch
    from ultralytics import YOLO
    torch.manual_seed(0)
    weights = tmp_path / "tiny.pt"
    YOLO(arch).save(str(weights))                   # untrained, seeded: no download
    metas, dbs = zip(*[picture(image_id=i) for i in range(2)])
    write_session(tmp_path / "s", metas, dbs, with_png=False)

    def make():
        det = YoloDetector(model_path=str(weights), score_threshold=0.0001,
                           merge="greedy_nmm")
        return Pipeline(PipelineConfig(detector="yolo"), detector=det)

    live = [canonical_json(dict(run(m, d.copy(), make())[0], provenance={}))
            for m, d in zip(metas, dbs)]
    p = make()
    off = [canonical_json(dict(p.run(img)[0], provenance={})) for img in iter_session(tmp_path / "s")]
    assert live == off
    rec = json.loads(live[0])
    params = rec["processing"]["detector"]["params"]
    assert params["runtime"]["runtime"] == "ultralytics" and params["model"]["sha256"]
    assert rec["detections"] is not None
