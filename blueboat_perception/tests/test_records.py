"""One record per image; schema completeness; determinism; canonical JSON."""

from __future__ import annotations

import json

import numpy as np
import pytest

from blueboat_perception.core.image import SeabedImageInput
from blueboat_perception.core.pipeline import Pipeline, station_uid
from blueboat_perception.core.records import (RECORD_SCHEMA, canonical_json,
                                              comparable_json)
from conftest import make_db, make_metadata

TOP_KEYS = {"schema", "key", "stream_id", "image_id", "input", "processing", "time",
            "track", "sides", "footprint", "summary", "detections", "provenance"}
DET_KEYS = {"detection_id", "uid", "image_id", "stream_id", "class_name", "class_id",
            "confidence", "confidence_raw", "calibrated", "dummy", "pixel", "bbox_px",
            "world_xy", "covariance_xy", "extent_m", "t_row_s", "side", "slant_range_m",
            "ground_range_m", "look_azimuth_rad", "grazing_angle_rad", "boat_heading_rad",
            "boat_speed_mps", "boat_altitude_m", "boat_roll_rad", "boat_pitch_rad",
            "turn_radius_m", "turn_backscan"}
TRACK_KEYS = {"t_s", "x_m", "y_m", "heading_rad", "speed_mps", "altitude_m", "bottom_m",
              "georef_altitude_m", "roll_rad", "pitch_rad", "yaw_rate_rad_s",
              "yaw_rate_source", "look_port_rad", "look_stbd_rad", "port_ping_number",
              "stbd_ping_number", "yaw_rate_smoothed_rad_s", "turn_radius_m",
              "turn_inner_side", "backscan_from_m"}


class NoDetections:
    name, version, is_dummy = "none", "1", False

    def detect(self, inp):
        return []


def _img(**kw):
    meta = make_metadata(**kw)
    return SeabedImageInput.from_metadata(meta, make_db(meta["rows"], meta["cols"]))


def test_record_schema_is_complete():
    rec, dets = Pipeline().run(_img())
    assert set(rec) == TOP_KEYS and rec["schema"] == RECORD_SCHEMA
    assert set(rec["track"]) == TRACK_KEYS
    assert len(rec["track"]["t_s"]) == rec["time"]["n_rows"] == 32
    assert rec["track"]["yaw_rate_source"] == "imu"
    assert rec["processing"]["detector"] == {"name": "dummy_center", "version": "1",
                                             "dummy": True, "status": "ran"}
    assert rec["processing"]["calibration"]["calibrated"] is False
    assert rec["processing"]["uncertainty"]["assumed"] is True
    assert rec["input"]["georef_convention"] == 2
    assert len(dets) == 1 and set(dets[0]) == DET_KEYS
    d = dets[0]
    assert d["uid"] == station_uid(0, 0) == 2_000_000
    assert d["detection_id"] == f"{rec['key']}/0"
    assert d["dummy"] is True and d["confidence"] == d["confidence_raw"] == 0.5
    cov = np.array(d["covariance_xy"])
    assert np.allclose(cov, cov.T) and np.all(np.linalg.eigvalsh(cov) > 0)
    for side in ("port", "starboard"):
        s = rec["sides"][side]
        assert s["present_rows"] == 32 and s["polygon_xy"][0] == s["polygon_xy"][-1]
    assert rec["footprint"]["outline_xy"][0] == rec["footprint"]["outline_xy"][-1]


def test_an_image_without_detections_still_gets_a_record():
    rec, dets = Pipeline(detector=NoDetections()).run(_img())
    assert dets == [] and rec["detections"] == []          # searched, nothing found
    assert rec["processing"]["detector"]["status"] == "ran"
    assert rec["sides"] is not None


def test_an_image_without_pixels_is_unknown_not_negative():
    meta = make_metadata()
    rec, dets = Pipeline().run(SeabedImageInput.from_metadata(meta, None))
    assert dets is None and rec["detections"] is None
    assert rec["processing"]["detector"]["status"] == "no_pixels"
    assert rec["sides"] is None and rec["footprint"] is None
    assert len(rec["track"]["t_s"]) == 32               # the track is still known


def test_schema4_without_attitude_derives_the_yaw_rate_and_says_so():
    meta = make_metadata(schema=4, convention=None, attitude=False, stream_id=None)
    img = SeabedImageInput.from_metadata(meta, make_db(), stream_id="session:x")
    rec, dets = Pipeline().run(img)
    assert rec["track"]["yaw_rate_source"] == "derived"
    assert rec["track"]["roll_rad"] is None and rec["track"]["port_ping_number"] is None
    assert dets[0]["boat_roll_rad"] is None
    assert rec["track"]["yaw_rate_rad_s"][5] == pytest.approx(0.05, abs=1e-9)


def test_one_sided_rows_are_counted():
    meta = make_metadata()
    db = make_db(nan_side="starboard", nan_rows=range(5, 9))
    rec, _ = Pipeline().run(SeabedImageInput.from_metadata(meta, db))
    assert rec["summary"]["one_sided_rows"] == 4
    assert rec["sides"]["starboard"]["present_rows"] == 28


def test_records_are_deterministic_and_canonical():
    a = canonical_json(Pipeline().run(_img())[0])
    b = canonical_json(Pipeline().run(_img())[0])
    assert a == b
    assert "NaN" not in a and "Infinity" not in a
    assert json.loads(a)["schema"] == RECORD_SCHEMA
    # null roll in one row: NaN never leaks, it becomes null
    meta = make_metadata()
    meta["rows_data"][3]["roll_rad"] = None
    rec = Pipeline().run(SeabedImageInput.from_metadata(meta, make_db()))[0]
    assert json.loads(canonical_json(rec))["track"]["roll_rad"][3] is None


def test_comparable_drops_only_run_dependent_keys():
    rec, _ = Pipeline().run(_img())
    a = dict(rec, provenance={"host": "a"}, source="live")
    b = dict(rec, provenance={"host": "b"}, source="offline")
    assert comparable_json(a) == comparable_json(b)
    assert canonical_json(a) != canonical_json(b)


def test_uid_ranges_do_not_collide():
    assert station_uid(3, 15) == 2_000_000 + 48 + 15
    assert station_uid(3, 16) != station_uid(4, 0)
    assert station_uid(0, 0) != 1_000_000                 # the station's old range
