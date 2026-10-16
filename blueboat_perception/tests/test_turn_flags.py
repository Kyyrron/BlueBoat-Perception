"""Decision 6.2: turn flags in the records -- never a row removed."""

from __future__ import annotations

import math

import numpy as np
import pytest

from blueboat_perception.core import geometry as geo
from blueboat_perception.core.image import SeabedImageInput
from blueboat_perception.core.pipeline import Pipeline
from conftest import make_db, make_metadata


def test_turn_geometry_is_cobras_backscan_condition():
    v = np.array([0.6, 0.6, 0.6, 0.6, 0.0])
    w = np.array([0.0, 0.06, -0.06, 0.012, 0.3])                   # rad/s
    rng = np.full(5, 20.0)
    rstar, side, back = geo.turn_geometry(v, w, rng)
    assert math.isnan(rstar[0]) and side[0] == 0                   # straight
    assert rstar[1] == pytest.approx(10.0) and side[1] == 1        # left turn: port inner
    assert side[2] == -1                                           # right turn: stbd inner
    assert back[1] == pytest.approx(10.0)                          # inside the 20 m swath
    assert math.isnan(back[3])                                     # r* = 50 m > range
    assert math.isnan(rstar[4])                                    # not moving: undefined


def test_smoothing_in_time():
    t = np.array([3.0, 2.0, 1.0, 0.0])                             # newest first
    w = np.array([4.0, 2.0, np.nan, 0.0])
    s = geo.smooth_in_time(t, w, 2.0)                              # +/- 1 s
    assert s[0] == pytest.approx(3.0) and s[1] == pytest.approx(3.0)
    assert s[2] == pytest.approx(1.0) and s[3] == pytest.approx(0.0)   # NaN ignored
    np.testing.assert_array_equal(geo.smooth_in_time(t, w, 1.0)[[0, 1, 3]], w[[0, 1, 3]])


@pytest.mark.parametrize("pitch, inside", [(0.6, True), (0.05, False)])
def test_a_turn_flags_rows_and_keeps_all_of_them(pitch, inside):
    """6.9 deg/s left turn at 0.6 m/s: r* = 5 m. With a 12 m swath (pitch 0.6)
    the inner side is backscanned beyond 5 m; with a 1 m swath it is not."""
    meta = make_metadata(H=32, W=40, pitch=pitch, yaw_rate=0.12)
    rec, dets = Pipeline().run(SeabedImageInput.from_metadata(meta, make_db()))
    tr = rec["track"]
    assert len(tr["turn_radius_m"]) == len(tr["t_s"]) == 32        # every row kept
    assert tr["turn_radius_m"][5] == pytest.approx(0.6 / 0.12)
    assert tr["turn_inner_side"][5] == "port"
    if inside:
        assert tr["backscan_from_m"][5] == pytest.approx(5.0)
        assert rec["summary"]["turn"]["rows_rstar_lt_range"] == 32
        assert rec["summary"]["turn"]["share_rstar_lt_range"] == 1.0
    else:
        assert tr["backscan_from_m"][5] is None
        assert rec["summary"]["turn"]["rows_rstar_lt_range"] == 0
    assert len(dets) == 1                                          # nothing rejected


def test_straight_rows_carry_no_flag():
    meta = make_metadata(H=32, yaw_rate=0.0)
    for r in meta["rows_data"]:
        r["yaw_rate_rad_s"] = 0.0
    rec, dets = Pipeline().run(SeabedImageInput.from_metadata(meta, make_db()))
    assert all(v is None for v in rec["track"]["turn_radius_m"])
    assert rec["summary"]["turn"]["rows_rstar_lt_range"] == 0
    assert dets[0]["turn_backscan"] is False


class FarPort:
    """A test detector: one point at the far port edge of the middle row."""
    name, version, is_dummy = "far_port", "1", False

    def detect(self, inp):
        from blueboat_perception.core.detectors.base import RawDetection
        r = inp.db.shape[0] // 2
        return [RawDetection((r, 0, r, 0), (r, 0), 0, "t", 0.9)]


@pytest.mark.parametrize("yaw_rate, flagged", [(0.12, True), (-0.12, False)])
def test_a_detection_beyond_rstar_on_the_inner_side_is_flagged_not_dropped(yaw_rate, flagged):
    meta = make_metadata(H=32, W=40, pitch=0.6, yaw_rate=yaw_rate)
    rec, dets = Pipeline(detector=FarPort()).run(SeabedImageInput.from_metadata(meta, make_db()))
    (d,) = dets                                                    # kept either way
    assert d["side"] == "port" and d["ground_range_m"] > 5.0
    assert d["turn_radius_m"] == pytest.approx(5.0)
    assert d["turn_backscan"] is flagged
