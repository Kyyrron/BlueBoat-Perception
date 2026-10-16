"""Pixel -> world, both conventions, against the closed form."""

from __future__ import annotations

import math

import numpy as np
import pytest

from blueboat_perception.core import geometry as geo
from blueboat_perception.core.image import Georef, SeabedImageInput
from conftest import make_db, make_metadata


def _img(**kw):
    meta = make_metadata(**kw)
    return SeabedImageInput.from_metadata(meta, make_db(meta["rows"], meta["cols"]))


@pytest.mark.parametrize("conv", [1, 2])
def test_grids_follow_the_documented_formula(conv):
    img = _img(convention=conv, H=8, W=20, pitch=0.1)
    wx, wy = geo.world_grids(img)
    lever = 0.285 if conv == 2 else 0.0
    for i in (0, 3, 7):
        h = img.rows.bottom[i] if conv == 2 else img.rows.altitude[i]
        for j in (0, 5, 9, 10, 14, 19):
            k = (10 - 1 - j) if j < 10 else (j - 10)
            s = (k + 0.5) * 0.1
            g = math.sqrt(max(s * s - h * h, 0.0))
            y = (lever + g) if j < 10 else -(lever + g)
            yaw = img.rows.yaw[i]
            assert wx[i, j] == pytest.approx(img.rows.x[i] - math.sin(yaw) * y, abs=1e-5)
            assert wy[i, j] == pytest.approx(img.rows.y[i] + math.cos(yaw) * y, abs=1e-5)
    assert wx.dtype == np.float32 and wy.dtype == np.float32


def test_georef_absent_is_convention_1():
    meta = make_metadata(convention=None, schema=4)
    assert Georef.from_metadata(meta) == Georef()
    img = SeabedImageInput.from_metadata(meta, make_db())
    assert img.georef.convention == 1 and img.georef.altitude_key == "altitude_m"


def test_convention_2_inside_the_water_column_maps_to_the_transducer():
    img = _img(convention=2, H=4, W=20, pitch=0.01, yaw0=0.0, yaw_rate=0.0)
    wx, wy = geo.world_grids(img)
    # first port / starboard bins are inside the water column: |y| = lever
    assert wy[0, 9] - img.rows.y[0] == pytest.approx(0.285, abs=1e-6)
    assert wy[0, 10] - img.rows.y[0] == pytest.approx(-0.285, abs=1e-6)


def test_look_azimuths_and_grazing():
    p, s = geo.look_azimuths(np.array([0.0, math.pi]))
    assert p[0] == pytest.approx(math.pi / 2) and s[0] == pytest.approx(-math.pi / 2)
    assert p[1] == pytest.approx(-math.pi / 2) and s[1] == pytest.approx(math.pi / 2)
    assert geo.grazing_angle(2.0, 1.0) == pytest.approx(math.asin(0.5))
    assert geo.grazing_angle(0.5, 1.0) == pytest.approx(math.pi / 2)
    assert geo.grazing_angle(0.0, 1.0) is None


def test_derived_yaw_rate_recovers_a_constant_turn():
    t = np.linspace(10.0, 8.0, 41)              # newest first
    yaw = 3.0 + 0.2 * (t - 10.0)                # crosses +pi: unwrap needed
    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
    rate = geo.derived_yaw_rate(t, yaw)
    np.testing.assert_allclose(rate, 0.2, atol=1e-9)
    t2 = np.array([1.0, 1.0, 0.9])              # repeated time: no rate there
    assert np.isnan(geo.derived_yaw_rate(t2, np.zeros(3))).any()


def test_side_spans_use_the_conventions_altitude():
    img = _img(convention=2, H=4, W=40, pitch=0.05, altitude=0.6)
    spans = geo.side_spans(img)
    for side in ("port", "starboard"):
        near = spans[side]["near"]
        assert np.all(near >= img.rows.bottom - 1e-12)
        assert np.all(near - img.rows.bottom < 0.05 + 1e-12)
