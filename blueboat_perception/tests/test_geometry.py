"""Pixel -> world, all three conventions, against the closed form."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from blueboat_perception.core import geometry as geo
from blueboat_perception.core.image import Georef, InputError, SeabedImageInput
from conftest import make_db, make_metadata

ARM = -0.345                 # the GCS's arm of 2026-11-15 (adopted, not measured)


def _img(**kw):
    meta = make_metadata(**kw)
    return SeabedImageInput.from_metadata(meta, make_db(meta["rows"], meta["cols"]))


@pytest.mark.parametrize("conv", [1, 2, 3])
def test_grids_follow_the_documented_formula(conv):
    img = _img(convention=conv, H=8, W=20, pitch=0.1)
    wx, wy = geo.world_grids(img)
    lever = 0.285 if conv >= 2 else 0.0
    a = ARM if conv == 3 else 0.0
    for i in (0, 3, 7):
        h = img.rows.bottom[i] if conv >= 2 else img.rows.altitude[i]
        for j in (0, 5, 9, 10, 14, 19):
            k = (10 - 1 - j) if j < 10 else (j - 10)
            s = (k + 0.5) * 0.1
            g = math.sqrt(max(s * s - h * h, 0.0))
            y = (lever + g) if j < 10 else -(lever + g)
            yaw = img.rows.yaw[i]
            ex = img.rows.x[i] + math.cos(yaw) * a - math.sin(yaw) * y
            ey = img.rows.y[i] + math.sin(yaw) * a + math.cos(yaw) * y
            assert wx[i, j] == pytest.approx(ex, abs=1e-5)
            assert wy[i, j] == pytest.approx(ey, abs=1e-5)
    assert wx.dtype == np.float32 and wy.dtype == np.float32


def _pass(conv, yaw, x_row, y_row, *, arm=ARM, W=200):
    """A 4-row picture whose rows all sit at one navigation pose."""
    meta = make_metadata(H=4, W=W, pitch=0.1, convention=conv, lever_x=arm, yaw0=yaw,
                         yaw_rate=0.0, attitude=False)
    for r in meta["rows_data"]:
        r.update(x_m=x_row, y_m=y_row, yaw_rad=yaw, bottom_m=2.0)
    return SeabedImageInput.from_metadata(meta, make_db(4, W))


def test_convention_3_places_one_point_seen_on_opposite_passes_at_one_place():
    # P is imaged on port, column j, when the TRANSDUCERS are abeam of it. They
    # sit `ARM` along the heading from the navigation pose, so the boat reports
    # x = Px − ARM heading east and x = Px + ARM heading west.
    px, j = 10.0, 40
    probe = _pass(3, 0.0, 0.0, 0.0)
    y_local = float(geo.world_grids(probe)[1][0, j])           # lateral of column j
    east = _pass(3, 0.0, px - ARM, 0.0)
    west = _pass(3, math.pi, px + ARM, 2.0 * y_local)
    (ex, ey), (wx, wy) = geo.world_grids(east), geo.world_grids(west)
    assert ex[0, j] == pytest.approx(px, abs=1e-5) and wx[0, j] == pytest.approx(px, abs=1e-5)
    assert ey[0, j] == pytest.approx(y_local, abs=1e-5)
    assert wy[0, j] == pytest.approx(y_local, abs=1e-5)
    # ignoring the arm (convention 2, same navigation poses): 2·|ARM| apart along track
    e2, w2 = geo.world_grids(_pass(2, 0.0, px - ARM, 0.0)), \
        geo.world_grids(_pass(2, math.pi, px + ARM, 2.0 * y_local))
    assert float(e2[0][0, j]) - float(w2[0][0, j]) == pytest.approx(2.0 * abs(ARM), abs=1e-5)


def test_convention_3_edge_points_are_the_grids_at_that_slant():
    img = _img(convention=3, H=16, W=40, pitch=0.05)
    wx, wy = geo.world_grids(img)
    port, s = geo.column_slant(40, 0.05)
    for j in (3, 17, 25, 36):
        pts = geo.edge_points(img, list(range(16)), np.full(16, s[j]), bool(port[j]))
        assert len(pts) == 16
        for i, (x, y) in enumerate(pts):
            assert np.float32(x) == pytest.approx(wx[i, j], abs=1e-5)
            assert np.float32(y) == pytest.approx(wy[i, j], abs=1e-5)


def test_the_arm_is_used_under_convention_3_only():
    conv2 = geo.world_grids(_img(convention=2))
    zero3 = geo.world_grids(_img(convention=3, lever_x=0.0))
    for a, b in zip(conv2, zero3):
        np.testing.assert_array_equal(a, b)
    # a convention-2 picture is never shifted, whatever its x says
    meta = make_metadata(convention=2)
    meta["georef"]["lever_arm_m"]["x"] = ARM
    img = SeabedImageInput.from_metadata(meta, make_db())
    assert geo.fore_aft_arm(img) == 0.0
    for a, b in zip(conv2, geo.world_grids(img)):
        np.testing.assert_array_equal(a, b)
    assert geo.fore_aft_arm(_img(convention=3)) == ARM


@pytest.mark.parametrize("conv", [1, 2])
def test_no_arm_leaves_a_negative_zero_row_pose_untouched(conv):
    # x + cos(yaw)·0.0 would turn −0.0 into +0.0 (and "-0.0" into "0.0" in a record)
    # (port columns: there −0.0 − sin(0)·y keeps the sign; on starboard IEEE
    # gives −0.0 − (−0.0) = +0.0 with or without an arm)
    img = _pass(conv, 0.0, -0.0, -0.0, W=20)
    wx, _ = geo.world_grids(img)
    assert np.signbit(wx[:, :10]).all()
    port, s = geo.column_slant(20, 0.1)
    pts = geo.edge_points(img, [0, 1], np.full(4, s[2]), True)
    assert all(math.copysign(1.0, p[0]) == -1.0 for p in pts)


@pytest.mark.parametrize("conv", [0, 4, 7])
def test_an_unknown_convention_is_rejected(conv):
    meta = make_metadata(convention=2)
    meta["georef"]["convention"] = conv
    with pytest.raises(InputError, match=f"georef convention {conv} is not supported"):
        Georef.from_metadata(meta)
    with pytest.raises(InputError):
        SeabedImageInput.from_metadata(meta, make_db())


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


def test_convention_3_inside_the_water_column_maps_to_the_transducer_point():
    img = _img(convention=3, H=4, W=20, pitch=0.01, yaw0=0.0, yaw_rate=0.0)
    wx, wy = geo.world_grids(img)
    # heading east: the transducers sit ARM along x and ±0.285 m across
    for j, lat in ((9, 0.285), (10, -0.285)):
        assert wx[0, j] - img.rows.x[0] == pytest.approx(ARM, abs=1e-6)
        assert wy[0, j] - img.rows.y[0] == pytest.approx(lat, abs=1e-6)


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


# --- CR-PER-2 (session 12): Perception places a pixel where Belief places that ping's sample ---

_BELIEF_GEOMETRY = (Path(__file__).resolve().parents[2].parent / "BlueBoat-Belief"
                    / "blueboat_belief" / "blueboat_belief" / "core" / "geometry.py")


def _belief_mirror():
    """Belief's two placement steps (``core/geometry.py`` ``sensor_xy`` then
    ``sample_xy``, ``core/belief.py`` ``_process_ping``), written out."""
    def sensor_xy(x, y, yaw, arm):
        return x + arm * math.cos(yaw), y + arm * math.sin(yaw)

    def sample_xy(x, y, yaw, yoff):
        s, c = math.sin(yaw), math.cos(yaw)
        return x - yoff * s, y + yoff * c
    return sensor_xy, sample_xy


def _belief_real():
    """Belief's own functions when the sibling checkout is beside this module
    (loaded by file path: no package import, no dependency); else None."""
    if not _BELIEF_GEOMETRY.is_file():
        return None
    import importlib.util
    spec = importlib.util.spec_from_file_location("_belief_geometry_cr_per_2", _BELIEF_GEOMETRY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.sensor_xy, mod.sample_xy


@pytest.mark.parametrize("source", ["mirror", "belief"])
@pytest.mark.parametrize("yaw_deg", [0, 90, 180, 270])
@pytest.mark.parametrize("j", [12, 170])                 # one port bin, one starboard bin
def test_convention_3_matches_belief_at_four_headings(source, yaw_deg, j):
    fns = _belief_mirror() if source == "mirror" else _belief_real()
    if fns is None:
        pytest.skip(f"BlueBoat-Belief not beside this module ({_BELIEF_GEOMETRY})")
    sensor_xy, sample_xy = fns
    yaw, x0, y0, bottom, W, pitch = math.radians(yaw_deg), 3.0, -2.0, 2.0, 200, 0.1
    img3, img2 = _pass(3, yaw, x0, y0, W=W), _pass(2, yaw, x0, y0, W=W)
    wx3, wy3 = geo.world_grids(img3)
    wx2, wy2 = geo.world_grids(img2)
    # the same bin as the processor would publish it: signed ground offset, + port
    port, s = geo.column_slant(W, pitch)
    ground = math.sqrt(max(s[j] ** 2 - bottom ** 2, 0.0))
    yoff = (0.285 + ground) if port[j] else -(0.285 + ground)
    assert bool(port[j]) == (j < W // 2)
    bx, by = sample_xy(*sensor_xy(x0, y0, yaw, ARM), yaw, np.float64(yoff))
    for r in range(4):
        assert abs(float(wx3[r, j]) - float(bx)) < 1e-3
        assert abs(float(wy3[r, j]) - float(by)) < 1e-3
    # the arm moves the point 0.345 m AFT along the heading, so the shift flips with it
    dx, dy = float(wx3[0, j]) - float(wx2[0, j]), float(wy3[0, j]) - float(wy2[0, j])
    assert dx == pytest.approx(ARM * math.cos(yaw), abs=1e-5)
    assert dy == pytest.approx(ARM * math.sin(yaw), abs=1e-5)
    assert math.hypot(dx, dy) == pytest.approx(0.345, abs=1e-5)
    assert dx * math.cos(yaw) + dy * math.sin(yaw) == pytest.approx(-0.345, abs=1e-5)
