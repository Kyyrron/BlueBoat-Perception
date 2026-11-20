"""Dataset labels: no box extent comes from a contact (2026-11-18).

``SIM_VALIDITY_CONDITIONS.md`` §5.5 (superproject): labels come from the rendered picture
and the world metadata; a contact is never a label. The acceptance report's C5 check found
the renderer's ``shadow_bins`` 5 to 35 bins past the picture's shadow, so the shadow end is
now the flat-floor tip from the object's metadata and the seabed under it, and
``shadow_bins`` is only reported (``s_mask``). Without terrain, the seabed is taken at the
sensor message's altitude, not at the contact's range.
"""
from __future__ import annotations

import math

import pytest

from blueboat_perception.core import sim_labels as SL

SZ, ZSEA = -0.2, -4.0                     # transducer and seabed under the object
BIN = 20.0 / 600
KW = dict(bin_m=BIN, range_max_m=BIN * 600, n_bins=600)
POSE = [0.3, 0.285, SZ, 0.0, 0.0, 0.0]    # yaw 0: port looks along +y


def obj(y=5.0, h=0.3, w=0.2):
    return SL.WorldObject(object_id=7, type="concrete_block", x=0.3, y=y, yaw=0.0,
                          length=0.4, width=w, height=h)


def flat_tip(o):
    a_max = o.y + o.width / 2 - POSE[1]
    dz = SZ - ZSEA
    return math.hypot(a_max * dz / (dz - o.height), dz)


def test_the_shadow_end_is_the_flat_floor_tip_of_the_world_geometry():
    o = obj()
    g = SL.ping_object_bins(POSE, 1, o, {"shadow_bins": 0.0}, ZSEA, **KW)
    assert g["shadow_from"] == "flat_seabed"
    assert g["s_tip"] == pytest.approx(flat_tip(o), abs=1e-9)
    assert g["k"][2] == math.ceil(flat_tip(o) / BIN) - 1 + SL.DEFAULT_RANGE_MARGIN_BINS


@pytest.mark.parametrize("shadow_bins", [0.0, 3.0, 35.0, 300.0])
def test_the_contacts_shadow_bins_is_reported_but_never_sets_the_end(shadow_bins):
    o = obj()
    g0 = SL.ping_object_bins(POSE, 1, o, {"shadow_bins": 0.0}, ZSEA, **KW)
    g = SL.ping_object_bins(POSE, 1, o, {"shadow_bins": shadow_bins}, ZSEA, **KW)
    assert g["k"] == g0["k"] and g["shadow_m"] == g0["shadow_m"]
    assert g["s_mask"] == pytest.approx(g0["s_mask"] + shadow_bins * BIN)


def test_an_object_up_to_the_transducer_casts_its_shadow_to_the_range_limit():
    o = obj(y=8.0, h=SZ - ZSEA + 0.5)                     # a pile standing out of the water
    g = SL.ping_object_bins(POSE, 1, o, {}, ZSEA, **KW)
    assert g["s_tip"] == pytest.approx(BIN * 600) and g["clipped"] is True


def test_without_terrain_the_sensor_altitude_sets_the_height_not_the_contact():
    o = obj()
    bogus = {"slant_range_m": 99.0, "shadow_bins": 0.0}
    g = SL.ping_object_bins(POSE, 1, o, bogus, None, altitude=SZ - ZSEA, **KW)
    assert g["dz"] == pytest.approx(SZ - ZSEA)
    assert g["s_tip"] == pytest.approx(flat_tip(o), abs=1e-9)
    # a legacy record without an altitude still falls back to the contact's range
    legacy = SL.ping_object_bins(POSE, 1, o, bogus, None, altitude=None, **KW)
    assert legacy["dz"] != pytest.approx(SZ - ZSEA)


def test_a_flat_object_has_no_shadow():
    g = SL.ping_object_bins(POSE, 1, obj(h=0.0), {}, ZSEA, **KW)
    assert g["shadow_m"] == pytest.approx(0.0)
