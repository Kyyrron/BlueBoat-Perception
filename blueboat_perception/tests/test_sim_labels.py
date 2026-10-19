"""YOLO labels from simulator ground truth (``core/sim_labels.py``), on synthetic
pictures, sensor poses and contacts."""

from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from conftest import make_metadata
from blueboat_perception.core import sim_labels as SL
from blueboat_perception.core.image import SeabedImageInput
from blueboat_perception.core.pipeline import Pipeline
from blueboat_perception.core.records import canonical_json

H, W, PITCH = 40, 1200, 20.0 / 599     # GCS pitch L/(N - 1), N = 600 bins per side
N = W // 2
BIN = PITCH * (N - 1) / N              # the device's own bin, L/N
SZ, ZSEA = -0.2, -4.0                  # transducer and seabed heights
STEP = 0.02                            # along-track metres per ping
SIM_REPO = Path(__file__).resolve().parents[3] / "BlueBoat-SSS-Sim" / "blueboat_sss_sim"


def obj(oid=7, typ="concrete_block", x=0.3, y=5.0, yaw=0.0, L=0.4, Wd=0.2, h=0.3):
    return SL.WorldObject(object_id=oid, type=typ, x=x, y=y, yaw=yaw, length=L, width=Wd,
                          height=h)


def world(*objects, terrain=True):
    t = None
    if terrain:
        t = SL.Terrain(np.full((40, 40), ZSEA), -10.0, -10.0, 0.5, False, "test")
    return SL.WorldInfo({o.object_id: o for o in objects}, "test.yaml", "0" * 64, t)


def picture(first_ping=1000, t0=100.0, image_id=0, side_numbers=("port",), png=False):
    """Row 0 newest; row i shows ping first_ping + H - 1 - i at t0 - 0.05 i."""
    meta = make_metadata(H=H, W=W, pitch=PITCH, image_id=image_id, t0=t0)
    for i, r in enumerate(meta["rows_data"]):
        n = first_ping + H - 1 - i
        r["port_ping_number"] = n if "port" in side_numbers else 0
        r["stbd_ping_number"] = n if "starboard" in side_numbers else 0
    db = np.zeros((H, W), np.float32)
    return SeabedImageInput.from_metadata(meta, db, np.zeros((H, W), np.uint8) if png else None)


def feed(buf, objects, *, first_ping=1000, t0=100.0, n_pings=H + 1, side="port",
         contact_extra=None, contacts_first=False):
    """One sensor message per ping (the boat moving +x at STEP per ping, yaw 0)
    and the renderer's contacts for every object (all within range)."""
    sign = 1 if side == "port" else -1
    for j in range(n_pings):
        n = first_ping + j
        stamp = t0 - 0.05 * (H - 1 - j)
        t_sim = stamp - 50.0
        pose = [STEP * j, sign * 0.285, SZ, 0.0, 0.0, 0.0]
        sensor = {"t_sim": round(t_sim, 4), "stamp": round(stamp, 4),
                  "sensors": {side: {"pose": pose, "altitude_m": SZ - ZSEA,
                                     "ping_number": n, "dropped": False}}}
        cs = []
        for o in objects:
            across = sign * (o.y - pose[1])
            c = {"side": side, "object_id": o.object_id, "type": o.type,
                 "slant_range_m": round(math.hypot(across, SZ - ZSEA - o.height), 3),
                 "extent_bins": 9.0, "shadow_bins": 3.0, "visible": True,
                 "ping_number": n, "ghost": False, "via": ""}
            c.update(contact_extra or {})
            cs.append(c)
        msg = {"t_sim": round(t_sim, 3), "contacts": cs}
        if contacts_first:
            buf.add_contacts(msg)
            buf.add_sensor(sensor)
        else:
            buf.add_sensor(sensor)
            buf.add_contacts(msg)


def source(objects, buf, **kw):
    return SL.SimGroundTruthLabelSource(world(*objects), buf, **kw)


def expected_bins(o, x_sensor, side="port"):
    """The documented geometry, written out for an axis-aligned object."""
    sign = 1 if side == "port" else -1
    across = sign * (o.y - sign * 0.285)
    dz = SZ - ZSEA
    R = math.hypot(across, dz)
    h = min(3.0 * R * math.radians(0.5) / 2.355, BIN * N * math.radians(0.5) / 2.0)
    if abs(o.x - x_sensor) > o.length / 2 + h:
        return None
    a_min, a_max = across - o.width / 2, across + o.width / 2
    s_near = math.hypot(a_min, dz - o.height)
    s_hl = max(math.hypot(a_max, dz - o.height), math.hypot(a_min, dz))
    s_tip = math.hypot(a_max * dz / (dz - o.height), dz)
    s_mask = math.hypot(a_max, dz) + 3.0 * BIN
    s_end = max(s_tip, s_mask, s_hl)
    return (math.floor(s_near / BIN) - 2, math.ceil(s_hl / BIN) - 1 + 2,
            math.ceil(s_end / BIN) - 1 + 2)


# --------------------------------------------------------------------- geometry
def test_box_matches_the_documented_geometry_port():
    o = obj()
    buf = SL.GroundTruthBuffer()
    feed(buf, [o])
    src = source([o], buf)
    img = picture()
    d = src.detail_for(img)
    assert d["ground_truth_coverage"] == 1.0
    (b,) = d["boxes"]
    exp = {i: expected_bins(o, STEP * (H - 1 - i)) for i in range(H)}
    rows = [i for i, e in exp.items() if e is not None]
    k0 = min(exp[i][0] for i in rows)
    k2 = max(exp[i][2] for i in rows)
    assert (b["box_px"]["r0"], b["box_px"]["r1"]) == (rows[0], rows[-1])
    assert (b["box_px"]["c0"], b["box_px"]["c1"]) == (N - 1 - k2, N - 1 - k0)
    assert b["type"] == "concrete_block" and b["side"] == "port"
    assert b["highlight_cols"][1] == N - 1 - k0
    assert b["shadow_cols"][0] == b["box_px"]["c0"]
    # the YOLO line is the box, normalised
    (yb,) = src.labels_for(img)
    p = b["box_px"]
    assert yb.cx == pytest.approx((p["c0"] + p["c1"] + 1) / 2 / W, abs=1e-6)
    assert yb.cy == pytest.approx((p["r0"] + p["r1"] + 1) / 2 / H, abs=1e-6)
    assert yb.w == pytest.approx((p["c1"] - p["c0"] + 1) / W, abs=1e-6)
    assert yb.h == pytest.approx((p["r1"] - p["r0"] + 1) / H, abs=1e-6)
    assert yb.class_id == SL.load_class_map().index("concrete_block")


def test_starboard_mirrors_port():
    o = obj(y=-5.0)
    buf = SL.GroundTruthBuffer()
    feed(buf, [o], side="starboard")
    (b,) = source([o], buf).detail_for(picture(side_numbers=("starboard",)))["boxes"]
    po = obj()
    pbuf = SL.GroundTruthBuffer()
    feed(pbuf, [po])
    (pb,) = source([po], pbuf).detail_for(picture())["boxes"]
    assert b["side"] == "starboard"
    assert (b["box_px"]["c0"], b["box_px"]["c1"]) == (W - 1 - pb["box_px"]["c1"],
                                                      W - 1 - pb["box_px"]["c0"])
    assert (b["box_px"]["r0"], b["box_px"]["r1"]) == (pb["box_px"]["r0"], pb["box_px"]["r1"])


def test_a_tall_object_starts_at_its_top_and_its_shadow_is_longer():
    low, tall = obj(h=0.05), obj(h=0.8)
    boxes = []
    for o in (low, tall):
        buf = SL.GroundTruthBuffer()
        feed(buf, [o])
        boxes.append(source([o], buf).detail_for(picture())["boxes"][0]["box_px"])
    assert boxes[1]["c1"] > boxes[0]["c1"]     # port: a larger column is nearer
    assert boxes[1]["c0"] < boxes[0]["c0"]     # and its shadow reaches farther


def test_the_renderers_longer_mask_shadow_wins():
    o = obj()
    a, b = SL.GroundTruthBuffer(), SL.GroundTruthBuffer()
    feed(a, [o])
    feed(b, [o], contact_extra={"shadow_bins": 200.0})
    ca = source([o], a).detail_for(picture())["boxes"][0]["box_px"]["c0"]
    cb = source([o], b).detail_for(picture())["boxes"][0]["box_px"]["c0"]
    assert ca - cb >= 150


def test_a_thin_oblique_object_is_seen_where_the_beam_crosses_it():
    rope = obj(typ="rope", yaw=math.radians(60.0), L=4.0, Wd=0.03, h=0.02, x=0.4)
    buf = SL.GroundTruthBuffer()
    feed(buf, [rope])
    src = source([rope], buf)
    g_mid = SL.ping_object_bins([0.4, 0.285, SZ, 0, 0, 0], 1, rope, {"shadow_bins": 0.0},
                                ZSEA, bin_m=BIN, range_max_m=BIN * N, n_bins=N)
    # one ping crosses the rope over a few centimetres, not its 3.5 m across extent
    assert g_mid["a_max"] - g_mid["a_min"] < 0.3
    (b,) = src.detail_for(picture())["boxes"]
    # the box is the union of the crossings the pings swept, narrower than the rope
    assert (b["box_px"]["c1"] - b["box_px"]["c0"]) * BIN < 4.0 * math.sin(math.radians(60))


def test_rows_outside_the_beam_strip_are_not_labelled():
    o = obj(x=0.0, L=0.1)                       # only the oldest pings pass over it
    buf = SL.GroundTruthBuffer()
    feed(buf, [o])
    d = source([o], buf).detail_for(picture())
    assert d["skipped"]["outside_beam"] > 0
    (b,) = d["boxes"]
    assert b["box_px"]["r1"] == H - 1 and b["box_px"]["r0"] > H // 2


# ------------------------------------------------------------------- filtering
@pytest.mark.parametrize("extra,key", [({"ghost": True, "via": "wall:q"}, "ghost"),
                                       ({"visible": False}, "invisible")])
def test_ghosts_and_hidden_objects_are_not_labelled(extra, key):
    o = obj()
    buf = SL.GroundTruthBuffer()
    feed(buf, [o], contact_extra=extra)
    d = source([o], buf).detail_for(picture())
    assert d["boxes"] == [] and d["skipped"][key] > 0


def test_wall_echoes_are_not_labelled():
    o = obj()
    buf = SL.GroundTruthBuffer()
    feed(buf, [o], contact_extra={"object_id": "wall:quay", "type": "wall"})
    d = source([o], buf).detail_for(picture())
    assert d["boxes"] == [] and d["skipped"]["wall"] > 0


def test_a_picture_without_objects_has_an_empty_label():
    buf = SL.GroundTruthBuffer()
    feed(buf, [])
    src = source([obj()], buf)
    img = picture()
    assert src.labels_for(img) == []
    assert src.detail_for(img)["ground_truth_coverage"] == 1.0


def test_min_rows_and_split_runs():
    o = obj()
    buf = SL.GroundTruthBuffer()
    feed(buf, [o])
    assert source([o], buf, min_rows=H + 1).detail_for(picture())["boxes"] == []
    d = source([o], buf, min_rows=H + 1).detail_for(picture())
    assert d["boxes_below_min_rows"] == 1


# --------------------------------------------------------------------- joining
def test_a_counter_restart_is_resolved_by_time():
    near, far = obj(oid=1, y=4.0), obj(oid=2, y=9.0)
    buf = SL.GroundTruthBuffer()
    feed(buf, [near], t0=100.0)                 # first power-up: object 1
    feed(buf, [far], t0=400.0)                  # same ping numbers later: object 2
    src = source([near, far], buf)
    assert [b["object_id"] for b in src.detail_for(picture(t0=100.0))["boxes"]] == [1]
    assert [b["object_id"] for b in
            src.detail_for(picture(t0=400.0, image_id=1))["boxes"]] == [2]


def test_contacts_may_arrive_before_their_sensor_message():
    o = obj()
    a, b = SL.GroundTruthBuffer(), SL.GroundTruthBuffer()
    feed(a, [o])
    feed(b, [o], contacts_first=True)
    assert b.stats()["orphan_contacts"] == 0
    assert (source([o], a).detail_for(picture())["boxes"]
            == source([o], b).detail_for(picture())["boxes"])


def test_ready_needs_every_source_ping_and_the_next_one():
    o = obj()
    buf = SL.GroundTruthBuffer()
    feed(buf, [o], n_pings=H)                   # up to the newest row, not beyond
    src = source([o], buf)
    img = picture()
    assert src.coverage(img) == (H, H)
    assert not src.ready(img)
    feed(buf, [o], n_pings=H + 1)
    assert src.ready(picture())


def test_truncation_flags_say_whether_the_object_goes_on():
    o = obj(x=STEP * (H - 1))                   # centred on the newest ping
    buf = SL.GroundTruthBuffer()
    feed(buf, [o], n_pings=H + 20)
    (b,) = source([o], buf).detail_for(picture())["boxes"]
    assert b["box_px"]["r0"] == 0 and b["truncated_top"]
    assert not b["truncated_bottom"]


def test_missing_ground_truth_lowers_the_coverage():
    o = obj()
    buf = SL.GroundTruthBuffer()
    feed(buf, [o], n_pings=H // 2)
    d = source([o], buf).detail_for(picture())
    assert 0.0 < d["ground_truth_coverage"] < 1.0


# ----------------------------------------------------------- classes and world
def test_class_ids_are_frozen_and_unknown_types_are_appended():
    base = SL.load_class_map()
    assert len(base) == 29 and base[0] == "tire_car" and base[-1] == "shopping_trolley"
    classes = SL.classes_for([obj(typ="can"), obj(typ="zz_new"), obj(typ="aa_new")], base)
    assert classes[base.index("can")] == "can"
    assert [classes[29], classes[30]] == ["aa_new", "zz_new"]


@pytest.mark.skipif(not (SIM_REPO / "blueboat_sss_sim" / "worldgen" / "objects.py").is_file(),
                    reason="the simulator's source is not beside this module")
def test_the_class_map_is_the_world_builder_catalog():
    code = ("import sys; sys.path.insert(0, sys.argv[1]);"
            "from blueboat_sss_sim.worldgen.objects import CATALOG;"
            "print('\\n'.join(CATALOG))")
    r = subprocess.run([sys.executable, "-I", "-c", code, str(SIM_REPO)],
                       capture_output=True, text=True, timeout=120)
    if r.returncode != 0:
        pytest.skip(f"simulator not importable: {r.stderr[-200:]}")
    catalog = r.stdout.split()
    v1 = SL.load_class_map()
    # append-only: v1 is a prefix of the catalog; a longer catalog needs a v2 file
    assert catalog[:len(v1)] == v1
    assert len(catalog) == len(v1), "the World Builder has new classes: write yolo_classes_v2"


def test_load_world_reads_objects_and_terrain(tmp_path):
    yaml = pytest.importorskip("yaml")
    doc = {"objects": [{"object_id": 3, "type": "can", "x": 1.0, "y": 2.0, "yaw": 0.5,
                        "length": 0.1, "width": 0.09, "proud_height": 0.1, "burial": 0.5}]}
    (tmp_path / "metadata.yaml").write_text(yaml.safe_dump(doc))
    np.savez(tmp_path / "depth.npz", height=np.full((10, 10), -3.0), origin_x=0.0,
             origin_y=0.0, resolution=0.5)
    w = SL.load_world(str(tmp_path))
    o = w.objects[3]
    assert (o.type, o.height) == ("can", pytest.approx(0.05))
    assert w.terrain is not None and not w.terrain.stamped
    assert w.terrain.seabed_z(o) == pytest.approx(-3.0)
    # a CLI world: scene_manifest.yaml + scene.npz, objects stamped in
    cli = tmp_path / "cli"
    cli.mkdir()
    (cli / "scene_manifest.yaml").write_text(yaml.safe_dump(
        {"version": 1, "grid": {"origin_x": 0.0, "origin_y": 0.0, "resolution": 0.1,
                                "nx": 40, "ny": 40}, **doc}))
    h = np.full((40, 40), -3.0)
    h[19:22, 9:12] = -2.9                        # the stamped object at (1, 2)
    np.savez(cli / "scene.npz", height=h)
    w2 = SL.load_world(str(cli))
    assert w2.terrain.stamped
    assert w2.terrain.seabed_z(w2.objects[3]) == pytest.approx(-3.0)


def test_labels_are_deterministic():
    o = obj()
    outs = []
    for _ in range(2):
        buf = SL.GroundTruthBuffer()
        feed(buf, [o])
        outs.append(json.dumps(source([o], buf).detail_for(picture()), sort_keys=True))
    assert outs[0] == outs[1]


def test_records_are_identical_with_and_without_ground_truth_labels():
    img = picture()
    before = canonical_json(Pipeline().run(img))
    buf = SL.GroundTruthBuffer()
    feed(buf, [obj()])
    assert source([obj()], buf).labels_for(img)
    assert canonical_json(Pipeline().run(img)) == before
