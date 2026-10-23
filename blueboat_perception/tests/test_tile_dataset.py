"""The tile dataset writer (tools/make_tile_dataset.py) and the class-drift
checks of every dataset-creation path."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

from blueboat_perception.core.dataset import DatasetWriter
from blueboat_perception.core.image import SeabedImageInput
from blueboat_perception.core.labels import NullLabelSource, YoloBox
from blueboat_perception.core.pipeline import Pipeline
from blueboat_perception.core import sim_labels as SL
from conftest import PKG_ROOT, PY_PKG, envelope, make_metadata

TOOLS = PKG_ROOT.parent / "tools"
NEEDS = [m for m in ("cv2", "yaml") if importlib.util.find_spec(m) is None]
pytestmark = pytest.mark.skipif(bool(NEEDS), reason=f"needs {NEEDS}")
V1 = SL.load_class_map()
H, W = 256, 1200


def _tool():
    if str(TOOLS) not in sys.path:
        sys.path.insert(0, str(TOOLS))
    import make_tile_dataset
    return make_tile_dataset


class FakeLabels:
    """A label source with the simulator source's interface."""

    name = SL.LABEL_SOURCE_NAME

    def __init__(self, boxes, classes=None):
        self.classes = classes or {i: n for i, n in enumerate(V1)}
        self.boxes = boxes

    def detail_for(self, img):
        return {"source": self.name, "boxes": self.boxes.get(img.image_id, [])}

    def labels_for(self, img):
        out = []
        for b in self.boxes.get(img.image_id, []):
            p = b["box_px"]
            out.append(YoloBox(b["class_id"], (p["c0"] + p["c1"] + 1) / 2 / W,
                               (p["r0"] + p["r1"] + 1) / 2 / H, (p["c1"] - p["c0"] + 1) / W,
                               (p["r1"] - p["r0"] + 1) / H))
        return out


def box(c0, c1, h0, h1, r0, r1, cls, side="port", shadow_m=1.0, oid=1):
    hl = [h0, h1]
    sh = [c0, h0 - 1] if side == "port" else [h1 + 1, c1]
    return {"object_id": oid, "type": V1[cls] if cls < len(V1) else "zz_new",
            "class_id": cls, "side": side, "box_px": {"r0": r0, "r1": r1, "c0": c0, "c1": c1},
            "highlight_cols": hl, "shadow_cols": sh, "shadow_m": shadow_m, "rows": r1 - r0 + 1}


def scene(seed=0):
    rng = np.random.default_rng(seed)
    db = (30.0 + 1.0 * rng.standard_normal((H, W))).astype(np.float32)
    db[:, W // 2 - 3: W // 2 + 3] = np.nan
    A = box(300, 470, 430, 470, 40, 89, cls=4)                      # visible, port
    db[40:90, 430:471] += 15.0                                       # bright highlight
    db[40:90, 300:430] -= 15.0                                       # dark shadow
    F = box(900, 940, 900, 915, 100, 140, cls=6, side="starboard", oid=2)   # faint
    return db, {0: [A, F]}


def make_dataset(root, name, boxes, db, world="", labels="fake", classes=None):
    meta = make_metadata(H=H, W=W, pitch=20.0 / (W // 2), image_id=0)
    img = SeabedImageInput.from_wire(envelope(meta), db)
    pipe = Pipeline()
    src = FakeLabels(boxes, classes) if labels == "fake" else NullLabelSource()
    w = DatasetWriter.create(root, name, world_dir=world, label_source=src,
                             detector_input=pipe.detector_input_info())
    rec, _d, inp = pipe.run_full(img)
    assert w.write(img, rec, inp)
    w.close()
    return w.dir


def split_file(tmp_path, doc):
    p = tmp_path / "split.json"
    p.write_text(json.dumps(doc))
    return p


def run_tool(tmp_path, datasets, split, *extra):
    m = _tool()
    rc = m.main([*map(str, datasets), "--split", str(split), "--name", "t",
                 "--data-dir", str(tmp_path / "root"), *extra])
    assert rc == 0
    out = tmp_path / "root" / "data" / "Tile_data" / "t"
    return out, json.loads((out / "manifest.json").read_text())


def labels_of(out, split):
    return {p.stem: p.read_text().split("\n")[:-1]
            for p in sorted((out / "labels" / split).glob("*.txt"))}


# ------------------------------------------------------------------ tile writer
def test_rule_b_i_visibility_and_background_share(tmp_path):
    root = tmp_path / "root"
    db, boxes = scene()
    tr = make_dataset(root, "w_train", boxes, db, world="/worlds/world_a")
    va = make_dataset(root, "w_val", boxes, db, world="/worlds/world_b")
    out, man = run_tool(tmp_path, [tr, va],
                        split_file(tmp_path, {"train": ["world_a"], "val": ["world_b"]}))
    lt, lv = labels_of(out, "train"), labels_of(out, "val")
    # train: the two tiles holding A's highlight; the empty tiles sampled away (10 %)
    assert sorted(lt) == ["w_train__gcs-20261016T000000Z-abcd_00000_c0256",
                          "w_train__gcs-20261016T000000Z-abcd_00000_c0384"]
    whole = lt["w_train__gcs-20261016T000000Z-abcd_00000_c0256"][0].split()
    assert whole[0] == "4"
    assert float(whole[3]) == pytest.approx(171 / 256, abs=1e-6)      # [300, 470] whole
    clipped = lt["w_train__gcs-20261016T000000Z-abcd_00000_c0384"][0].split()
    assert float(clipped[3]) == pytest.approx(87 / 256, abs=1e-6)     # [384, 470]: shadow cut
    c = man["counts"]["train"]
    assert c["dropped_partial"] == 1                 # tile 128: no highlight -> label dropped
    assert c["boxes_faint"] == 1 and c["units_excluded_faint"] == 2   # tiles 768 and 896
    assert c["empty_dropped"] > 0
    # val keeps every unit (minus the faint-excluded ones): 9 tiles - 2
    assert len(lv) == 7 and sum(1 for v in lv.values() if v) == 2
    assert man["splits"] == {"train": ["world_a"], "val": ["world_b"]}
    assert man["classes"] == {"map": "yolo_classes_v1", "nc": 29}
    assert man["data"] == "simulated" and man["seed"] == 0
    y = (out / "data.yaml").read_text()
    assert "train: images/train" in y and "nc: 29" in y and "channels" not in y
    assert "channels: 1" in (out / "data_gray1.yaml").read_text()
    side = json.loads(next((out / "images" / "train").glob("*.json")).read_text())
    assert side["schema"] == "blueboat_perception/tile_sidecar/1"
    assert side["intensity_mapping"] == "log" and "slant_range_m" not in side
    import cv2
    img = cv2.imread(str(next((out / "images" / "train").glob("*.png"))), cv2.IMREAD_UNCHANGED)
    assert img.shape == (256, 256) and img.dtype == np.uint8


def test_the_tiles_are_the_inference_tiles(tmp_path):
    from blueboat_perception.core.preprocess import FixedGray
    from blueboat_perception.core.tiling import cut_tiles
    import cv2
    root = tmp_path / "root"
    db, boxes = scene()
    tr = make_dataset(root, "w", boxes, db, world="/worlds/world_a")
    out, _ = run_tool(tmp_path, [tr], split_file(tmp_path, {"val": ["world_a"]}))
    meta = make_metadata(H=H, W=W, pitch=20.0 / (W // 2), image_id=0)
    inp = FixedGray().run(SeabedImageInput.from_wire(envelope(meta), db))
    ref = {t.c0: t.gray for t in cut_tiles(inp.gray, inp.valid)}
    for p in (out / "images" / "val").glob("*.png"):
        c0 = int(p.stem.rsplit("_c", 1)[1])
        np.testing.assert_array_equal(cv2.imread(str(p), cv2.IMREAD_UNCHANGED), ref[c0])


def test_full_mode_writes_whole_pictures(tmp_path):
    root = tmp_path / "root"
    db, boxes = scene()
    boxes = {0: [boxes[0][0]]}                                       # no faint box
    tr = make_dataset(root, "w", boxes, db, world="/worlds/world_a")
    out, man = run_tool(tmp_path, [tr], split_file(tmp_path, {"val": ["world_a"]}),
                        "--mode", "full")
    (lab,) = labels_of(out, "val").values()
    assert len(lab) == 1 and float(lab[0].split()[3]) == pytest.approx(171 / W, abs=1e-6)
    assert man["mode"] == "full"


def test_tiling_is_reproducible(tmp_path):
    db, boxes = scene()
    outs = []
    for k in range(2):
        root = tmp_path / f"r{k}"
        tr = make_dataset(root, "w", boxes, db, world="/worlds/world_a")
        m = _tool()
        assert m.main([str(tr), "--split", str(split_file(tmp_path, {"train": ["world_a"]})),
                       "--name", "t", "--data-dir", str(root), "--empty-fraction", "0.5"]) == 0
        o = root / "data" / "Tile_data" / "t"
        outs.append({p.name: p.read_bytes() for p in sorted((o / "labels" / "train").glob("*"))})
    assert outs[0] == outs[1]


# ------------------------------------------------------------------ split rules
def test_split_errors(tmp_path):
    m = _tool()
    with pytest.raises(m.DatasetRefused):
        m.load_split(split_file(tmp_path, {"train": ["a"], "val": ["a"]}))
    with pytest.raises(m.DatasetRefused):
        m.load_split(split_file(tmp_path, {"training": ["a"]}))
    db, boxes = scene()
    tr = make_dataset(tmp_path / "root", "w", boxes, db, world="/worlds/world_z")
    with pytest.raises(m.DatasetRefused):
        run_tool(tmp_path, [tr], split_file(tmp_path, {"train": ["world_a"]}))


# ------------------------------------------------------------------ class drift
def test_a_world_only_class_is_refused(tmp_path):
    m = _tool()
    db, boxes = scene()
    classes = {**{i: n for i, n in enumerate(V1)}, 29: "zz_new"}
    boxes = {0: [box(300, 470, 430, 470, 40, 89, cls=29)]}
    tr = make_dataset(tmp_path / "root", "w", boxes, db, world="/worlds/world_a",
                      classes=classes)
    with pytest.raises(m.DatasetRefused, match="class drift"):
        run_tool(tmp_path, [tr], split_file(tmp_path, {"train": ["world_a"]}))


def test_renamed_or_reordered_classes_are_refused(tmp_path):
    m = _tool()
    db, boxes = scene()
    swapped = {i: n for i, n in enumerate(V1)}
    swapped[0], swapped[1] = swapped[1], swapped[0]
    tr = make_dataset(tmp_path / "root", "w", boxes, db, world="/worlds/world_a",
                      classes=swapped)
    with pytest.raises(m.DatasetRefused, match="class drift"):
        run_tool(tmp_path, [tr], split_file(tmp_path, {"train": ["world_a"]}))


def test_an_unlabelled_dataset_needs_the_flag(tmp_path):
    m = _tool()
    db, _ = scene()
    tr = make_dataset(tmp_path / "root", "w", {}, db, world="/worlds/world_a", labels="none")
    assert "names: {}" in (tr / "data.yaml").read_text()
    with pytest.raises(m.DatasetRefused, match="no labels"):
        run_tool(tmp_path, [tr], split_file(tmp_path, {"val": ["world_a"]}))
    out, man = run_tool(tmp_path, [tr], split_file(tmp_path, {"val": ["world_a"]}),
                        "--unlabelled")
    assert all(not v for v in labels_of(out, "val").values())
    assert "names:\n  0: tire_car" in (out / "data.yaml").read_text()   # v1, not {}


def test_the_simulator_label_source_uses_the_frozen_classes(tmp_path):
    """Live (node) and offline (CLI --world --ground-truth-bag) both build
    SimGroundTruthLabelSource with the default class map: ids 0-28 are v1."""
    world = SL.WorldInfo(objects={1: SL.WorldObject(1, "can", 0, 0, 0, 0.1, 0.1, 0.1)},
                         path="x", sha256="0")
    src = SL.SimGroundTruthLabelSource(world, SL.GroundTruthBuffer())
    assert [src.classes[i] for i in range(29)] == V1 and len(src.classes) == 29
    # the node and the CLI pass no class list of their own
    node = (PY_PKG / "ros" / "perception_node.py").read_text()
    cli = (PY_PKG / "cli" / "offline.py").read_text()
    assert "SimGroundTruthLabelSource(world, self.ground_truth," in node
    assert "SimGroundTruthLabelSource(load_world(str(world_dir)), buffer, min_rows=min_rows)" in cli
    for text in (node, cli):
        assert "class_map=" not in text and "classes=" not in text


def test_the_cli_path_writes_v1(tmp_path, monkeypatch):
    yaml = pytest.importorskip("yaml")
    from blueboat_perception.cli import offline
    (tmp_path / "w").mkdir()
    (tmp_path / "w" / "metadata.yaml").write_text(yaml.safe_dump(
        {"objects": [{"object_id": 1, "type": "anchor", "x": 0, "y": 0}]}))
    monkeypatch.setattr(offline, "read_ground_truth_bag", lambda bag: iter(()))
    src = offline.ground_truth_labels(tmp_path / "w", tmp_path / "bag")
    assert [src.classes[i] for i in range(29)] == V1


AUG = Path(__file__).resolve().parents[3] / "SSS-Dataset-Aug-Studio"


@pytest.mark.skipif(not (importlib.util.find_spec("pydantic") and (AUG / "sss_aug_studio").is_dir()),
                    reason="needs pydantic and the SSS-Dataset-Aug-Studio sibling")
def test_the_tile_sidecar_loads_but_declares_no_geometry(tmp_path):
    """Versioned (tile_sidecar/1): the intensity declaration stays valid for the
    Aug Studio; no across-track geometry is declared (a tile does not start at
    the nadir), so the Aug Studio runs on the full-picture dataset only."""
    sys.path.insert(0, str(AUG))
    try:
        from sss_aug_studio.core.meta import AcquisitionMeta
    finally:
        sys.path.remove(str(AUG))
    db, boxes = scene()
    tr = make_dataset(tmp_path / "root", "w", boxes, db, world="/worlds/world_a")
    out, _ = run_tool(tmp_path, [tr], split_file(tmp_path, {"val": ["world_a"]}))
    side = json.loads(next((out / "images" / "val").glob("*.json")).read_text())
    am = AcquisitionMeta(**side)
    src = json.loads(next((tr / "images").glob("*.json")).read_text())
    assert am.intensity_mapping == "log" and am.log_range_db == pytest.approx(src["log_range_db"])
    assert am.slant_range_m is None and am.res_across_m is None
