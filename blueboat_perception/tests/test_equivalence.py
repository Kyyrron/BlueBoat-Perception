"""Wire input == file input, so offline records == live records (DESIGN §9)."""

from __future__ import annotations

import json

import numpy as np

from blueboat_perception.core.image import SeabedImageInput
from blueboat_perception.core.pipeline import Pipeline
from blueboat_perception.core.records import comparable_json
from blueboat_perception.core.session import iter_session, list_images
from conftest import envelope, make_db, make_metadata, write_session


def test_wire_and_files_give_identical_records(tmp_path):
    metas = [make_metadata(image_id=i, t0=50.0 + i) for i in range(3)]
    dbs = [make_db(seed=10 + i) for i in range(3)]
    write_session(tmp_path / "s", metas, dbs)
    pipeline = Pipeline()
    live = []
    for meta, db in zip(metas, dbs):
        # the wire: the metadata JSON as the GCS serialises it (json.dumps)
        img = SeabedImageInput.from_wire(envelope(json.loads(json.dumps(meta))), db.copy())
        rec, _ = pipeline.run(img)
        live.append(dict(rec, source="live", provenance={"x": 1}))
    off = []
    for img in iter_session(tmp_path / "s"):
        rec, _ = pipeline.run(img)
        off.append(dict(rec, source="offline", provenance={"y": 2}))
    assert [comparable_json(r) for r in live] == [comparable_json(r) for r in off]


def test_session_reader_orders_by_id_and_falls_back_on_the_folder_name(tmp_path):
    metas = [make_metadata(image_id=i, stream_id=None, schema=4, convention=None)
             for i in (2, 0, 1)]
    write_session(tmp_path / "2026_09_28-10_27_47", metas, [make_db()] * 3)
    files = list_images(tmp_path / "2026_09_28-10_27_47")
    assert [f.image_id for f in files] == [0, 1, 2]
    imgs = list(iter_session(tmp_path / "2026_09_28-10_27_47"))
    assert {i.stream_id for i in imgs} == {"session:2026_09_28-10_27_47"}
    assert imgs[0].key == "session:2026_09_28-10_27_47/0"


def test_a_missing_npz_gives_an_image_without_pixels(tmp_path):
    write_session(tmp_path / "s", [make_metadata()], [make_db()], with_npz=False)
    img = next(iter_session(tmp_path / "s"))
    assert img.db is None
    assert Pipeline().run(img)[0]["processing"]["detector"]["status"] == "no_pixels"


def test_the_float32_array_round_trips_bit_exactly(tmp_path):
    db = make_db(seed=3)
    db[0, 0] = np.float32(1e-38)
    write_session(tmp_path / "s", [make_metadata()], [db])
    img = next(iter_session(tmp_path / "s"))
    assert img.db.dtype == np.float32
    np.testing.assert_array_equal(img.db, db)
