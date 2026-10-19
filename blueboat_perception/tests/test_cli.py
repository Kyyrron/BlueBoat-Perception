"""The offline CLI over a session; refusal inside the session; --compare."""

from __future__ import annotations

import importlib.util
import json

import pytest

from blueboat_perception.cli import offline
from blueboat_perception.core.records import canonical_json, read_jsonl
from conftest import write_session


def test_offline_run_writes_records_detections_summary(tmp_path, synthetic_session):
    rc = offline.main([str(synthetic_session), "--data-dir", str(tmp_path / "root")])
    assert rc == 0
    out = tmp_path / "root" / "data" / "Perception_data" / "offline" / synthetic_session.name
    recs = list(read_jsonl(out / "records.jsonl"))
    assert [r["image_id"] for r in recs] == [0, 1, 2, 3]
    assert all(r["source"] == "offline" for r in recs)
    dets = json.loads((out / "detections.json").read_text())
    assert len(dets) == 4
    summ = json.loads((out / "summary.json").read_text())
    assert summ["images"] == 4 and summ["status"] == {"ran": 4}
    # a second run claims a new folder
    assert offline.main([str(synthetic_session), "--data-dir", str(tmp_path / "root")]) == 0
    assert (out.parent / f"{synthetic_session.name}-2" / "records.jsonl").is_file()


def test_refuses_an_output_inside_the_session(tmp_path, synthetic_session):
    assert offline.main([str(synthetic_session), "--out",
                         str(synthetic_session / "perception")]) == 2
    assert not (synthetic_session / "perception").exists()


def test_compare_against_a_live_log(tmp_path, synthetic_session):
    root = tmp_path / "root"
    assert offline.main([str(synthetic_session), "--data-dir", str(root)]) == 0
    out = root / "data" / "Perception_data" / "offline" / synthetic_session.name
    live = tmp_path / "live.jsonl"
    with open(live, "w", encoding="utf-8") as f:
        for r in read_jsonl(out / "records.jsonl"):
            r.update(source="live", provenance={"latency_ms": {"total": 1.0}})
            f.write(canonical_json(r) + "\n")
    assert offline.main([str(synthetic_session), "--data-dir", str(root),
                         "--compare", str(live)]) == 0
    summ = json.loads((out.parent / f"{synthetic_session.name}-2" / "summary.json").read_text())
    assert summ["compare"]["identical"] == summ["compare"]["compared"] == 4
    # a tampered live record is reported and fails the run
    lines = live.read_text().splitlines()
    rec = json.loads(lines[1])
    rec["track"]["x_m"][0] += 1e-9
    lines[1] = canonical_json(rec)
    live.write_text("\n".join(lines) + "\n")
    assert offline.main([str(synthetic_session), "--data-dir", str(root),
                         "--compare", str(live)]) == 1


def test_dataset_from_a_session(tmp_path, synthetic_session):
    root = tmp_path / "root"
    assert offline.main([str(synthetic_session), "--data-dir", str(root),
                         "--dataset", "offline-ds"]) == 0
    d = root / "data" / "Dataset_data" / "offline-ds"
    assert len(list((d / "images").glob("*.png"))) == 4
    assert json.loads((d / "manifest.json").read_text())["mode"] == "offline"


HAVE_BAG = all(importlib.util.find_spec(m) is not None
               for m in ("rosbag2_py", "rclpy", "std_msgs"))


class _Collect:
    def __init__(self):
        self.msgs = []

    def add_sensor(self, d):
        self.msgs.append(("/side_scan_sonar/ground_truth/sensor", d))

    def add_contacts(self, d):
        self.msgs.append(("/side_scan_sonar/ground_truth/contacts", d))


@pytest.mark.skipif(not HAVE_BAG, reason="needs rosbag2_py, rclpy and std_msgs")
def test_dataset_labels_from_a_recorded_ground_truth_bag(tmp_path):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String

    import test_node
    import test_sim_labels as T
    world = test_node._world(tmp_path)
    meta, db, _png = test_node._labelled_picture_meta()
    session = write_session(tmp_path / "sessions" / "s1", [meta], [db])
    col = _Collect()
    T.feed(col, [T.obj()])
    bag = tmp_path / "ground_truth_bag"
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    for k, name in enumerate(sorted({t for t, _ in col.msgs})):
        w.create_topic(rosbag2_py.TopicMetadata(id=k, name=name, type="std_msgs/msg/String",
                                                serialization_format="cdr"))
    for i, (topic, d) in enumerate(col.msgs):
        w.write(topic, serialize_message(String(data=json.dumps(d))), 1_000_000 * (i + 1))
    del w
    root = tmp_path / "root"
    assert offline.main([str(session), "--data-dir", str(root), "--dataset", "gt",
                         "--world", str(world), "--ground-truth-bag", str(bag)]) == 0
    d = root / "data" / "Dataset_data" / "gt"
    (lab,) = (d / "labels").glob("*.txt")
    assert len(lab.read_text().splitlines()) == 1
    m = json.loads((d / "manifest.json").read_text())
    assert m["label_source"] == "sim_ground_truth/1" and m["labels"]["boxes"] == 1


def test_world_and_bag_go_together_with_a_dataset(tmp_path, synthetic_session):
    assert offline.main([str(synthetic_session), "--data-dir", str(tmp_path),
                         "--world", str(tmp_path)]) == 2
