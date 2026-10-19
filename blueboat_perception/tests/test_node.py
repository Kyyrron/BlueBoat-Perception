"""The ROS node in-process: three parts in -> one record + one array out.

Gated on rclpy / sensor_msgs / vision_msgs (``find_spec`` + ``skipif``).
Messages are fed straight into the callbacks and the publishers are
captured, so no DDS discovery is involved.
"""

from __future__ import annotations

import importlib.util
import json

import numpy as np
import pytest

from conftest import envelope, make_db, make_metadata

MISSING = [m for m in ("rclpy", "sensor_msgs", "std_msgs", "vision_msgs")
           if importlib.util.find_spec(m) is None]
pytestmark = pytest.mark.skipif(bool(MISSING), reason=f"needs ROS 2 (missing {MISSING})")


@pytest.fixture
def node_factory(tmp_path):
    import rclpy
    from rclpy.parameter import Parameter
    rclpy.init()
    made = []

    def make(**params):
        from blueboat_perception.ros.perception_node import PerceptionNode
        from rclpy.context import Context  # noqa: F401
        overrides = {"data_dir": str(tmp_path), **params}
        import rclpy.node as rn
        orig = rn.Node.__init__

        def patched(self, name, **kw):
            kw["parameter_overrides"] = [Parameter(k, value=v) for k, v in overrides.items()]
            orig(self, name, **kw)
        rn.Node.__init__ = patched
        try:
            n = PerceptionNode()
        finally:
            rn.Node.__init__ = orig
        sent = {"det": [], "rec": [], "status": []}
        n.pub_det.publish = sent["det"].append
        n.pub_rec.publish = sent["rec"].append
        n.pub_status.publish = sent["status"].append
        made.append(n)
        return n, sent
    yield make
    for n in made:
        n.shutdown()
        n.destroy_node()
    rclpy.shutdown()


def _image_msg(arr, key, encoding):
    from sensor_msgs.msg import Image
    m = Image()
    m.header.frame_id = key
    m.height, m.width = arr.shape
    m.encoding = encoding
    m.is_bigendian = 0
    m.step = arr.shape[1] * arr.dtype.itemsize
    m.data = np.ascontiguousarray(arr).tobytes()
    return m


def _feed(node, meta, db, png=None):
    from std_msgs.msg import String
    key = f"{meta['stream_id']}/{meta['image_id']}"
    node._on_metadata(String(data=envelope(meta)))
    node._on_image(_image_msg(db.astype("<f4"), key, "32FC1"), "image")
    if png is not None:
        node._on_image(_image_msg(png, key, "mono8"), "png")


def test_one_record_and_one_detection_array_per_image(node_factory, tmp_path):
    node, sent = node_factory()
    meta, db = make_metadata(image_id=0), make_db()
    _feed(node, meta, db)
    assert len(sent["rec"]) == 1 and len(sent["det"]) == 1
    rec = json.loads(sent["rec"][0].data)
    assert rec["source"] == "live" and rec["key"].endswith("/0")
    assert rec["provenance"]["join_status"] == "complete"
    arr = sent["det"][0]
    assert len(arr.detections) == 1
    d = arr.detections[0]
    det = rec["detections"][0]
    assert d.id == str(det["uid"]) == "2000000"
    assert d.results[0].hypothesis.class_id == "dummy_center"
    assert d.results[0].hypothesis.score == 0.5
    assert d.bbox.center.position.x == pytest.approx(det["world_xy"][0])
    t = d.header.stamp.sec + d.header.stamp.nanosec * 1e-9
    assert t == pytest.approx(det["t_row_s"], abs=1e-9)
    assert d.results[0].pose.covariance[0] == pytest.approx(det["covariance_xy"][0][0])
    log = tmp_path / "data" / "Perception_data" / "live" / f"{meta['stream_id']}.jsonl"
    assert log.is_file() and len(log.read_text().splitlines()) == 1


def test_the_32fc1_array_arrives_bit_exact(node_factory):
    from blueboat_perception.ros.convert import image_to_array
    db = make_db(seed=4)
    back = image_to_array(_image_msg(db.astype("<f4"), "k/0", "32FC1"))
    np.testing.assert_array_equal(back, db)
    padded = _image_msg(db.astype("<f4"), "k/0", "32FC1")
    padded.step += 8
    padded.data = b"".join(row.tobytes() + b"\0" * 8 for row in db.astype("<f4"))
    np.testing.assert_array_equal(image_to_array(padded), db)


def test_metadata_without_pixels_gives_a_no_pixels_record(node_factory):
    from std_msgs.msg import String
    node, sent = node_factory(join_timeout_s=0.0)
    node._on_metadata(String(data=envelope(make_metadata(image_id=3))))
    node._poll()
    rec = json.loads(sent["rec"][0].data)
    assert rec["detections"] is None
    assert rec["processing"]["detector"]["status"] == "no_pixels"
    assert len(sent["det"][0].detections) == 0


def test_dataset_mode_writes_the_sample(node_factory, tmp_path):
    node, sent = node_factory(mode="dataset", dataset_name="2026_10_16-10_00_00-t")
    meta, db = make_metadata(image_id=0), make_db()
    png = np.nan_to_num(np.clip(db, 0, 60) * 4).astype(np.uint8)
    _feed(node, meta, db)                      # waits for the png in dataset mode
    assert sent["rec"] == []
    node._on_image(_image_msg(png, f"{meta['stream_id']}/0", "mono8"), "png")
    assert len(sent["rec"]) == 1
    d = tmp_path / "data" / "Dataset_data" / "2026_10_16-10_00_00-t"
    assert len(list((d / "images").glob("*.png"))) == 1
    node._publish_status()
    status = json.loads(sent["status"][-1].data)
    assert status["mode"] == "dataset" and status["dataset"]["images"] == 1


def _world(tmp_path):
    import yaml
    w = tmp_path / "world"
    w.mkdir()
    (w / "metadata.yaml").write_text(yaml.safe_dump({"objects": [{
        "object_id": 7, "type": "concrete_block", "x": 0.3, "y": 5.0, "yaw": 0.0,
        "length": 0.4, "width": 0.2, "proud_height": 0.3, "burial": 0.0}]}))
    np.savez(w / "depth.npz", height=np.full((40, 40), -4.0), origin_x=-10.0,
             origin_y=-10.0, resolution=0.5)
    return w


def _labelled_picture_meta():
    import test_sim_labels as T
    meta = make_metadata(H=T.H, W=T.W, pitch=T.PITCH, image_id=0, t0=100.0)
    for i, r in enumerate(meta["rows_data"]):
        r["port_ping_number"] = 1000 + T.H - 1 - i
        r["stbd_ping_number"] = 0
    db = np.zeros((T.H, T.W), np.float32)
    return meta, db, np.zeros((T.H, T.W), np.uint8)


def test_ground_truth_is_read_only_in_dataset_mode_with_a_world(node_factory, tmp_path):
    normal, _ = node_factory()
    assert normal.ground_truth is None and len(list(normal.subscriptions)) == 3
    plain, _ = node_factory(mode="dataset", dataset_name="no-world")
    assert plain.ground_truth is None and len(list(plain.subscriptions)) == 3
    assert plain.dataset.labels.name == "none"
    sim, _ = node_factory(mode="dataset", dataset_name="with-world",
                          world_dir=str(_world(tmp_path)))
    assert sim.ground_truth is not None and len(list(sim.subscriptions)) == 5
    assert sim.dataset.labels.name == "sim_ground_truth/1"


def test_dataset_mode_waits_for_the_ground_truth_and_labels_the_picture(node_factory,
                                                                        tmp_path):
    import test_sim_labels as T
    node, sent = node_factory(mode="dataset", dataset_name="lab",
                              world_dir=str(_world(tmp_path)), label_wait_s=60.0)
    meta, db, png = _labelled_picture_meta()
    _feed(node, meta, db, png)
    assert len(sent["rec"]) == 1                # the record does not wait
    d = tmp_path / "data" / "Dataset_data" / "lab"
    assert list((d / "images").glob("*.png")) == []      # the sample does
    T.feed(node.ground_truth, [T.obj()])
    node._poll()
    (lab,) = (d / "labels").glob("*.txt")
    assert len(lab.read_text().splitlines()) == 1
    node._publish_status()
    st = json.loads(sent["status"][-1].data)["dataset"]
    assert st["labels"]["boxes"] == 1 and st["pending_label_writes"] == 0


def test_shutdown_writes_a_picture_still_waiting_for_ground_truth(node_factory, tmp_path):
    node, _ = node_factory(mode="dataset", dataset_name="late",
                           world_dir=str(_world(tmp_path)), label_wait_s=60.0)
    meta, db, png = _labelled_picture_meta()
    _feed(node, meta, db, png)
    d = tmp_path / "data" / "Dataset_data" / "late"
    assert list((d / "images").glob("*.png")) == []
    node.shutdown()
    (lab,) = (d / "labels").glob("*.txt")
    assert lab.read_text() == ""                # nothing arrived: labelled empty
    side = json.loads(next((d / "images").glob("*.json")).read_text())
    assert side["perception"]["labels"]["ground_truth_coverage"] == 0.0
