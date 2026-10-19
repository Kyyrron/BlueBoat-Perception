"""The Perception ROS node (thin): join -> core pipeline -> publish.

Subscribes to the three GCS seabed-image topics (RELIABLE / KEEP_LAST 32 /
VOLATILE, the GCS publisher's profile -- CM-4). It never subscribes to sea
state, the sonar or the boat. **Dataset mode in a simulator world only**
(owner's decision, 2026-10-16): it also reads the simulator's ground truth
(``ground_truth.py``), which feeds the dataset label writer and nothing else.
The pipeline, the detections and the records never see it.

Publishes:
* ``/sss_ai/detections`` (``vision_msgs/Detection2DArray``, one per image,
  possibly empty) -- the GCS ``DetectionsListener``'s existing slot;
* ``/sss_ai/image_records`` (JSON, one record per image);
* ``/sss_ai/perception/status`` (JSON, 1 Hz, latched).

Contract: ``docs/topics.md``. Latency is wall clock, measured from the
receipt of an image's last part to the end of publishing; data times are
the images' own (sonar ROS time), so the node needs no ``use_sim_time``.
"""

from __future__ import annotations

import json
import signal
import socket
import sys
import time
from collections import deque
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from .. import __version__
from ..core.dataroot import data_root, perception_dir, sss_data_dir
from ..core.dataset import DatasetError, DatasetWriter
from ..core.image import InputError, SeabedImageInput
from ..core.join import JoinBuffer, Released
from ..core.pipeline import Pipeline, PipelineConfig
from ..core.recordlog import RecordLog
from ..core.records import canonical_json
from ..core.stats import LatencyStats
from .convert import detections_to_msg, image_to_array

STATUS_SCHEMA = "blueboat_perception/status/1"

DEFAULT_TOPICS = {
    "topic_image": "/sss_ai/seabed_image",
    "topic_png": "/sss_ai/seabed_image/png",
    "topic_metadata": "/sss_ai/seabed_image/metadata",
    "topic_detections": "/sss_ai/detections",
    "topic_records": "/sss_ai/image_records",
    "topic_status": "/sss_ai/perception/status",
}

IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=32,
                       durability=DurabilityPolicy.VOLATILE)
RECORD_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST, depth=100,
                        durability=DurabilityPolicy.VOLATILE)
STATUS_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST, depth=1,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _utc_now() -> str:
    t = time.time()
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(t)) + f".{int((t % 1) * 1000):03d}Z"


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("perception")
        p = self._declare()
        self.mode = p["mode"]
        if self.mode not in ("normal", "dataset"):
            raise ValueError(f"mode must be normal|dataset, got {self.mode!r}")
        self.root = data_root(p["data_dir"])
        self.pipeline = Pipeline(PipelineConfig.from_params(
            preprocess=p["preprocess"], detector=p["detector"],
            calibration=p["calibration"], sigma_h_m=p["sigma_h_m"],
            sigma_heading_deg=p["sigma_heading_deg"], sigma_nav_m=p["sigma_nav_m"],
            footprint_row_step=p["footprint_row_step"],
            detector_input_mapping=p["detector_input_mapping"],
            near_range_mask_m=p["near_range_mask_m"], turn_window_s=p["turn_window_s"]))
        self.join = JoinBuffer(timeout_s=p["join_timeout_s"],
                               need_png=(self.mode == "dataset"))
        self.latency = {"join": LatencyStats(), "pipeline": LatencyStats(),
                        "publish": LatencyStats(), "total": LatencyStats()}
        self.records_published = 0
        self.input_errors = 0
        self.host = socket.gethostname()
        self.log = (RecordLog(perception_dir(self.root, "live"))
                    if p["record_log"] else None)
        self.dataset: Optional[DatasetWriter] = None
        #: dataset mode in a simulator world: the label source and its ground truth
        self.label_source = None
        self.ground_truth = None
        self.label_wait_s = float(p["label_wait_s"])
        self._pending_writes: deque = deque()
        if self.mode == "dataset":
            self.dataset = self._open_dataset(p)

        self.pub_det = self.create_publisher(Detection2DArray, p["topic_detections"], 10)
        self.pub_rec = self.create_publisher(String, p["topic_records"], RECORD_QOS)
        self.pub_status = self.create_publisher(String, p["topic_status"], STATUS_QOS)
        self.create_subscription(Image, p["topic_image"],
                                 lambda m: self._on_image(m, "image"), IMAGE_QOS)
        self.create_subscription(Image, p["topic_png"],
                                 lambda m: self._on_image(m, "png"), IMAGE_QOS)
        self.create_subscription(String, p["topic_metadata"], self._on_metadata, IMAGE_QOS)
        self.create_timer(0.5, self._poll)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f"perception {__version__}: mode={self.mode} detector={self.pipeline.detector.name}"
            f" (dummy={self.pipeline.detector.is_dummy}) data root={self.root}"
            + (f" dataset={self.dataset.dir}" if self.dataset else ""))
        self._publish_status()

    # ---- setup -------------------------------------------------------------------
    def _declare(self) -> dict:
        defaults = {
            "mode": "normal", "dataset_name": "", "dataset_path": "", "world_dir": "",
            "data_dir": "", "record_log": True, "join_timeout_s": 5.0,
            "detector": "dummy_center", "preprocess": "fixed_gray",
            "detector_input_mapping": "fixed_gray_v1", "near_range_mask_m": 0.0,
            "turn_window_s": 1.0,
            "label_source": "auto", "label_min_rows": 3, "label_wait_s": 3.0,
            "calibration": "identity", "sigma_h_m": 0.10,
            "sigma_heading_deg": 2.0, "sigma_nav_m": 1.0, "footprint_row_step": 16,
            **DEFAULT_TOPICS,
        }
        return {k: self.declare_parameter(k, v).value for k, v in defaults.items()}

    def _open_dataset(self, p: dict) -> DatasetWriter:
        cfg = {"pipeline": self.pipeline.config.to_json(), "node": {
            k: p[k] for k in ("mode", "dataset_name", "world_dir", "join_timeout_s")}}
        forbidden = [sss_data_dir(self.root)]
        labels = self._open_labels(p)
        try:
            if p["dataset_path"]:
                # Claimed by the launch file (which also points the
                # ground-truth recorder into it).
                d = Path(p["dataset_path"])
                return DatasetWriter(d, name=d.name, world_dir=p["world_dir"],
                                     config=cfg, forbidden_roots=forbidden,
                                     label_source=labels,
                                     detector_input=self.pipeline.detector_input_info())
            return DatasetWriter.create(self.root, p["dataset_name"],
                                        world_dir=p["world_dir"], config=cfg,
                                        forbidden_roots=forbidden, label_source=labels,
                                        detector_input=self.pipeline.detector_input_info())
        except DatasetError as exc:
            self.get_logger().error(f"dataset mode refused: {exc}")
            raise

    def _open_labels(self, p: dict):
        """Dataset mode only: the simulator ground-truth label source, or None
        (no world, ``label_source: none``, or a world without objects)."""
        if p["label_source"] == "none" or not p["world_dir"]:
            self.get_logger().info("dataset labels: none (no simulator world)"
                                   if not p["world_dir"] else "dataset labels: off")
            return None
        if p["label_source"] != "auto":
            raise ValueError(f"label_source must be auto|none, got {p['label_source']!r}")
        from ..core.sim_labels import GroundTruthBuffer, SimGroundTruthLabelSource, load_world
        from . import ground_truth
        try:
            world = load_world(p["world_dir"])
        except (OSError, ValueError, KeyError) as exc:
            self.get_logger().error(f"dataset labels: none -- {exc}")
            return None
        self.ground_truth = GroundTruthBuffer()
        self.label_source = SimGroundTruthLabelSource(world, self.ground_truth,
                                                      min_rows=p["label_min_rows"])
        ground_truth.subscribe(self, self.ground_truth)
        self.get_logger().info(
            f"dataset labels: simulator ground truth, {len(world.objects)} objects "
            f"({world.path}), {len(self.label_source.classes)} classes")
        return self.label_source

    # ---- input -------------------------------------------------------------------
    def _on_image(self, msg: Image, part: str) -> None:
        key = msg.header.frame_id
        try:
            arr = image_to_array(msg)
        except ValueError as exc:
            self.input_errors += 1
            self.get_logger().warning(f"{part} for {key!r} unreadable: {exc}")
            return
        self._add(part, key, arr)

    def _on_metadata(self, msg: String) -> None:
        try:
            env = json.loads(msg.data)
            key = env["key"]
        except (ValueError, KeyError, TypeError) as exc:
            self.input_errors += 1
            self.get_logger().warning(f"metadata unreadable: {exc}")
            return
        self._add("metadata", key, env)

    def _add(self, part: str, key: str, payload) -> None:
        now = time.monotonic()
        try:
            released = self.join.add(part, key, payload, now)
        except ValueError as exc:
            self.input_errors += 1
            self.get_logger().warning(f"bad image key {key!r}: {exc}")
            return
        for r in released:
            self._process(r)

    def _poll(self) -> None:
        for r in self.join.poll(time.monotonic()):
            self._process(r)
        self._drain_writes()
        if self.ground_truth is not None:
            self.ground_truth.prune()

    # ---- processing --------------------------------------------------------------
    def _process(self, rel: Released) -> None:
        t0 = time.monotonic()
        last = rel.last_seen
        if rel.status != "complete":
            self.get_logger().warning(f"image {rel.key} released partially: {rel.status}")
        try:
            img = SeabedImageInput.from_wire(rel.metadata, rel.parts.get("image"),
                                             rel.parts.get("png"))
        except (InputError, ValueError, KeyError) as exc:
            self.input_errors += 1
            self.get_logger().error(f"image {rel.key} rejected: {exc}")
            return
        record, dets, inp = self.pipeline.run_full(img)
        t1 = time.monotonic()
        record["source"] = "live"
        record["provenance"] = {
            "received_wall_utc": _utc_now(), "host": self.host,
            "join_status": rel.status,
            "stream_stats": self.join.streams[rel.stream_id].to_json(),
            "latency_ms": {"join": 1000.0 * (last - rel.first_seen),
                           "pipeline": 1000.0 * (t1 - t0)}}
        self.pub_det.publish(detections_to_msg(dets, record["time"]["t_max_s"] or 0.0))
        rec_json = canonical_json(record)
        self.pub_rec.publish(String(data=rec_json))
        t2 = time.monotonic()
        record["provenance"]["latency_ms"]["publish"] = 1000.0 * (t2 - t1)
        record["provenance"]["latency_ms"]["total"] = 1000.0 * (t2 - last)
        for k in ("join", "pipeline", "publish", "total"):
            self.latency[k].add(record["provenance"]["latency_ms"][k])
        self.records_published += 1
        if self.log is not None:
            self.log.write(record)
        if self.dataset is not None:
            self._dataset_write(img, record, inp)

    # ---- dataset -----------------------------------------------------------------
    def _dataset_write(self, img, record, inp) -> None:
        """Write now, or queue (in order) until the picture's ground truth has
        arrived, at most ``label_wait_s``."""
        if self.label_source is not None and (self._pending_writes
                                              or not self.label_source.ready(img)):
            self._pending_writes.append((time.monotonic() + self.label_wait_s,
                                         img, record, inp))
            self._drain_writes()
            return
        self._write_sample(img, record, inp)

    def _drain_writes(self, force: bool = False) -> None:
        now = time.monotonic()
        while self._pending_writes:
            deadline, img, record, inp = self._pending_writes[0]
            if not (force or now >= deadline or self.label_source.ready(img)):
                break
            self._pending_writes.popleft()
            if not self.label_source.ready(img):
                self.get_logger().warning(
                    f"dataset {img.key}: ground truth incomplete, labelled with what arrived")
            self._write_sample(img, record, inp)

    def _write_sample(self, img, record, inp) -> None:
        try:
            self.dataset.write(img, record, inp)
        except (OSError, DatasetError) as exc:
            self.get_logger().error(f"dataset write failed for {img.key}: {exc}")

    def _publish_status(self) -> None:
        status = {
            "schema": STATUS_SCHEMA, "perception_version": __version__,
            "mode": self.mode,
            "dataset": (None if self.dataset is None else
                        {"name": self.dataset.name, "dir": str(self.dataset.dir),
                         "images": self.dataset.count,
                         "skipped_no_png": self.dataset.skipped_no_png,
                         "label_source": self.dataset.labels.name,
                         "labels": (self.label_source.stats()
                                    if self.label_source is not None else None),
                         "pending_label_writes": len(self._pending_writes),
                         "ground_truth": (self.ground_truth.stats()
                                          if self.ground_truth is not None else None)}),
            "detector": {"name": self.pipeline.detector.name,
                         "dummy": bool(self.pipeline.detector.is_dummy)},
            "streams": self.join.stats_json(),
            "pending": self.join.pending,
            "records_published": self.records_published,
            "input_errors": self.input_errors,
            "latency_ms": {k: v.summary() for k, v in self.latency.items()},
            "record_log": ({sid: str(p) for sid, p in self.log.paths.items()}
                           if self.log else None),
            "wall_utc": _utc_now(),
        }
        self.pub_status.publish(String(data=json.dumps(status, sort_keys=True)))

    def shutdown(self) -> None:
        for r in self.join.flush(time.monotonic()):
            self._process(r)
        if self.log is not None:
            self.log.close()
        if self.dataset is not None:
            self._drain_writes(force=True)
            self.dataset.close()


def main(argv=None) -> int:
    rclpy.init(args=argv if argv is not None else sys.argv)
    node = None
    try:
        node = PerceptionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A launch teardown can deliver SIGINT twice (the process group,
        # then the launch's own forward): the second must not interrupt
        # the flush of the record log and the dataset manifest.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            try:
                node.shutdown()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
