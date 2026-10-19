"""blueboat-perception-offline -- the live pipeline over a recorded session.

    blueboat-perception-offline SESSION [--out DIR] [--data-dir ROOT]
                                [--dataset NAME [--world DIR --ground-truth-bag BAG]]
                                [--compare LIVE.jsonl ...]

SESSION is a GCS recording session, a merged session, a "Save pictures"
folder or a ``seabed_images/`` folder. The same ``core.Pipeline`` the node
runs is applied to every saved image, in id order. Outputs, in a new
folder (default ``<root>/data/Perception_data/offline/<session>/``,
``-2`` ... on a clash):

* ``records.jsonl`` -- one canonical record per image (``source: offline``);
* ``detections.json`` -- every detection, flat;
* ``summary.json`` -- counts, configuration, input.

``--dataset NAME`` also writes a dataset (``<root>/data/Dataset_data/NAME``)
from the session's images. With ``--world`` (the simulator world folder) and
``--ground-truth-bag`` (a dataset-mode run's ``ground_truth_bag/``), its labels
are made from the simulator's ground truth exactly as the live node makes them
(``core/sim_labels.py``); rosbag2_py is needed for that option only.
``--compare`` checks the records against a live
record log (``Perception_data/live/<stream_id>.jsonl``) -- equal means
byte-equal canonical JSON once ``provenance`` and ``source`` are removed.
The session is opened read-only, and no output may sit inside it (CM-7).
No ROS needed.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

from .. import __version__
from ..core.dataroot import claim_dir, data_root, is_inside, perception_dir, safe_name
from ..core.dataset import DatasetWriter
from ..core.pipeline import Pipeline, PipelineConfig
from ..core.records import canonical_json, comparable_json, read_jsonl
from ..core.session import images_dir, iter_session, session_name


def read_ground_truth_bag(bag: Path):
    """Every message of a dataset's archival ground-truth bag, as the dicts the
    simulator published. ROS (rosbag2_py, rclpy, std_msgs) is imported here
    only, so the rest of the CLI runs without it."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    while reader.has_next():
        _topic, data, _t = reader.read_next()
        yield json.loads(deserialize_message(data, String).data)


def ground_truth_labels(world_dir: Path, bag: Path, min_rows: int = 3):
    """The live node's label source, fed from the recorded bag."""
    from ..core.sim_labels import GroundTruthBuffer, SimGroundTruthLabelSource, load_world
    buffer = GroundTruthBuffer(horizon_s=None)
    for doc in read_ground_truth_bag(bag):
        if "sensors" in doc:
            buffer.add_sensor(doc)
        elif "contacts" in doc:
            buffer.add_contacts(doc)
    return SimGroundTruthLabelSource(load_world(str(world_dir)), buffer, min_rows=min_rows)


def run_session(session: Path, pipeline: Pipeline,
                dataset: Optional[DatasetWriter] = None) -> List[dict]:
    records = []
    # the baseline detector input IS the display PNG, so read it then
    load_png = pipeline.preprocessor.name == "identity"
    for img in iter_session(session, load_png=load_png):
        record, _dets, inp = pipeline.run_full(img)
        record["source"] = "offline"
        record["provenance"] = {"session": str(images_dir(session)),
                                "metadata_path": img.metadata_path}
        records.append(record)
        if dataset is not None:
            dataset.write(img, record, inp)
    return records


def compare(records: List[dict], live_paths: List[Path]) -> Dict[str, object]:
    live: Dict[str, str] = {}
    for p in live_paths:
        for rec in read_jsonl(p):
            live[rec["key"]] = comparable_json(rec)
    off = {r["key"]: comparable_json(r) for r in records}
    both = sorted(set(off) & set(live))
    diff = [k for k in both if off[k] != live[k]]
    return {"compared": len(both), "identical": len(both) - len(diff),
            "different": diff,
            "only_offline": sorted(set(off) - set(live)),
            "only_live": sorted(set(live) - set(off))}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="blueboat-perception-offline",
                                 description=__doc__.splitlines()[0])
    ap.add_argument("session", type=Path)
    ap.add_argument("--out", type=Path, default=None,
                    help="output folder (default <root>/data/Perception_data/offline/<session>)")
    ap.add_argument("--data-dir", default="", help="data root override (as the node's data_dir)")
    ap.add_argument("--dataset", default="", help="also write a dataset of this name")
    ap.add_argument("--world", type=Path, default=None,
                    help="simulator world folder (labels from ground truth; with "
                         "--ground-truth-bag)")
    ap.add_argument("--ground-truth-bag", type=Path, default=None,
                    help="a dataset-mode run's ground_truth_bag/ (with --world)")
    ap.add_argument("--label-min-rows", type=int, default=3)
    ap.add_argument("--compare", type=Path, nargs="*", default=[],
                    help="live record log(s) to compare against")
    ap.add_argument("--detector", default="dummy_center")
    ap.add_argument("--preprocess", default="fixed_gray",
                    help="detector input: fixed_gray (default) or identity (the display PNG)")
    ap.add_argument("--detector-input-mapping", default="fixed_gray_v1")
    ap.add_argument("--near-range-mask-m", type=float, default=0.0,
                    help="mask the detector input below this slant range (0 = off)")
    ap.add_argument("--turn-window-s", type=float, default=1.0)
    ap.add_argument("--calibration", default="identity")
    ap.add_argument("--footprint-row-step", type=int, default=16)
    args = ap.parse_args(argv)

    session = args.session.expanduser().resolve()
    src = images_dir(session)
    tree = src.parent if src.name == "seabed_images" else src
    root = data_root(args.data_dir)
    if args.out is not None:
        out = args.out.expanduser().resolve()
        if is_inside(out, tree):
            print(f"refusing --out {out}: inside the session {tree} (CM-7)", file=sys.stderr)
            return 2
        out.mkdir(parents=True, exist_ok=False)
    else:
        out = claim_dir(perception_dir(root, "offline"), safe_name(session_name(session)))
    if is_inside(out, tree):
        print(f"refusing output {out}: inside the session {tree} (CM-7)", file=sys.stderr)
        return 2

    pipeline = Pipeline(PipelineConfig(preprocess=args.preprocess, detector=args.detector,
                                       calibration=args.calibration,
                                       footprint_row_step=args.footprint_row_step,
                                       detector_input_mapping=args.detector_input_mapping,
                                       near_range_mask_m=args.near_range_mask_m,
                                       turn_window_s=args.turn_window_s))
    if (args.world is None) != (args.ground_truth_bag is None) or \
            (args.world is not None and not args.dataset):
        print("--world and --ground-truth-bag go together, with --dataset", file=sys.stderr)
        return 2
    labels = None
    if args.world is not None:
        labels = ground_truth_labels(args.world.expanduser().resolve(),
                                     args.ground_truth_bag.expanduser().resolve(),
                                     args.label_min_rows)
    dataset = None
    if args.dataset:
        dataset = DatasetWriter.create(root, args.dataset, mode="offline",
                                       world_dir=str(args.world or ""),
                                       config={"pipeline": pipeline.config.to_json(),
                                               "session": str(src),
                                               "ground_truth_bag": (
                                                   str(args.ground_truth_bag)
                                                   if args.ground_truth_bag else None)},
                                       forbidden_roots=[tree], label_source=labels,
                                       detector_input=pipeline.detector_input_info())
    t0 = time.monotonic()
    records = run_session(session, pipeline, dataset)
    elapsed = time.monotonic() - t0
    with open(out / "records.jsonl", "x", encoding="utf-8") as f:
        for r in records:
            f.write(canonical_json(r) + "\n")
    dets = [d for r in records for d in (r["detections"] or [])]
    with open(out / "detections.json", "x", encoding="utf-8") as f:
        f.write(canonical_json(dets) + "\n")
    statuses: Dict[str, int] = {}
    for r in records:
        s = r["processing"]["detector"]["status"]
        statuses[s] = statuses.get(s, 0) + 1
    summary = {"schema": "blueboat_perception/offline_summary/1",
               "perception_version": __version__, "session": str(src),
               "images": len(records), "detections": len(dets), "status": statuses,
               "streams": sorted({r["stream_id"] for r in records}),
               "pipeline": pipeline.config.to_json(), "elapsed_s": elapsed,
               "dataset": None if dataset is None else str(dataset.dir)}
    if args.compare:
        summary["compare"] = compare(records, list(args.compare))
    with open(out / "summary.json", "x", encoding="utf-8") as f:
        json.dump(summary, f, indent=1, sort_keys=True)
    if dataset is not None:
        dataset.close()
    print(f"{len(records)} image(s), {len(dets)} detection(s) -> {out}")
    if args.compare:
        c = summary["compare"]
        print(f"compare: {c['identical']}/{c['compared']} identical, "
              f"{len(c['different'])} different, {len(c['only_offline'])} offline-only, "
              f"{len(c['only_live'])} live-only")
        return 0 if not c["different"] else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
