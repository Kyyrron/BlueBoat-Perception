#!/usr/bin/env python3
"""Evaluate a detector configuration on whole pictures of held-out simulated
worlds (not installed; offline only).

    python3 tools/evaluate_detector.py --name NAME --split split.yaml --on val \\
        --detector yolo --detector-model <weights> [--detector-tiling tiles|full] \\
        [--tile-stride 128] [--merge nms|greedy_nmm] [--merge-threshold 0.5] \\
        DATASET [DATASET ...]

The detections come from the **real pipeline** (``core.Pipeline`` with the
YOLO adapter: tiling, merging, the 999 cap), run on each picture rebuilt from
the dataset (its sidecar's GCS metadata + its raw dB), so what is measured is
what the node publishes. The ground truth is the dataset's **full-picture**
labels; tile-edge label rules play no part here.

- ``--on val``: the validation worlds -- where thresholds and the merge rule
  are chosen. ``--on test``: the held-out worlds, **report only, never tune**
  (the results say which).
- Faint boxes (``label_visibility.py``, the tile writer's rule) are *ignore*
  regions: a detection matching one is neither a hit nor a false alarm, and
  missing one is not a miss.
- Metrics (``results.json``):
  - AP50 and AP50:95, COCO-style (101-point interpolated precision, IoU
    0.50:0.05:0.95, greedy matching by descending score, per class, then the
    mean over the classes present), overall and per class;
  - recall per slant-range band of the box centre (``--range-bands``) and per
    class, at ``--operating-threshold`` (0.10 = ``map_min_confidence``) and
    IoU 0.5;
  - false detections per picture at that threshold;
  - latency per picture (pipeline only; p50 / p95 / max) on ``--detector-device``.
- Output: ``<root>/data/Perception_eval/<NAME>/`` (claimed exclusively):
  ``results.json`` (configuration, model card and SHA-256, datasets + manifest
  SHA-256, split, versions, code version, metrics) and ``per_picture.jsonl``.

Every result is a result **on simulated data**, model-conditional (CM-13).
"""

from __future__ import annotations

import argparse
import json
import platform
import sys
import time
from pathlib import Path
from typing import Dict, List, Sequence

import numpy as np

TOOLS = Path(__file__).resolve().parent
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from make_tile_dataset import (DatasetRefused, check_box_class, check_classes,  # noqa: E402
                               code_version, load_split, picture, split_of)
from label_visibility import DEFAULT_MIN_SHADOW_M, DEFAULT_THRESHOLD_DB  # noqa: E402
from label_visibility import box_contrast, classify  # noqa: E402

from blueboat_perception.core.dataroot import claim_dir, data_root, safe_name  # noqa: E402
from blueboat_perception.core.detectors.yolo import PARAM_DEFAULTS, detector_args  # noqa: E402
from blueboat_perception.core.pipeline import Pipeline, PipelineConfig  # noqa: E402
from blueboat_perception.core.sim_labels import load_class_map  # noqa: E402

RESULTS_SCHEMA = "blueboat_perception/evaluation/1"
IOUS = np.round(np.arange(0.50, 0.951, 0.05), 2)
STATEMENT = ("Results on simulated data (held-out simulated worlds), model-conditional: "
             "in a calibrated simulation environment, not an empirical field result (CM-13).")


def iou_matrix(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """IoU of continuous boxes [x0, y0, x1, y1], (n, m)."""
    if len(a) == 0 or len(b) == 0:
        return np.zeros((len(a), len(b)))
    ix0 = np.maximum(a[:, None, 0], b[None, :, 0])
    iy0 = np.maximum(a[:, None, 1], b[None, :, 1])
    ix1 = np.minimum(a[:, None, 2], b[None, :, 2])
    iy1 = np.minimum(a[:, None, 3], b[None, :, 3])
    inter = np.clip(ix1 - ix0, 0, None) * np.clip(iy1 - iy0, 0, None)
    area = lambda x: (x[:, 2] - x[:, 0]) * (x[:, 3] - x[:, 1])  # noqa: E731
    return inter / (area(a)[:, None] + area(b)[None, :] - inter)


def px_box(r0, c0, r1, c1) -> List[float]:
    """Inclusive pixel box -> continuous [x0, y0, x1, y1]."""
    return [float(c0), float(r0), float(c1 + 1), float(r1 + 1)]


def match_picture(dets: List[dict], gts: List[dict], iou_t: float):
    """Greedy COCO matching of one picture and class at one IoU threshold.
    Returns, per detection (in input order, sorted by score), 1 = hit,
    0 = false alarm, -1 = ignored; and the matched flag per GT."""
    order = sorted(range(len(dets)), key=lambda i: -dets[i]["score"])
    real = [g for g in gts if not g["ignore"]]
    ign = [g for g in gts if g["ignore"]]
    db = np.array([dets[i]["box"] for i in order]).reshape(-1, 4)
    m_real = iou_matrix(db, np.array([g["box"] for g in real]).reshape(-1, 4))
    m_ign = iou_matrix(db, np.array([g["box"] for g in ign]).reshape(-1, 4))
    used = np.zeros(len(real), bool)
    out = np.zeros(len(dets), int)
    for k, i in enumerate(order):
        best, bj = iou_t, -1
        for j in range(len(real)):
            if not used[j] and m_real[k, j] >= best:
                best, bj = m_real[k, j], j
        if bj >= 0:
            used[bj] = True
            out[i] = 1
        elif len(ign) and m_ign[k].max() >= iou_t:
            out[i] = -1
        else:
            out[i] = 0
    return out, used, real


def average_precision(scores: Sequence[float], hits: Sequence[int], n_gt: int) -> float:
    """101-point interpolated AP (COCO); ``hits`` 1/0 (ignored removed)."""
    if n_gt == 0:
        return float("nan")
    if not len(scores):
        return 0.0
    o = np.argsort(-np.asarray(scores), kind="stable")
    h = np.asarray(hits)[o]
    tp, fp = np.cumsum(h == 1), np.cumsum(h == 0)
    rec = tp / n_gt
    prec = tp / np.maximum(tp + fp, 1e-12)
    prec = np.maximum.accumulate(prec[::-1])[::-1]
    out = 0.0
    for r in np.linspace(0, 1, 101):
        idx = np.searchsorted(rec, r, side="left")
        out += prec[idx] if idx < len(prec) else 0.0
    return out / 101.0


def evaluate(pictures: List[dict], classes: List[str], bands: List[float], thr: float) -> dict:
    present = sorted({g["class_id"] for p in pictures for g in p["gts"] if not g["ignore"]})
    per_class = {}
    for c in present:
        aps = []
        for t in IOUS:
            scores, hits, n_gt = [], [], 0
            for p in pictures:
                d = [x for x in p["dets"] if x["class_id"] == c]
                g = [x for x in p["gts"] if x["class_id"] == c]
                out, _used, real = match_picture(d, g, float(t))
                n_gt += len(real)
                for x, o in zip(d, out):
                    if o >= 0:
                        scores.append(x["score"])
                        hits.append(int(o))
            aps.append(average_precision(scores, hits, n_gt))
        per_class[classes[c] if c < len(classes) else str(c)] = {
            "ap50": aps[0], "ap50_95": float(np.mean(aps)),
            "n_gt": sum(1 for p in pictures for g in p["gts"]
                        if g["class_id"] == c and not g["ignore"])}
    # operating point: IoU 0.5, confidence >= thr
    edges = list(bands)
    band_hit = np.zeros(len(edges) - 1, int)
    band_n = np.zeros(len(edges) - 1, int)
    cls_hit: Dict[str, List[int]] = {}
    false = []
    for p in pictures:
        fa = 0
        for c in sorted({x["class_id"] for x in p["dets"]} | {g["class_id"] for g in p["gts"]}):
            d = [x for x in p["dets"] if x["class_id"] == c and x["score"] >= thr]
            g = [x for x in p["gts"] if x["class_id"] == c]
            out, used, real = match_picture(d, g, 0.5)
            fa += int(np.count_nonzero(out == 0))
            for gt, u in zip(real, used):
                b = np.searchsorted(edges, gt["slant_m"], side="right") - 1
                if 0 <= b < len(band_n):
                    band_n[b] += 1
                    band_hit[b] += int(u)
                name = classes[c] if c < len(classes) else str(c)
                cls_hit.setdefault(name, [0, 0])
                cls_hit[name][0] += int(u)
                cls_hit[name][1] += 1
        false.append(fa)
    lat = np.array([p["latency_ms"] for p in pictures]) if pictures else np.zeros(0)
    return {
        "ap50": float(np.nanmean([v["ap50"] for v in per_class.values()])) if per_class else None,
        "ap50_95": (float(np.nanmean([v["ap50_95"] for v in per_class.values()]))
                    if per_class else None),
        "per_class": per_class,
        "operating_point": {
            "confidence": thr, "iou": 0.5,
            "recall_by_slant_range_m": [
                {"band_m": [edges[i], edges[i + 1]], "n_gt": int(band_n[i]),
                 "recall": (float(band_hit[i] / band_n[i]) if band_n[i] else None)}
                for i in range(len(band_n))],
            "recall_by_class": {k: {"n_gt": v[1], "recall": v[0] / v[1] if v[1] else None}
                                for k, v in sorted(cls_hit.items())},
            "false_detections_per_picture": {
                "mean": float(np.mean(false)) if false else None,
                "max": int(max(false)) if false else None}},
        "latency_ms_per_picture": ({"n": int(lat.size), "p50": float(np.percentile(lat, 50)),
                                    "p95": float(np.percentile(lat, 95)),
                                    "max": float(lat.max())} if lat.size else None),
        "pictures": len(pictures),
        "gt_boxes": sum(1 for p in pictures for g in p["gts"] if not g["ignore"]),
        "ignored_boxes": sum(1 for p in pictures for g in p["gts"] if g["ignore"]),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("datasets", nargs="+", type=Path)
    ap.add_argument("--split", type=Path, required=True)
    ap.add_argument("--on", choices=("val", "test"), required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--data-dir", default="")
    ap.add_argument("--detector", default="yolo", help="yolo | dummy_center")
    for name, default in PARAM_DEFAULTS.items():
        ap.add_argument("--" + name.replace("_", "-"), dest=name, default=default,
                        type=type(default))
    ap.add_argument("--operating-threshold", type=float, default=0.10)
    ap.add_argument("--range-bands", default="0,5,10,15,20,30")
    ap.add_argument("--visibility-db", type=float, default=DEFAULT_THRESHOLD_DB)
    ap.add_argument("--limit", type=int, default=0, help="pictures per dataset (0 = all)")
    a = ap.parse_args(argv)

    classes = load_class_map()
    split = load_split(a.split)
    sources = []
    for ds in a.datasets:
        ds = ds.expanduser().resolve()
        mtext = (ds / "manifest.json").read_text(encoding="utf-8")
        man = json.loads(mtext)
        s, key = split_of(ds, man, split)
        if s != a.on:
            continue
        check_classes(ds, man, classes, unlabelled=False)
        import hashlib
        sources.append((ds, man, key, hashlib.sha256(mtext.encode()).hexdigest()))
    if not sources:
        print(f"no dataset in split {a.on!r}", file=sys.stderr)
        return 2
    det_args = detector_args(a.detector, vars(a))
    pipeline = Pipeline(PipelineConfig(detector=a.detector, detector_args=det_args))
    pictures, per_pic = [], []
    for ds, man, key, _sha in sources:
        stems = sorted(p.stem for p in (ds / "images").glob("*.json"))
        for stem in stems[:a.limit or None]:
            img, _gray, _valid, sidecar = picture(ds, stem, man)
            boxes = ((sidecar.get("perception") or {}).get("labels") or {}).get("boxes") or []
            for b in boxes:
                check_box_class(ds, stem, b, classes)
            t0 = time.perf_counter()
            rec, dets = pipeline.run(img)
            ms = 1000.0 * (time.perf_counter() - t0)
            gts = []
            for b in boxes:
                p = b["box_px"]
                v = classify(box_contrast(img.db, boxes, b), float(b.get("shadow_m") or 0.0),
                             a.visibility_db, DEFAULT_MIN_SHADOW_M)
                gts.append({"box": px_box(p["r0"], p["c0"], p["r1"], p["c1"]),
                            "class_id": int(b["class_id"]), "ignore": v == "faint",
                            "slant_m": float(b.get("slant_m") or 0.0)})
            d = [{"box": px_box(*x["bbox_px"]), "class_id": int(x["class_id"]),
                  "score": float(x["confidence_raw"])} for x in (dets or [])]
            pictures.append({"dets": d, "gts": gts, "latency_ms": ms})
            per_pic.append({"dataset": ds.name, "world": key, "stem": stem, "key": rec["key"],
                            "latency_ms": ms, "detections": len(d), "gt": len(gts),
                            "detector_tiles": rec["summary"].get("detector_tiles"),
                            "status": rec["processing"]["detector"]["status"]})
    bands = [float(x) for x in a.range_bands.split(",")]
    metrics = evaluate(pictures, classes, bands, a.operating_threshold)
    out = claim_dir(data_root(a.data_dir) / "data" / "Perception_eval", safe_name(a.name))
    vers = {"python": platform.python_version()}
    for mod in ("ultralytics", "torch", "numpy"):
        try:
            vers[mod] = __import__(mod).__version__
        except ImportError:
            vers[mod] = None
    results = {"schema": RESULTS_SCHEMA, "name": out.name, "statement": STATEMENT,
               "split_evaluated": a.on,
               "held_out_report_only": a.on == "test",
               "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
               "detector": pipeline.detector.name,
               "detector_params": getattr(pipeline.detector, "params", None),
               "pipeline": pipeline.config.to_json(),
               "datasets": [{"dir": str(ds), "world": key, "manifest_sha256": sha}
                            for ds, _m, key, sha in sources],
               "split_file": str(a.split), "visibility_db": a.visibility_db,
               "versions": vers, "code": code_version(), "metrics": metrics,
               "config": {k: str(v) if isinstance(v, Path) else
                          ([str(x) for x in v] if isinstance(v, list) else v)
                          for k, v in vars(a).items()}}
    with open(out / "results.json", "x", encoding="utf-8") as f:
        json.dump(results, f, indent=1, sort_keys=True)
    with open(out / "per_picture.jsonl", "x", encoding="utf-8") as f:
        for r in per_pic:
            f.write(json.dumps(r, sort_keys=True) + "\n")
    print(f"{STATEMENT}\n{a.on}: {len(pictures)} pictures, AP50 {metrics['ap50']}, "
          f"AP50:95 {metrics['ap50_95']} -> {out}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except DatasetRefused as exc:
        print(f"refused: {exc}", file=sys.stderr)
        sys.exit(2)
