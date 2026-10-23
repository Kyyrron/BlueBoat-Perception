#!/usr/bin/env python3
"""Train a YOLO detector (YOLO11s / YOLO26s, Ultralytics) on a tile dataset
(not installed; offline only; needs ``ultralytics`` and ``torch``).

    python3 tools/train_yolo.py --data <root>/data/Tile_data/<tiles>/data.yaml \\
        --model yolo11s.pt --name yolo11s-tiles128-s0 [--seed 0] [--epochs 150] \\
        [--batch 32] [--device cpu|0] [--imgsz 256]

``--model`` is the pretrained checkpoint (``yolo11s.pt`` or ``yolo26s.pt``;
Ultralytics downloads it on first use). ``--data`` is ``data.yaml`` (grey
copied into 3 channels) or ``data_gray1.yaml`` (single channel; Ultralytics
8.4 fills the first convolution from the summed pretrained filters). The
squeeze baseline trains on a ``--mode full`` dataset with ``--imgsz 640``.

Output, claimed exclusively (``-2`` ... on a clash):
``<root>/data/Perception_models/<name>/``
    weights/best.pt, weights/last.pt, args.yaml, results.csv  (Ultralytics)
    model.json   what the detector records and what reproduces the run:
                 family, pretrained checkpoint, dataset (+ manifest SHA-256,
                 world split), seed, every training argument, augmentation,
                 library and code versions, best.pt SHA-256, tile-level
                 validation metrics. "Simulated data" is stated.

**Augmentation, set explicitly** (Ultralytics' defaults suit photographs,
not sonar; ``ultralytics/cfg/default.yaml``):

- ``hsv_h = hsv_s = 0`` (grey input: no hue / saturation), ``hsv_v = 0.2``
  (mild brightness only), ``bgr = 0``;
- ``degrees = shear = perspective = 0``: a rotated or sheared tile mixes
  range and along-track, which no ping can produce;
- ``mosaic = 0`` (and ``close_mosaic = 0``), ``mixup = cutmix = copy_paste =
  0``: they paste pieces of other tiles, breaking the range <-> pixel
  relation and the nadir geometry;
- ``scale = 0``: a pixel is one slant bin (``bin_pitch_m``) and objects keep
  their physical size in pixels; ``translate = 0``: a shift pads with grey
  that no sonar picture contains;
- ``fliplr = flipud = 0.5``: **whole-tile** flips only (Ultralytics flips the
  whole image and its labels). A left-right flip mirrors the nadir with the
  object, so the shadow stays on the side away from the nadir: a mirrored
  port tile is a physically consistent starboard tile. An up-down flip
  reverses the along-track order of the rows, also a picture a boat can
  record (sailing the other way). **Limitation:** an object whose echo is
  asymmetric (an L-shaped or tilted object, a lattice) or depends on aspect
  appears mirrored, i.e. at another aspect; flips must be switched off
  (``--no-flips``) for the aspect / heading-response experiment, where
  aspect is the variable measured.

Seeded (``--seed``, ``deterministic=True``). The held-out test worlds are
never read here: Ultralytics trains on ``train`` and selects ``best.pt`` on
``val`` only.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import subprocess
import sys
import time
from pathlib import Path

PKG = Path(__file__).resolve().parents[1] / "blueboat_perception"
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from blueboat_perception import __version__  # noqa: E402
from blueboat_perception.core.dataroot import claim_dir, data_root, safe_name  # noqa: E402

AUGMENTATION = {"hsv_h": 0.0, "hsv_s": 0.0, "hsv_v": 0.2, "degrees": 0.0, "translate": 0.0,
                "scale": 0.0, "shear": 0.0, "perspective": 0.0, "flipud": 0.5,
                "fliplr": 0.5, "bgr": 0.0, "mosaic": 0.0, "close_mosaic": 0,
                "mixup": 0.0, "cutmix": 0.0, "copy_paste": 0.0}
MODEL_SCHEMA = "blueboat_perception/model/1"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def code_version() -> dict:
    root = Path(__file__).resolve().parents[1]
    try:
        sha = subprocess.run(["git", "-C", str(root), "rev-parse", "HEAD"],
                             capture_output=True, text=True, timeout=10).stdout.strip()
        dirty = bool(subprocess.run(["git", "-C", str(root), "status", "--porcelain"],
                                    capture_output=True, text=True, timeout=10).stdout.strip())
    except (OSError, subprocess.SubprocessError):
        sha, dirty = "", None
    return {"perception_version": __version__, "git_commit": sha or None, "dirty": dirty}


def versions() -> dict:
    out = {"python": platform.python_version(), "platform": platform.platform()}
    for mod in ("ultralytics", "torch", "torchvision", "numpy"):
        try:
            out[mod] = __import__(mod).__version__
        except ImportError:
            out[mod] = None
    try:
        import torch
        out["cuda"] = torch.version.cuda if torch.cuda.is_available() else None
        out["device_name"] = (torch.cuda.get_device_name(0) if torch.cuda.is_available()
                              else platform.processor() or None)
    except ImportError:
        pass
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--data", type=Path, required=True, help="tile dataset data.yaml")
    ap.add_argument("--model", required=True, help="yolo11s.pt | yolo26s.pt (pretrained)")
    ap.add_argument("--name", required=True)
    ap.add_argument("--data-dir", default="", help="data root (as the node's data_dir)")
    ap.add_argument("--imgsz", type=int, default=256)
    ap.add_argument("--epochs", type=int, default=150)
    ap.add_argument("--patience", type=int, default=50)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--device", default="cpu", help="cpu | 0 (first GPU)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-flips", action="store_true",
                    help="no flips (the aspect / heading-response experiment)")
    a = ap.parse_args(argv)

    data = a.data.expanduser().resolve()
    tile_manifest = data.parent / "manifest.json"
    if not tile_manifest.is_file():
        print(f"{data}: not a tile dataset (no manifest.json beside it)", file=sys.stderr)
        return 2
    mtext = tile_manifest.read_text(encoding="utf-8")
    man = json.loads(mtext)
    if "train" not in man.get("splits", {}) or "val" not in man.get("splits", {}):
        print(f"{data}: needs a train and a val split", file=sys.stderr)
        return 2

    from ultralytics import YOLO          # lazy: optional dependency

    out = claim_dir(data_root(a.data_dir) / "data" / "Perception_models", safe_name(a.name))
    aug = dict(AUGMENTATION)
    if a.no_flips:
        aug.update(fliplr=0.0, flipud=0.0)
    train_args = {"data": str(data), "imgsz": a.imgsz, "epochs": a.epochs,
                  "patience": a.patience, "batch": a.batch, "workers": a.workers,
                  "device": a.device, "seed": a.seed, "deterministic": True,
                  "project": str(out.parent), "name": out.name, "exist_ok": True,
                  "pretrained": True, "plots": True, "val": True, **aug}
    started = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    model = YOLO(a.model)
    results = model.train(**train_args)
    best = out / "weights" / "best.pt"
    metrics = {}
    for k, v in (getattr(results, "results_dict", None) or {}).items():
        try:
            metrics[k] = float(v)
        except (TypeError, ValueError):
            pass
    card = {"schema": MODEL_SCHEMA, "name": out.name,
            "family": Path(a.model).stem, "model": a.model,
            "created_utc": started, "finished_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                                                   time.gmtime()),
            "data": "simulated" if man.get("data") == "simulated" else man.get("data"),
            "dataset": {"data_yaml": str(data), "name": man.get("name"),
                        "manifest_sha256": hashlib.sha256(mtext.encode()).hexdigest(),
                        "mode": man.get("mode"), "tiling": man.get("tiling"),
                        "splits": man.get("splits"), "rules": man.get("rules"),
                        "channels": 1 if "channels: 1" in data.read_text() else 3},
            "seed": a.seed, "train_args": train_args, "augmentation": aug,
            "versions": versions(), "code": code_version(),
            "weights": {"best": "weights/best.pt",
                        "best_sha256": sha256(best) if best.is_file() else None},
            "tile_val_metrics": metrics,
            "note": "tile-level validation metrics from Ultralytics (val worlds); the "
                    "evaluation is tools/evaluate_detector.py on whole pictures"}
    (out / "model.json").write_text(json.dumps(card, indent=1, sort_keys=True),
                                    encoding="utf-8")
    print(f"model -> {out}  (detector_model: {best})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
