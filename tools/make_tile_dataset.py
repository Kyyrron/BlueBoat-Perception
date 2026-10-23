#!/usr/bin/env python3
"""Turn Perception datasets into an Ultralytics tile dataset, split by world
(not installed; offline only).

    python3 tools/make_tile_dataset.py --split split.yaml --name NAME \\
        DATASET [DATASET ...] [--mode tiles|full] [--stride 128] [--seed 0]

``DATASET`` = a dataset-mode folder (``<root>/data/Dataset_data/<name>/``).
``split.yaml`` assigns every dataset to ``train`` / ``val`` / ``test`` by its
**world** (the folder name of the manifest's ``world_dir``) or by its own
folder name; consecutive pictures overlap 50 %, so a split is by world (or
session), never by image (Sethuraman et al. 2024 §4, R12)::

    train: [world_a, world_b, world_c]
    val:   [world_d]
    test:  [world_e]          # held out: never used to tune anything

Output: ``<root>/data/Tile_data/<NAME>/`` (claimed exclusively, ``-2`` ... on a
clash; nothing is ever overwritten, no input is modified):

    data.yaml            Ultralytics: images/{train,val,test}, nc 29, names = yolo_classes_v1
    data_gray1.yaml      the same with ``channels: 1`` (the single-channel arm)
    images/<split>/<unit>.png   the detector input (``images/`` of the source), tiled
    images/<split>/<unit>.json  tile sidecar (blueboat_perception/tile_sidecar/1)
    labels/<split>/<unit>.txt   YOLO labels in tile pixels (empty = background)
    manifest.json        configuration, split, seed, sources (+ manifest SHA-256),
                         code version, counts and every drop

**The same function as inference.** Tiles are cut by ``core/tiling.py`` from the
dataset's own training image (the ``fixed_gray`` detector input), with
``valid`` rebuilt from the raw dB and the dataset's near-range mask; for a
``fixed_gray`` dataset the stored picture is re-mapped from the raw dB and must
equal it bit for bit (else the dataset was made with another mapping: refused).

**Label rules (owner's decisions 2026-10-23).**

- *Visibility* (``label_visibility.py``): a box is kept if its highlight is
  >= 2 dB above the background or its shadow <= -2 dB below; a ``faint`` box
  is dropped, and a unit (tile, or picture in ``--mode full``) holding >= 50 %
  of a faint box is left out, so the model is never told "faint object =
  background". Provisional threshold (``--visibility-db``).
- *Tile edges, rule B + (i)*: a box cut by a tile edge is kept, clipped to the
  tile, when >= 50 % of its **highlight** columns are inside and the clipped
  box is >= 4 px wide; otherwise its label is dropped and the tile kept. Any
  highlight up to 129 px wide lies whole inside some tile (stride 128), so no
  object is lost. A label dropped while the image keeps the cut object is
  also what Ultralytics' own crops do (``RandomPerspective.box_candidates``:
  boxes keeping < 10 % of their area are dropped, the image kept).
- *Background*: ``--empty-fraction`` (default 0.10, Ultralytics' "about
  0-10 % background images") caps the share of empty units in **train**,
  sampled with ``--seed``; val and test keep every unit.

**Class drift.** Labels must use the frozen ``yolo_classes_v1`` ids: a dataset
whose ``data.yaml`` names differ from it, a box whose class id is >= 29 (a
world type appended by ``sim_labels.classes_for``) or whose type disagrees
with its id is refused; a dataset without labels (``label_source: none``,
e.g. a world that failed to load) is refused unless ``--unlabelled``.

**Aug Studio.** Tile sidecars carry only the intensity declaration
(``intensity_mapping: log``, ``log_range_db``) and the along-track facts: a
tile's first column is not the nadir, so the Aug Studio's range map
(``core/image.py``, range from the nadir edge) would be wrong on it. Run the
Aug Studio on the full-picture dataset, never on tiles (schema versioned).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np

PKG = Path(__file__).resolve().parents[1] / "blueboat_perception"
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from blueboat_perception import __version__  # noqa: E402
from blueboat_perception.core.dataroot import claim_dir, data_root, safe_name  # noqa: E402
from blueboat_perception.core.image import SeabedImageInput  # noqa: E402
from blueboat_perception.core.preprocess import make_preprocessor, near_range_mask  # noqa: E402
from blueboat_perception.core.sim_labels import load_class_map  # noqa: E402
from blueboat_perception.core.tiling import TilingConfig, cut_tiles  # noqa: E402
from label_visibility import (DEFAULT_MIN_SHADOW_M, DEFAULT_THRESHOLD_DB,  # noqa: E402
                              box_contrast, classify)

MANIFEST_SCHEMA = "blueboat_perception/tile_dataset_manifest/1"
SIDECAR_SCHEMA = "blueboat_perception/tile_sidecar/1"
SPLITS = ("train", "val", "test")
CLASS_MAP = "yolo_classes_v1"
#: AcquisitionMeta fields that stay true for a tile (no across-track geometry)
TILE_META_FIELDS = ("intensity_mapping", "log_range_db", "altitude_m", "res_along_m",
                    "speed_mps", "heading_deg", "depth_m", "prf_hz", "timestamp")


class DatasetRefused(RuntimeError):
    pass


def _utc() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def code_version() -> Dict[str, Any]:
    root = Path(__file__).resolve().parents[1]
    try:
        sha = subprocess.run(["git", "-C", str(root), "rev-parse", "HEAD"],
                             capture_output=True, text=True, timeout=10).stdout.strip()
        dirty = bool(subprocess.run(["git", "-C", str(root), "status", "--porcelain"],
                                    capture_output=True, text=True, timeout=10).stdout.strip())
    except (OSError, subprocess.SubprocessError):
        sha, dirty = "", None
    return {"perception_version": __version__, "git_commit": sha or None, "dirty": dirty}


def load_split(path: Path) -> Dict[str, str]:
    """{world or dataset name: split}; a name in two splits is refused."""
    text = path.read_text(encoding="utf-8")
    try:
        doc = json.loads(text)
    except ValueError:
        import yaml
        doc = yaml.safe_load(text) or {}
    out: Dict[str, str] = {}
    for split, names in (doc or {}).items():
        if split not in SPLITS:
            raise DatasetRefused(f"{path}: unknown split {split!r} (use {SPLITS})")
        for n in names or []:
            if str(n) in out:
                raise DatasetRefused(f"{path}: {n!r} is in {out[str(n)]} and {split}")
            out[str(n)] = split
    return out


def split_of(ds: Path, manifest: dict, split: Dict[str, str]) -> Tuple[str, str]:
    """(split, the key that matched)."""
    world = Path(manifest.get("world_dir") or "").name
    hits = {k: split[k] for k in (world, ds.name) if k and k in split}
    if not hits:
        raise DatasetRefused(f"{ds.name}: neither its world {world!r} nor its name is in "
                             f"the split file")
    if len(set(hits.values())) > 1:
        raise DatasetRefused(f"{ds.name}: world and name fall in different splits {hits}")
    key = world if world in hits else ds.name
    return hits[key], key


def check_classes(ds: Path, manifest: dict, v1: List[str], unlabelled: bool) -> bool:
    """Whether the dataset is labelled; refuses class drift (module docstring)."""
    import yaml
    doc = yaml.safe_load((ds / "data.yaml").read_text(encoding="utf-8")) or {}
    names = doc.get("names") or {}
    names = ({int(k): str(v) for k, v in names.items()} if isinstance(names, dict)
             else {i: str(v) for i, v in enumerate(names)})
    if not names or manifest.get("label_source") in (None, "none"):
        if not unlabelled:
            raise DatasetRefused(f"{ds.name}: no labels (label_source "
                                 f"{manifest.get('label_source')!r}); pass --unlabelled to "
                                 "write background-only tiles")
        return False
    first = [names.get(i) for i in range(len(v1))]
    if first != v1:
        raise DatasetRefused(f"{ds.name}: data.yaml classes differ from {CLASS_MAP} "
                             f"(class drift): {first}")
    extra = {i: n for i, n in names.items() if i >= len(v1)}
    if extra:
        print(f"{ds.name}: world-only classes appended after the 29 ({extra}); boxes of "
              "those classes are refused", file=sys.stderr)
    return True


def check_box_class(ds: Path, stem: str, b: dict, v1: List[str]) -> None:
    cid = int(b["class_id"])
    if cid >= len(v1) or v1[cid] != b.get("type"):
        raise DatasetRefused(f"{ds.name}/{stem}: box class {cid} {b.get('type')!r} is not "
                             f"{CLASS_MAP} (class drift; a new class needs yolo_classes_v2)")


def picture(ds: Path, stem: str, manifest: dict):
    """(SeabedImageInput with raw dB, stored grey picture, valid, sidecar)."""
    import cv2
    sidecar = json.loads((ds / "images" / f"{stem}.json").read_text(encoding="utf-8"))
    db = np.load(ds / "raw" / f"{stem}_db.npz")["db"]
    meta = sidecar["gcs_metadata"]
    img = SeabedImageInput.from_metadata(meta, db,
                                         stream_id=(sidecar.get("perception") or {})
                                         .get("stream_id") or "dataset")
    gray = cv2.imread(str(ds / "images" / f"{stem}.png"), cv2.IMREAD_GRAYSCALE)
    if gray is None or gray.shape != db.shape:
        raise DatasetRefused(f"{ds.name}/{stem}: training image missing or of the wrong size")
    di = manifest.get("detector_input") or {}
    mask_m = float((di.get("params") or {}).get("near_range_mask_m", 0.0))
    valid = np.isfinite(db)
    cols = near_range_mask(img, mask_m)
    if cols is not None:
        valid[:, cols] = False
    return img, gray, valid, sidecar


def verify_gray(ds: Path, stem: str, img, gray, manifest: dict) -> None:
    di = manifest.get("detector_input") or {}
    if di.get("name") != "fixed_gray":
        return
    p = di.get("params") or {}
    pre = make_preprocessor("fixed_gray", mapping=p.get("mapping", "fixed_gray_v1"),
                            near_range_mask_m=float(p.get("near_range_mask_m", 0.0)))
    if pre.params.get("mapping_sha256") != p.get("mapping_sha256"):
        raise DatasetRefused(f"{ds.name}: made with another {p.get('mapping')} file "
                             "(SHA-256 differs)")
    if not np.array_equal(pre.run(img).gray, gray):
        raise DatasetRefused(f"{ds.name}/{stem}: stored training image != fixed_gray of the "
                             "raw dB (not the detector input of this code)")


def tile_labels(boxes: List[dict], vis: List[str], c0: int, cols: int, rows: int, T: int,
                a) -> Tuple[List[str], Dict[str, int], bool]:
    """YOLO lines of one tile, drop counts, and whether a faint box excludes it."""
    lines: List[str] = []
    n = {"labels": 0, "dropped_partial": 0, "dropped_narrow": 0}
    c1t = c0 + cols - 1
    exclude = False
    for b, v in zip(boxes, vis):
        p = b["box_px"]
        bc0, bc1 = int(p["c0"]), int(p["c1"])
        lc0, lc1 = max(bc0, c0), min(bc1, c1t)
        if lc1 < lc0 or int(p["r0"]) >= rows:
            continue                                          # not in this tile
        if v == "faint":
            if (lc1 - lc0 + 1) >= a.faint_unit_fraction * (bc1 - bc0 + 1):
                exclude = True
            continue
        h0, h1 = int(b["highlight_cols"][0]), int(b["highlight_cols"][1])
        inside = max(0, min(h1, c1t) - max(h0, c0) + 1)
        if inside < a.min_highlight_fraction * (h1 - h0 + 1):
            n["dropped_partial"] += 1
            continue
        if lc1 - lc0 + 1 < a.min_width_px:
            n["dropped_narrow"] += 1
            continue
        r0, r1 = int(p["r0"]), min(int(p["r1"]), rows - 1)
        x0, x1 = lc0 - c0, lc1 - c0
        lines.append(f"{int(b['class_id'])} {(x0 + x1 + 1) / 2 / T:.6f} {(r0 + r1 + 1) / 2 / T:.6f} "
                     f"{(x1 - x0 + 1) / T:.6f} {(r1 - r0 + 1) / T:.6f}")
        n["labels"] += 1
    return lines, n, exclude


def full_labels(boxes: List[dict], vis: List[str], H: int, W: int):
    lines, exclude = [], False
    for b, v in zip(boxes, vis):
        if v == "faint":
            exclude = True
            continue
        p = b["box_px"]
        lines.append(f"{int(b['class_id'])} {(p['c0'] + p['c1'] + 1) / 2 / W:.6f} "
                     f"{(p['r0'] + p['r1'] + 1) / 2 / H:.6f} {(p['c1'] - p['c0'] + 1) / W:.6f} "
                     f"{(p['r1'] - p['r0'] + 1) / H:.6f}")
    return lines, {"labels": len(lines), "dropped_partial": 0, "dropped_narrow": 0}, exclude


def plan_units(sources, a, v1, counts) -> List[dict]:
    """Pass 1: every unit (tile or picture) with its labels, nothing written."""
    cfg = TilingConfig(a.tile, a.stride, a.min_valid_fraction)
    units: List[dict] = []
    for src in sources:
        ds, manifest, split, labelled = src["dir"], src["manifest"], src["split"], src["labelled"]
        c = counts[split]
        for js in sorted((ds / "images").glob("*.json")):
            stem = js.stem
            img, gray, valid, sidecar = picture(ds, stem, manifest)
            if not a.no_verify:
                verify_gray(ds, stem, img, gray, manifest)
            boxes = (((sidecar.get("perception") or {}).get("labels") or {}).get("boxes") or []
                     if labelled else [])
            for b in boxes:
                check_box_class(ds, stem, b, v1)
            vis = [classify(box_contrast(img.db, boxes, b), float(b.get("shadow_m") or 0.0),
                            a.visibility_db, DEFAULT_MIN_SHADOW_M) for b in boxes]
            for v in vis:
                c["boxes_" + v] += 1
            c["pictures"] += 1
            H, W = gray.shape
            if a.mode == "full":
                parts = [(None, 0, W, H)]
            else:
                parts = []
                for t in cut_tiles(gray, valid, cfg):
                    if t.skipped:
                        c["tiles_skipped"] += 1
                        continue
                    parts.append((t, t.c0, t.cols, t.rows))
            for t, c0, cols, rows in parts:
                if a.mode == "full":
                    lines, n, exclude = full_labels(boxes, vis, H, W)
                else:
                    lines, n, exclude = tile_labels(boxes, vis, c0, cols, rows, a.tile, a)
                for k, v in n.items():
                    c[k] += v
                if exclude:
                    c["units_excluded_faint"] += 1
                    continue
                units.append({"source": src["index"], "stem": stem, "c0": c0, "cols": cols,
                              "rows": rows, "split": split, "lines": lines,
                              "unit": f"{safe_name(ds.name)}__{stem}"
                                      + ("" if a.mode == "full" else f"_c{c0:04d}")})
    return units


def sample_empty(units: List[dict], fraction: float, seed: int, counts) -> List[dict]:
    """Cap the share of empty units in train (seeded, deterministic)."""
    train = [u for u in units if u["split"] == "train"]
    empty = sorted((u for u in train if not u["lines"]), key=lambda u: u["unit"])
    full = len(train) - len(empty)
    if fraction >= 1.0:
        keep = len(empty)
    else:
        keep = min(len(empty), int(math.floor(fraction / (1.0 - fraction) * full)))
    kept = set(u["unit"] for u in random.Random(seed).sample(empty, keep))
    counts["train"]["empty_dropped"] = len(empty) - keep
    return [u for u in units if u["split"] != "train" or u["lines"] or u["unit"] in kept]


def write_units(out: Path, units: List[dict], sources, a, counts) -> None:
    import cv2
    by_src: Dict[int, List[dict]] = {}
    for u in units:
        by_src.setdefault(u["source"], []).append(u)
    for si, us in sorted(by_src.items()):
        src = sources[si]
        cache: Dict[str, Any] = {}
        cfg = TilingConfig(a.tile, a.stride, a.min_valid_fraction)
        for u in us:
            if u["stem"] not in cache:                 # one picture at a time
                cache.clear()
                img, gray, valid, sidecar = picture(src["dir"], u["stem"], src["manifest"])
                tiles = ({} if a.mode == "full" else
                         {t.c0: t for t in cut_tiles(gray, valid, cfg)})
                cache[u["stem"]] = (img, gray, sidecar, tiles)
            img, gray, sidecar, tiles = cache[u["stem"]]
            arr = gray if a.mode == "full" else tiles[u["c0"]].gray
            split = u["split"]
            ok, buf = cv2.imencode(".png", np.ascontiguousarray(arr, np.uint8))
            if not ok:
                raise RuntimeError("PNG encoding failed")
            with open(out / "images" / split / f"{u['unit']}.png", "xb") as f:
                f.write(buf.tobytes())
            with open(out / "labels" / split / f"{u['unit']}.txt", "x", encoding="utf-8") as f:
                f.write("".join(ln + "\n" for ln in u["lines"]))
            meta = {k: sidecar[k] for k in TILE_META_FIELDS if k in sidecar}
            H, W = gray.shape
            meta.update({"schema": SIDECAR_SCHEMA,
                         "aug_studio": "not an Aug Studio input: a tile does not start at the "
                                       "nadir; augment the full-picture dataset instead",
                         "perception_tile": {"source_dataset": src["dir"].name,
                                             "source_stem": u["stem"], "mode": a.mode,
                                             "c0": u["c0"], "cols": u["cols"], "rows": u["rows"],
                                             "picture_shape": [H, W],
                                             "nadir_col_in_tile": (W // 2 - u["c0"]
                                                                   if a.mode == "tiles" else W // 2),
                                             "bin_pitch_m": img.bin_pitch_m,
                                             "key": (sidecar.get("perception") or {}).get("key")}})
            with open(out / "images" / split / f"{u['unit']}.json", "x", encoding="utf-8") as f:
                json.dump(meta, f, indent=1, sort_keys=True)
            c = counts[split]
            c["units"] += 1
            c["units_with_labels"] += bool(u["lines"])
            for ln in u["lines"]:
                cid = int(ln.split()[0])
                c["by_class"][cid] = c["by_class"].get(cid, 0) + 1


def data_yaml(names: List[str], splits: List[str], channels: int) -> str:
    head = ("# BlueBoat-Perception tile dataset (simulated data). Split by world: see\n"
            "# manifest.json. 'path' is omitted: Ultralytics then uses this file's folder.\n")
    lines = [f"{s}: images/{s}" for s in splits]
    if channels == 1:
        lines.append("channels: 1")
    return (head + "\n".join(lines) + f"\nnc: {len(names)}\nnames:\n"
            + "".join(f"  {i}: {n}\n" for i, n in enumerate(names)))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("datasets", nargs="+", type=Path)
    ap.add_argument("--split", type=Path, required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--data-dir", default="", help="data root (as the node's data_dir)")
    ap.add_argument("--mode", choices=("tiles", "full"), default="tiles",
                    help="tiles (256 x 256) or full pictures (the squeeze baseline)")
    ap.add_argument("--tile", type=int, default=256)
    ap.add_argument("--stride", type=int, default=128)
    ap.add_argument("--min-valid-fraction", type=float, default=0.001)
    ap.add_argument("--min-highlight-fraction", type=float, default=0.5)
    ap.add_argument("--min-width-px", type=int, default=4)
    ap.add_argument("--visibility-db", type=float, default=DEFAULT_THRESHOLD_DB)
    ap.add_argument("--faint-unit-fraction", type=float, default=0.5)
    ap.add_argument("--empty-fraction", type=float, default=0.10)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--unlabelled", action="store_true")
    ap.add_argument("--no-verify", action="store_true",
                    help="skip the bit-exact check of the stored detector input")
    a = ap.parse_args(argv)

    v1 = load_class_map(CLASS_MAP)
    split = load_split(a.split)
    sources = []
    for i, ds in enumerate(a.datasets):
        ds = ds.expanduser().resolve()
        mtext = (ds / "manifest.json").read_text(encoding="utf-8")
        manifest = json.loads(mtext)
        s, key = split_of(ds, manifest, split)
        labelled = check_classes(ds, manifest, v1, a.unlabelled)
        sources.append({"index": i, "dir": ds, "manifest": manifest, "split": s, "key": key,
                        "labelled": labelled,
                        "manifest_sha256": hashlib.sha256(mtext.encode()).hexdigest()})
    used = sorted({s["split"] for s in sources}, key=SPLITS.index)
    if "train" not in used or "val" not in used:
        print("warning: no train or no val dataset", file=sys.stderr)

    def zero():
        return {"pictures": 0, "units": 0, "units_with_labels": 0, "labels": 0,
                "dropped_partial": 0, "dropped_narrow": 0, "units_excluded_faint": 0,
                "boxes_visible": 0, "boxes_faint": 0, "boxes_unmeasurable": 0,
                "tiles_skipped": 0, "empty_dropped": 0, "by_class": {}}
    counts = {s: zero() for s in SPLITS}
    units = plan_units(sources, a, v1, counts)
    units = sample_empty(units, a.empty_fraction, a.seed, counts)

    root = data_root(a.data_dir)
    out = claim_dir(root / "data" / "Tile_data", safe_name(a.name))
    for s in used:
        (out / "images" / s).mkdir(parents=True)
        (out / "labels" / s).mkdir(parents=True)
    write_units(out, units, sources, a, counts)
    (out / "data.yaml").write_text(data_yaml(v1, used, 3), encoding="utf-8")
    (out / "data_gray1.yaml").write_text(data_yaml(v1, used, 1), encoding="utf-8")
    manifest = {
        "schema": MANIFEST_SCHEMA, "name": out.name, "created_utc": _utc(),
        "code": code_version(), "seed": a.seed, "mode": a.mode,
        "data": "simulated" if all(s["manifest"].get("world_dir") for s in sources)
                else "mixed or real",
        "tiling": TilingConfig(a.tile, a.stride, a.min_valid_fraction).to_json(),
        "rules": {"visibility_db": a.visibility_db, "min_shadow_m": DEFAULT_MIN_SHADOW_M,
                  "faint_unit_fraction": a.faint_unit_fraction,
                  "tile_edge": "B + (i): keep if >= min_highlight_fraction of the highlight "
                               "columns are inside and the clipped box >= min_width_px wide; "
                               "else drop the label, keep the tile",
                  "min_highlight_fraction": a.min_highlight_fraction,
                  "min_width_px": a.min_width_px, "empty_fraction_train": a.empty_fraction},
        "classes": {"map": CLASS_MAP, "nc": len(v1)},
        "splits": {s: sorted({src["key"] for src in sources if src["split"] == s})
                   for s in used},
        "sources": [{"dir": str(s["dir"]), "name": s["dir"].name, "split": s["split"],
                     "world_dir": s["manifest"].get("world_dir"),
                     "label_source": s["manifest"].get("label_source"),
                     "detector_input": s["manifest"].get("detector_input"),
                     "manifest_sha256": s["manifest_sha256"]} for s in sources],
        "counts": {s: dict(c, by_class={v1[k] if k < len(v1) else str(k): n
                                         for k, n in sorted(c["by_class"].items())})
                   for s, c in counts.items()},
        "config": {k: (str(v) if isinstance(v, Path) else
                                         [str(x) for x in v] if isinstance(v, list) else v)
                                     for k, v in vars(a).items()}}
    with open(out / "manifest.json", "x", encoding="utf-8") as f:
        json.dump(manifest, f, indent=1, sort_keys=True)
    print(f"{sum(c['units'] for c in counts.values())} unit(s) -> {out}")
    for s in used:
        c = counts[s]
        print(f"  {s}: {c['units']} units ({c['units_with_labels']} labelled), {c['labels']} "
              f"labels, {c['dropped_partial']} cut labels dropped, {c['boxes_faint']} faint "
              f"boxes, {c['units_excluded_faint']} units left out")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except DatasetRefused as exc:
        print(f"refused: {exc}", file=sys.stderr)
        sys.exit(2)
