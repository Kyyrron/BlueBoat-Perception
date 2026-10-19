#!/usr/bin/env python3
"""Draw and check the labels of a Perception dataset (read-only).

    python3 tools/draw_labels.py DATASET --out DIR [--crops] [--stats] [--limit N]

For each image with labels it writes, into ``--out`` (never inside the dataset):
- ``<stem>.png``: the training image (``images/``), with every box drawn in red,
  its highlight columns tinted green and its shadow columns tinted blue (from
  the sidecar's ``perception.labels`` detail);
- with ``--crops``: ``<stem>_<k>_<type>.png``, each box with a margin, enlarged.

``--stats`` compares, per box, the raw dB (``raw/<stem>_db.npz``) of three
regions:
- the box's highlight columns over its rows;
- its shadow columns over its rows;
- the background: the same columns over the picture rows that no box of that
  side covers.

It prints the share of boxes whose highlight is brighter than the background
and whose shadow is darker, and writes ``label_stats.json``.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


def _boxes(sidecar: dict):
    return ((sidecar.get("perception") or {}).get("labels") or {}).get("boxes") or []


def _draw(img: np.ndarray, boxes, cv2) -> np.ndarray:
    rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    tint = rgb.copy()
    for b in boxes:
        r0, r1 = b["box_px"]["r0"], b["box_px"]["r1"]
        hc = b["highlight_cols"]
        tint[r0:r1 + 1, hc[0]:hc[1] + 1, 1] = 255
        if b.get("shadow_cols"):
            sc = b["shadow_cols"]
            tint[r0:r1 + 1, sc[0]:sc[1] + 1, 0] = 255
    rgb = cv2.addWeighted(rgb, 0.65, tint, 0.35, 0)
    for b in boxes:
        p = b["box_px"]
        cv2.rectangle(rgb, (p["c0"], p["r0"]), (p["c1"], p["r1"]), (0, 0, 255), 1)
        cv2.putText(rgb, b["type"], (p["c0"], max(p["r0"] - 2, 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)
    return rgb


def _region_mean(db, rows, c0, c1):
    if c1 < c0 or not len(rows):
        return float("nan")
    v = db[np.asarray(rows)][:, c0:c1 + 1]
    v = v[np.isfinite(v)]
    return float(v.mean()) if v.size else float("nan")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("dataset", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--crops", action="store_true")
    ap.add_argument("--stats", action="store_true")
    ap.add_argument("--limit", type=int, default=0, help="draw at most N images (0 = all)")
    a = ap.parse_args(argv)
    ds, out = a.dataset.resolve(), a.out.resolve()
    if out == ds or ds in out.parents:
        print("refusing an --out inside the dataset", file=sys.stderr)
        return 2
    out.mkdir(parents=True, exist_ok=True)
    import cv2
    drawn = 0
    stats = []
    for js in sorted((ds / "images").glob("*.json")):
        side = json.loads(js.read_text(encoding="utf-8"))
        boxes = _boxes(side)
        if not boxes:
            continue
        stem = js.stem
        img = cv2.imread(str(ds / "images" / f"{stem}.png"), cv2.IMREAD_GRAYSCALE)
        if a.limit == 0 or drawn < a.limit:
            cv2.imwrite(str(out / f"{stem}.png"), _draw(img, boxes, cv2))
            if a.crops:
                full = _draw(img, boxes, cv2)
                for k, b in enumerate(boxes):
                    p = b["box_px"]
                    m = 24
                    crop = full[max(p["r0"] - m, 0):p["r1"] + m + 1,
                                max(p["c0"] - m, 0):p["c1"] + m + 1]
                    scale = max(1, int(400 / max(crop.shape[:2])))
                    crop = cv2.resize(crop, None, fx=scale, fy=scale,
                                      interpolation=cv2.INTER_NEAREST)
                    cv2.imwrite(str(out / f"{stem}_{k}_{b['type']}.png"), crop)
            drawn += 1
        if a.stats:
            db = np.load(ds / "raw" / f"{stem}_db.npz")["db"]
            H = db.shape[0]
            for b in boxes:
                p = b["box_px"]
                rows = list(range(p["r0"], p["r1"] + 1))
                taken = set()
                for o in boxes:
                    if o["side"] == b["side"]:
                        taken.update(range(o["box_px"]["r0"], o["box_px"]["r1"] + 1))
                bg_rows = [r for r in range(H) if r not in taken]
                hc = b["highlight_cols"]
                sc = b.get("shadow_cols")
                hl = _region_mean(db, rows, hc[0], hc[1])
                bgh = _region_mean(db, bg_rows, hc[0], hc[1])
                sh = _region_mean(db, rows, sc[0], sc[1]) if sc else float("nan")
                bgs = _region_mean(db, bg_rows, sc[0], sc[1]) if sc else float("nan")
                stats.append({"stem": stem, "object_id": b["object_id"], "type": b["type"],
                              "rows": b["rows"], "shadow_m": b["shadow_m"],
                              "highlight_db": hl, "background_at_highlight_db": bgh,
                              "shadow_db": sh, "background_at_shadow_db": bgs})
    print(f"{drawn} image(s) drawn -> {out}")
    if a.stats:
        hl_ok = [s["highlight_db"] > s["background_at_highlight_db"] for s in stats
                 if np.isfinite(s["highlight_db"]) and np.isfinite(s["background_at_highlight_db"])]
        sh_ok = [s["shadow_db"] < s["background_at_shadow_db"] for s in stats
                 if s["shadow_m"] >= 0.05 and np.isfinite(s["shadow_db"])
                 and np.isfinite(s["background_at_shadow_db"])]
        summ = {"boxes": len(stats),
                "highlight_brighter": [sum(hl_ok), len(hl_ok)],
                "shadow_darker (shadow >= 5 cm)": [sum(sh_ok), len(sh_ok)],
                "median_highlight_minus_background_db": float(np.nanmedian(
                    [s["highlight_db"] - s["background_at_highlight_db"] for s in stats])),
                "median_shadow_minus_background_db": float(np.nanmedian(
                    [s["shadow_db"] - s["background_at_shadow_db"] for s in stats
                     if s["shadow_m"] >= 0.05] or [np.nan])),
                "per_box": stats}
        (out / "label_stats.json").write_text(json.dumps(summ, indent=1), encoding="utf-8")
        print(json.dumps({k: v for k, v in summ.items() if k != "per_box"}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
