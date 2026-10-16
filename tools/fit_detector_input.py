#!/usr/bin/env python3
"""Fit the fixed detector-input grayscale mapping (decision 6.4) on training data.

Dev tool, not installed; read-only on the data (CM-7). It writes the parameter
file the ``fixed_gray`` preprocessor loads, so the same fixed mapping is used
for training, validation, offline and live inference. Refit only deliberately,
under a new name (``fixed_gray_v2.json`` ...): a model trained on v1 inputs
must keep seeing v1 inputs.

    /usr/bin/python3 tools/fit_detector_input.py --name fixed_gray_v1 \\
        --out blueboat_perception/blueboat_perception/core/data/fixed_gray_v1.json \\
        <image folder> [<image folder> ...]

What is fitted, from the raw dB the GCS saves (``_world.npz:intensity_db``) and
the picture metadata -- never from the display PNG:

1. ``z = db − g``: the device's receive-gain step removed, ``g`` = the row's
   ``gain_port_db`` / ``gain_stbd_db`` (``20·log10(analog_gain / 74.55)``, the
   GCS's measured convention);
2. ``R(s)``: one fixed range-reference curve, the median of ``z`` over seabed
   samples per log-spaced slant-range bin, both sides pooled. It is **empirical**:
   it assumes no TVG law (whether and how much the device compensates range
   loss is not identifiable from its output, ``PREPROCESSING_STUDY.md`` §1.4),
   so it cannot compensate anything twice;
3. the window ``[lo, hi]``: percentiles (0.5, 99.5) of ``z − R(s)`` over the
   same seabed samples.

Seabed sample = finite, slant ``s ≥`` the row's ``bottom_m`` and ``s ≥ 0.75 m``
(outside the transmit ringing).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve()
PERC = HERE.parents[1] / "blueboat_perception"
sys.path.insert(0, str(PERC))

from blueboat_perception.core import geometry as geo  # noqa: E402
from blueboat_perception.core.image import SeabedImageInput  # noqa: E402
from blueboat_perception.core.preprocess import FIXED_GRAY_SCHEMA  # noqa: E402
from blueboat_perception.core.session import list_images  # noqa: E402

LOG_S_EDGES = np.linspace(np.log10(0.5), np.log10(40.0), 25)   # 24 bins, 0.5-40 m
PCT = (0.5, 99.5)
RING_M = 0.75


def seabed_samples(img: SeabedImageInput):
    """(slant [n], z [n]) of one image's seabed samples."""
    H, W = img.shape
    port, s = geo.column_slant(W, img.bin_pitch_m)
    g = np.where(port[None, :], img.rows.gain_port[:, None], img.rows.gain_stbd[:, None])
    z = np.asarray(img.db, np.float64) - g
    ss = np.broadcast_to(s[None, :], z.shape)
    m = np.isfinite(z) & (ss >= np.maximum(img.rows.bottom[:, None], RING_M))
    return ss[m], z[m]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("folders", nargs="+", type=Path)
    ap.add_argument("--name", required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--max-images", type=int, default=60,
                    help="images per folder, evenly spaced (bounds memory)")
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args(argv)
    if a.out.exists() and not a.force:
        print(f"refusing to overwrite {a.out}: a trained model may depend on it; "
              "fit under a new name")
        return 1
    rng = np.random.default_rng(20261016)
    per_bin = [[] for _ in range(len(LOG_S_EDGES) - 1)]
    pairs = []
    used = []
    for folder in a.folders:
        files = [f for f in list_images(folder) if f.npz.is_file()]
        pick = files[:: max(1, len(files) // a.max_images)][: a.max_images]
        for f in pick:
            img = SeabedImageInput.from_session_files(f.json, f.npz, stream_id="fit")
            s, z = seabed_samples(img)
            if s.size == 0:
                continue
            if s.size > 20000:                      # bounded, seeded subsample
                k = rng.choice(s.size, 20000, replace=False)
                s, z = s[k], z[k]
            pairs.append((s, z))
            b = np.digitize(np.log10(s), LOG_S_EDGES) - 1
            for i in range(len(per_bin)):
                sel = z[b == i]
                if sel.size:
                    per_bin[i].append(sel)
        used.append({"folder": str(folder), "images_used": len(pick),
                     "images_available": len(files)})
    centers = 0.5 * (LOG_S_EDGES[:-1] + LOG_S_EDGES[1:])
    med = np.array([np.median(np.concatenate(v)) if v and sum(x.size for x in v) >= 200
                    else np.nan for v in per_bin])
    counts = [int(sum(x.size for x in v)) for v in per_bin]
    ok = np.isfinite(med)
    if ok.sum() < 3:
        print("not enough seabed samples")
        return 1
    # light smoothing (3-bin running median), then fill empty bins from the
    # nearest fitted one: the curve is held flat beyond the data, never extrapolated
    sm = med.copy()
    for i in range(med.size):
        w = med[max(0, i - 1): i + 2]
        w = w[np.isfinite(w)]
        sm[i] = np.median(w) if w.size else np.nan
    idx = np.flatnonzero(np.isfinite(sm))
    filled = np.interp(np.arange(sm.size), idx, sm[idx])
    s_all = np.concatenate([p[0] for p in pairs])
    z_all = np.concatenate([p[1] for p in pairs])
    y = z_all - np.interp(np.log10(s_all), centers, filled)
    lo, hi = (float(np.percentile(y, q)) for q in PCT)
    params = {
        "schema": FIXED_GRAY_SCHEMA, "name": a.name,
        "fitted_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "training_data": used, "seabed_samples": int(s_all.size),
        "seabed_definition": f"finite, slant >= bottom_m and >= {RING_M} m",
        "gain": "z = db - gain_port_db / gain_stbd_db of the row (20*log10(analog_gain/74.55))",
        "range_reference": {"log10_slant_m": [round(float(c), 6) for c in centers],
                            "db": [round(float(v), 4) for v in filled],
                            "samples_per_bin": counts,
                            "fitted_bins": [bool(v) for v in ok],
                            "outside": "held at the end values (no extrapolation)"},
        "window_db": {"lo": round(lo, 4), "hi": round(hi, 4), "percentiles": list(PCT)},
        "mapping": "gray = round(255 * clip((db - g - R(s) - lo) / (hi - lo), 0, 1)); "
                   "NaN -> 0",
        "assumes_no_tvg_law": True,
    }
    text = json.dumps(params, indent=1, sort_keys=True) + "\n"
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(text, encoding="utf-8")
    print(f"wrote {a.out} ({hashlib.sha256(text.encode()).hexdigest()[:12]}): "
          f"lo {lo:.2f} hi {hi:.2f} dB over {s_all.size} samples")
    return 0


if __name__ == "__main__":
    sys.exit(main())
