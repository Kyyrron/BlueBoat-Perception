"""How visible a simulated label box is in its picture (shared by
``draw_labels.py --stats`` and ``make_tile_dataset.py``; not installed).

Per box, the raw dB (``raw/<stem>_db.npz``) of three regions:

- the box's highlight columns over its rows;
- its shadow columns over its rows;
- the background: the same columns over the picture rows that no box of
  that side covers.

**Visibility rule (owner's decision 2026-10-23, provisional):** a box is
``visible`` if its highlight is at least ``threshold_db`` above the
background **or** its shadow at least ``threshold_db`` below it (shadow
measured only when it is at least ``min_shadow_m`` long); ``faint``
otherwise; ``unmeasurable`` when neither contrast can be measured (no
background rows, or no finite samples) -- such a box is kept. The 2 dB
default is an engineering starting point, to be tuned on the first
simulated datasets (``.claude/TODO.md``). The simulator's own ``visible``
flag only says the object is taller than 5 mm and lit (renderer
``GroundTruthContact``); it is not a contrast.
"""

from __future__ import annotations

from typing import Dict, Iterable, List

import numpy as np

DEFAULT_THRESHOLD_DB = 2.0
DEFAULT_MIN_SHADOW_M = 0.05


def region_mean(db: np.ndarray, rows: Iterable[int], c0: int, c1: int) -> float:
    rows = list(rows)
    if c1 < c0 or not rows:
        return float("nan")
    v = db[np.asarray(rows)][:, c0:c1 + 1]
    v = v[np.isfinite(v)]
    return float(v.mean()) if v.size else float("nan")


def box_contrast(db: np.ndarray, boxes: List[dict], b: dict) -> Dict[str, float]:
    """The three region means of box ``b`` among its picture's ``boxes``
    (the sidecar's ``perception.labels.boxes``)."""
    H = db.shape[0]
    p = b["box_px"]
    rows = range(p["r0"], p["r1"] + 1)
    taken = set()
    for o in boxes:
        if o["side"] == b["side"]:
            taken.update(range(o["box_px"]["r0"], o["box_px"]["r1"] + 1))
    bg_rows = [r for r in range(H) if r not in taken]
    hc = b["highlight_cols"]
    sc = b.get("shadow_cols")
    return {"highlight_db": region_mean(db, rows, hc[0], hc[1]),
            "background_at_highlight_db": region_mean(db, bg_rows, hc[0], hc[1]),
            "shadow_db": region_mean(db, rows, sc[0], sc[1]) if sc else float("nan"),
            "background_at_shadow_db": (region_mean(db, bg_rows, sc[0], sc[1]) if sc
                                        else float("nan"))}


def classify(stats: Dict[str, float], shadow_m: float,
             threshold_db: float = DEFAULT_THRESHOLD_DB,
             min_shadow_m: float = DEFAULT_MIN_SHADOW_M) -> str:
    """``visible`` | ``faint`` | ``unmeasurable`` (module docstring)."""
    hl = stats["highlight_db"] - stats["background_at_highlight_db"]
    sh = (stats["shadow_db"] - stats["background_at_shadow_db"]
          if shadow_m >= min_shadow_m else float("nan"))
    measured = [v for v in (hl, sh) if np.isfinite(v)]
    if not measured:
        return "unmeasurable"
    if (np.isfinite(hl) and hl >= threshold_db) or (np.isfinite(sh) and sh <= -threshold_db):
        return "visible"
    return "faint"
