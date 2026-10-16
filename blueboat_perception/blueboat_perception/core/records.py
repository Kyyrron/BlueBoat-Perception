"""Image records (``blueboat_perception/image_record/1``) and canonical JSON.

One record per input image, **with or without detections** -- the record
of an image that was searched and held nothing is negative evidence for
the future Belief module; ``detections: null`` (pixels missing) means
*unknown*. Field contract: ``docs/topics.md`` §4.

Canonical JSON: sorted keys, compact separators, NaN/inf -> ``null``,
Python's shortest round-trip float repr. Everything run-dependent (wall
time, latency, host, stream counters) lives under ``provenance``; equality
between runs (live vs offline, run vs run) compares records with
``provenance`` and ``source`` removed (:func:`comparable`).
"""

from __future__ import annotations

import json
import math
from typing import Any, Dict, Iterable, List, Optional

import numpy as np

RECORD_SCHEMA = "blueboat_perception/image_record/1"
INPUT_SCHEMA = "blueboat_sss/seabed_image/1"
RUN_DEPENDENT_KEYS = ("provenance", "source")


def clean(obj: Any) -> Any:
    """JSON-safe copy: numpy -> Python, non-finite floats -> None."""
    if isinstance(obj, dict):
        return {str(k): clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [clean(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return [clean(v) for v in obj.tolist()]
    if isinstance(obj, (bool, np.bool_)):
        return bool(obj)
    if isinstance(obj, (int, np.integer)):
        return int(obj)
    if isinstance(obj, (float, np.floating)):
        f = float(obj)
        return f if math.isfinite(f) else None
    return obj


def canonical_json(obj: Any) -> str:
    return json.dumps(clean(obj), sort_keys=True, separators=(",", ":"),
                      allow_nan=False)


def comparable(record: Dict[str, Any]) -> Dict[str, Any]:
    """The record minus its run-dependent keys."""
    return {k: v for k, v in record.items() if k not in RUN_DEPENDENT_KEYS}


def comparable_json(record: Dict[str, Any]) -> str:
    return canonical_json(comparable(record))


def float_list(a: Optional[np.ndarray]) -> Optional[List[Optional[float]]]:
    if a is None:
        return None
    return [clean(v) for v in np.asarray(a, np.float64).tolist()]


def stats(a: np.ndarray) -> Dict[str, Optional[float]]:
    a = np.asarray(a, np.float64)
    a = a[np.isfinite(a)]
    if a.size == 0:
        return {"min": None, "mean": None, "max": None}
    return {"min": float(a.min()), "mean": float(a.mean()), "max": float(a.max())}


def read_jsonl(path) -> Iterable[Dict[str, Any]]:
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                yield json.loads(line)
