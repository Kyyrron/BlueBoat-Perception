"""Detector-input preprocessing (per image, computable from image + metadata).

The detector never sees the GCS's image generation change: SSS keeps forming the
seabed pictures, their display PNG and the raw dB exactly as before (decision
6.1), and this module builds a **separate** detector-input picture from them.
The original arrays are never modified.

Two mappings, switched by one configuration value (``preprocess``):

* ``identity`` -- the baseline: the detector input is the GCS display PNG as it
  is. That PNG goes through the GCS display model, which is re-fitted during a
  live mission, so the same seabed maps to different grey levels early and late
  in a mission and live vs a replay (``PREPROCESSING_STUDY.md`` §5, M5 / M6);
* ``fixed_gray`` -- decision 6.4: one fixed, reproducible mapping from the raw dB
  (the 32FC1 the GCS publishes / the session's ``_world.npz``), with parameters
  fitted once on real training data (``tools/fit_detector_input.py``) and loaded
  from a versioned file::

      gray = round(255 · clip((db − g − R(s) − lo) / (hi − lo), 0, 1)),  NaN → 0

  ``g`` = the row's receive-gain step, ``R(s)`` = the fixed empirical
  range-reference curve (no TVG law assumed), ``[lo, hi]`` = the fixed window.
  A pixel's grey level depends only on its own dB, its row's gain and its
  column's slant range: no per-image min/max stretch, no histogram equalisation.

Independent of the mapping, ``near_range_mask_m`` (decision 6.5) blanks the
detector input at slant ranges below the given distance (0.0 = off, 0.75 = the
GCS's live nadir blank); masked pixels are 0 and ``valid`` False.

Not implemented, behind explicit boundaries (decisions 6.6, 6.7): any roll or
radiometric correction beyond the above, geometric correction, despeckling,
histogram equalisation. Requesting one raises rather than silently running an
unvalidated formula.

Placement (Q4): a step needing per-ping data or changing geometry belongs in
SSS's imager; per-image steps live here and run identically live, offline and in
dataset mode. Rule: a preprocessor never modifies its input arrays.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional, Protocol

import numpy as np

from . import geometry as geo
from .image import SeabedImageInput

FIXED_GRAY_SCHEMA = "blueboat_perception/detector_input_fixed_gray/1"
DATA_DIR = Path(__file__).resolve().parent / "data"
DEFAULT_FIXED_GRAY = "fixed_gray_v1"
#: Deferred options (decision 6.7: "not yet") -- named so a configuration that
#: asks for one fails loudly instead of being ignored.
DEFERRED = ("geometric_correction", "despeckle", "histogram_equalisation", "retinex")


@dataclass(frozen=True, eq=False)
class DetectorInput:
    """What a detector sees.

    ``db`` / ``png`` are the originals, untouched. ``gray`` is the detector's
    uint8 [H, W] input picture (None = it cannot be built: no pixels, or no PNG
    for ``identity``); ``valid`` marks real, unmasked samples."""

    image: SeabedImageInput
    db: Optional[np.ndarray]
    png: Optional[np.ndarray]
    gray: Optional[np.ndarray] = None
    valid: Optional[np.ndarray] = None
    notes: Dict[str, Any] = field(default_factory=dict)


class Preprocessor(Protocol):
    name: str
    params: Dict[str, Any]

    def run(self, img: SeabedImageInput) -> DetectorInput: ...


def near_range_mask(img: SeabedImageInput, distance_m: float) -> Optional[np.ndarray]:
    """Columns [W] whose slant range is below ``distance_m`` (None when off).
    The boundary column (slant exactly ``distance_m``) is kept."""
    if not distance_m or distance_m <= 0.0:
        return None
    _port, s = geo.column_slant(img.shape[1], img.bin_pitch_m)
    return s < float(distance_m)


def _finish(img, gray, mask_cols, extra_valid=None) -> tuple:
    valid = None
    if img.db is not None:
        valid = np.isfinite(img.db)
        if extra_valid is not None:
            valid &= extra_valid
    if mask_cols is not None:
        if gray is not None:
            gray = gray.copy()
            gray[:, mask_cols] = 0
        if valid is not None:
            valid = valid.copy()
            valid[:, mask_cols] = False
    return gray, valid


class _Base:
    version = "1"

    def __init__(self, near_range_mask_m: float = 0.0,
                 roll_correction: bool = False) -> None:
        if roll_correction:
            raise NotImplementedError(
                "roll correction is disabled: no validated method exists for this "
                "sonar yet (PREPROCESSING_STUDY.md §6.6); roll stays in the records")
        self.near_range_mask_m = float(near_range_mask_m)
        if self.near_range_mask_m < 0.0:
            raise ValueError("near_range_mask_m must be >= 0")

    @property
    def params(self) -> Dict[str, Any]:
        return {"near_range_mask_m": self.near_range_mask_m}


class Identity(_Base):
    """Baseline: the detector input is the GCS display PNG, unchanged."""

    name = "identity"

    def run(self, img: SeabedImageInput) -> DetectorInput:
        gray, valid = _finish(img, img.png, near_range_mask(img, self.near_range_mask_m))
        return DetectorInput(image=img, db=img.db, png=img.png, gray=gray, valid=valid)


def load_fixed_gray(name_or_path: str = DEFAULT_FIXED_GRAY) -> Dict[str, Any]:
    """The parameter file (packaged ``core/data/<name>.json``, or a path)."""
    p = Path(name_or_path)
    if not p.suffix:
        p = DATA_DIR / f"{name_or_path}.json"
    text = p.read_text(encoding="utf-8")
    params = json.loads(text)
    if params.get("schema") != FIXED_GRAY_SCHEMA:
        raise ValueError(f"{p}: not a {FIXED_GRAY_SCHEMA} file")
    params["_sha256"] = hashlib.sha256(text.encode("utf-8")).hexdigest()
    return params


class FixedGray(_Base):
    """Decision 6.4: one fixed mapping from raw dB to the detector's 0-255 range."""

    name = "fixed_gray"

    def __init__(self, mapping: str = DEFAULT_FIXED_GRAY, near_range_mask_m: float = 0.0,
                 roll_correction: bool = False) -> None:
        super().__init__(near_range_mask_m, roll_correction)
        self.mapping = load_fixed_gray(mapping)
        rr = self.mapping["range_reference"]
        self._x = np.asarray(rr["log10_slant_m"], np.float64)
        self._r = np.asarray(rr["db"], np.float64)
        self._lo = float(self.mapping["window_db"]["lo"])
        self._hi = float(self.mapping["window_db"]["hi"])
        if not self._hi > self._lo:
            raise ValueError("fixed_gray window: hi must exceed lo")

    @property
    def params(self) -> Dict[str, Any]:
        return {"near_range_mask_m": self.near_range_mask_m,
                "mapping": self.mapping["name"], "mapping_sha256": self.mapping["_sha256"],
                "window_db": [self._lo, self._hi]}

    def reference(self, slant_m: np.ndarray) -> np.ndarray:
        """R(s): the fixed range-reference curve, held flat outside its range."""
        x = np.log10(np.maximum(np.asarray(slant_m, np.float64), 1e-3))
        return np.interp(x, self._x, self._r)

    def map_db(self, img: SeabedImageInput) -> np.ndarray:
        """The fixed mapping of the raw dB, uint8 [H, W]."""
        H, W = img.shape
        port, s = geo.column_slant(W, img.bin_pitch_m)
        g = np.where(port[None, :], img.rows.gain_port[:, None], img.rows.gain_stbd[:, None])
        y = np.asarray(img.db, np.float64) - g - self.reference(s)[None, :]
        u = np.clip((y - self._lo) / (self._hi - self._lo), 0.0, 1.0)
        out = np.rint(255.0 * np.nan_to_num(u, nan=0.0)).astype(np.uint8)
        return out

    def run(self, img: SeabedImageInput) -> DetectorInput:
        gray = None if img.db is None else self.map_db(img)
        gray, valid = _finish(img, gray, near_range_mask(img, self.near_range_mask_m))
        return DetectorInput(image=img, db=img.db, png=img.png, gray=gray, valid=valid,
                             notes={"mapping": self.mapping["name"]})


PREPROCESSORS = {"identity": Identity, "fixed_gray": FixedGray}


def make_preprocessor(name: str, **kwargs) -> Preprocessor:
    if name in DEFERRED:
        raise NotImplementedError(
            f"{name!r} is a deferred option (PREPROCESSING_STUDY.md §6.7: not yet); "
            "evaluate it against the original images before adding it here")
    try:
        cls = PREPROCESSORS[name]
    except KeyError:
        raise ValueError(f"unknown preprocessor {name!r}; known: {sorted(PREPROCESSORS)}")
    if cls is Identity:
        kwargs.pop("mapping", None)
    return cls(**kwargs)
