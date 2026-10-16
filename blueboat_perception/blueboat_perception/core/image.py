"""Input model: one GCS seabed image, from the wire or from session files.

Both constructors build the same :class:`SeabedImageInput` from the same
information, which is what makes "offline gives the same records as live"
hold by construction:

* live, ``from_wire``: the metadata envelope of ``/sss_ai/seabed_image/metadata``
  plus the 32FC1 array of ``/sss_ai/seabed_image`` (and the mono8 PNG);
* offline, ``from_session_files``: ``metadata/seabed_XXXXX.json`` plus the
  ``intensity_db`` of ``metadata/seabed_XXXXX_world.npz`` (and the PNG).

The 32FC1 array is the exact float32 array the GCS saves as
``_world.npz:intensity_db``, and the metadata JSON round-trips Python floats
exactly, so the two inputs are bit-identical for every image saved in a
session.

Conventions (GCS, ``seabed_imager.py``): row 0 is the **newest** row;
column 0 is far-range port, the last column far-range starboard, the
transducer at the centre seam. ``t_start_s`` is the newest row time.

Rules:
* unknown or newer metadata schemas are accepted when the required keys are
  present;
* optional keys that are missing stay ``None``; nothing is invented;
* ``georef`` absent means convention 1 (every session before the Q6 fix).
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Union

import numpy as np

ENVELOPE_SCHEMA = "blueboat_sss/seabed_image/1"

#: Per-row keys every metadata schema since 4 carries (float arrays).
_REQUIRED_ROW_KEYS = {
    "t": "t_s", "x": "x_m", "y": "y_m", "yaw": "yaw_rad", "range": "range_m",
    "speed": "speed_mps", "altitude": "altitude_m", "bottom": "bottom_m",
    "along": "along_track_m", "gain_port": "gain_port_db",
    "gain_stbd": "gain_stbd_db",
}
#: Optional per-row keys (schema >= 5, the Q8 additions).
_OPTIONAL_ROW_KEYS = {
    "roll": "roll_rad", "pitch": "pitch_rad", "yaw_rate": "yaw_rate_rad_s",
    "attitude_age": "attitude_age_s",
}
_OPTIONAL_INT_ROW_KEYS = {
    "port_ping_number": "port_ping_number",
    "stbd_ping_number": "stbd_ping_number",
}


class InputError(ValueError):
    """The metadata lacks a key every supported schema has."""


@dataclass(frozen=True)
class Georef:
    """The image's pixel -> world convention (``metadata["georef"]``).

    * convention 1: ``y = sign·sqrt(max(s² − altitude_m², 0))``;
    * convention 2: ``y = sign·(lever_side + sqrt(max(s² − bottom_m², 0)))``.
    """

    convention: int = 1
    altitude_key: str = "altitude_m"
    lever_x_m: float = 0.0
    lever_y_port_m: float = 0.0
    lever_y_stbd_m: float = 0.0

    @classmethod
    def from_metadata(cls, meta: Mapping[str, Any]) -> "Georef":
        g = meta.get("georef")
        if not g:
            return cls()
        conv = int(g.get("convention", 1))
        lever = g.get("lever_arm_m") or {}
        altitude = str(g.get("altitude", "altitude_m" if conv == 1 else "bottom_m"))
        if altitude not in ("altitude_m", "bottom_m"):
            raise InputError(f"georef.altitude must be altitude_m|bottom_m, got {altitude!r}")
        return cls(convention=conv, altitude_key=altitude,
                   lever_x_m=float(lever.get("x", 0.0)),
                   lever_y_port_m=float(lever.get("y_port", 0.0)),
                   lever_y_stbd_m=float(lever.get("y_stbd", 0.0)))

    def to_json(self) -> dict:
        return {"convention": self.convention, "altitude": self.altitude_key,
                "lever_arm_m": {"x": self.lever_x_m, "y_port": self.lever_y_port_m,
                                "y_stbd": self.lever_y_stbd_m}}


@dataclass(frozen=True, eq=False)
class RowArrays:
    """Per-row arrays, row 0 = newest, aligned with the image rows."""

    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    range: np.ndarray
    speed: np.ndarray
    altitude: np.ndarray
    bottom: np.ndarray
    along: np.ndarray
    gain_port: np.ndarray
    gain_stbd: np.ndarray
    ping_index: np.ndarray
    # Optional (schema >= 5); None when no row carries the key, NaN / 0
    # where a row carries ``null``.
    roll: Optional[np.ndarray] = None
    pitch: Optional[np.ndarray] = None
    yaw_rate: Optional[np.ndarray] = None
    attitude_age: Optional[np.ndarray] = None
    port_ping_number: Optional[np.ndarray] = None
    stbd_ping_number: Optional[np.ndarray] = None

    @property
    def n(self) -> int:
        return int(self.t.size)


def _float_column(rows, key: str, required: bool) -> Optional[np.ndarray]:
    present = [key in r for r in rows]
    if not any(present):
        if required:
            raise InputError(f"rows_data lacks {key!r}")
        return None
    out = np.full(len(rows), np.nan, np.float64)
    for i, r in enumerate(rows):
        v = r.get(key)
        if v is not None:
            out[i] = float(v)
    return out


def _int_column(rows, key: str) -> Optional[np.ndarray]:
    if not any(key in r for r in rows):
        return None
    return np.array([int(r.get(key) or 0) for r in rows], np.int64)


def rows_from_metadata(meta: Mapping[str, Any]) -> RowArrays:
    rows = meta.get("rows_data")
    if not rows:
        raise InputError("metadata has no rows_data")
    cols = {name: _float_column(rows, key, True)
            for name, key in _REQUIRED_ROW_KEYS.items()}
    if not any("ping_index" in r for r in rows):
        raise InputError("rows_data lacks 'ping_index'")
    cols["ping_index"] = np.array([int(r.get("ping_index", i))
                                   for i, r in enumerate(rows)], np.int64)
    for name, key in _OPTIONAL_ROW_KEYS.items():
        cols[name] = _float_column(rows, key, False)
    for name, key in _OPTIONAL_INT_ROW_KEYS.items():
        cols[name] = _int_column(rows, key)
    return RowArrays(**cols)


@dataclass(frozen=True, eq=False)
class SeabedImageInput:
    """One seabed image as Perception sees it (read-only by convention)."""

    key: str
    stream_id: str
    image_id: int
    schema: int                       # the GCS metadata schema (4, 5, ...)
    db: Optional[np.ndarray]          # float32 [H, W], NaN = no sample; None = pixels missing
    png: Optional[np.ndarray]         # uint8 [H, W] (the session PNG), or None
    meta: Dict[str, Any]
    rows: RowArrays
    bin_pitch_m: float
    row_geometry: str
    georef: Georef
    png_path: Optional[str] = None
    metadata_path: Optional[str] = None

    @property
    def shape(self):
        return (int(self.meta["rows"]), int(self.meta["cols"]))

    @property
    def display_model_ready(self) -> bool:
        return self.meta.get("display_model") is not None

    # ---- construction ----------------------------------------------------------
    @classmethod
    def from_metadata(cls, meta: Mapping[str, Any], db=None, png=None, *,
                      stream_id: Optional[str] = None, png_path=None,
                      metadata_path=None) -> "SeabedImageInput":
        meta = dict(meta)
        for k in ("image_id", "rows", "cols", "bin_pitch_m", "rows_data"):
            if k not in meta:
                raise InputError(f"metadata lacks {k!r}")
        # The metadata's own stream_id (schema >= 5) wins; the caller's is
        # the fallback for older sessions (``session:<folder>``).
        sid = meta.get("stream_id") or stream_id
        if not sid:
            raise InputError("no stream_id (metadata schema < 5 needs one from the caller)")
        image_id = int(meta["image_id"])
        H, W = int(meta["rows"]), int(meta["cols"])
        rows = rows_from_metadata(meta)
        if rows.n != H:
            raise InputError(f"rows_data has {rows.n} rows, metadata says {H}")
        if db is not None:
            db = np.asarray(db, dtype=np.float32)
            if db.shape != (H, W):
                raise InputError(f"pixel array {db.shape} != metadata {(H, W)}")
        if png is not None:
            png = np.asarray(png, dtype=np.uint8)
            if png.shape != (H, W):
                raise InputError(f"png {png.shape} != metadata {(H, W)}")
        return cls(key=f"{sid}/{image_id}", stream_id=str(sid), image_id=image_id,
                   schema=int(meta.get("schema", 0)), db=db, png=png, meta=meta,
                   rows=rows, bin_pitch_m=float(meta["bin_pitch_m"]),
                   row_geometry=str(meta.get("row_geometry", "ping")),
                   georef=Georef.from_metadata(meta),
                   png_path=png_path, metadata_path=metadata_path)

    @classmethod
    def from_wire(cls, envelope: Union[str, Mapping[str, Any]], db=None,
                  png=None) -> "SeabedImageInput":
        """From the ``/sss_ai/seabed_image/metadata`` envelope (+ arrays)."""
        env = json.loads(envelope) if isinstance(envelope, str) else dict(envelope)
        if "metadata" not in env:
            raise InputError("envelope has no 'metadata'")
        meta = env["metadata"]
        sid = env.get("stream_id") or meta.get("stream_id")
        img = cls.from_metadata(meta, db, png, stream_id=sid,
                                png_path=env.get("png_path"),
                                metadata_path=env.get("metadata_path"))
        if env.get("key") and env["key"] != img.key:
            raise InputError(f"envelope key {env['key']!r} != {img.key!r}")
        return img

    @classmethod
    def from_session_files(cls, json_path: Union[str, Path],
                           npz_path: Union[str, Path, None] = None,
                           png_path: Union[str, Path, None] = None, *,
                           stream_id: Optional[str] = None,
                           load_png: bool = False) -> "SeabedImageInput":
        """From a saved image (``metadata/seabed_XXXXX.json`` + its npz).

        The npz's ``intensity_db`` is the array the live path carried; a
        missing npz gives an image without pixels (``db=None``). The PNG is
        decoded only when ``load_png`` (needs OpenCV); its path is kept."""
        json_path = Path(json_path)
        with open(json_path, "r", encoding="utf-8") as f:
            meta = json.load(f)
        db = None
        if npz_path is not None and Path(npz_path).is_file():
            with np.load(npz_path) as z:
                if "intensity_db" in z.files:
                    db = np.asarray(z["intensity_db"], np.float32)
        png = None
        if load_png and png_path is not None and Path(png_path).is_file():
            import cv2  # lazy: the core stays OpenCV-free otherwise
            png = cv2.imread(str(png_path), cv2.IMREAD_GRAYSCALE)
        return cls.from_metadata(
            meta, db, png, stream_id=stream_id,
            png_path=str(png_path) if png_path is not None else None,
            metadata_path=str(json_path))
