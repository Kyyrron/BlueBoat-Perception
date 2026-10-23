"""Shared fixtures for the BlueBoat-Perception suite (ROS-free core).

Run from ``BlueBoat-Perception/blueboat_perception/`` with the system
python: ``PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest``.

* Corpus tests read the GCS sessions under ``$BLUEBOAT_DATA_DIR/data/SSS_data``
  or ``~/ros2_ws/data/SSS_data`` and skip cleanly without them. They only
  ever read (CM-7).
* "Imports without ROS" is simulated by blocking the ROS / Qt / OpenCV
  modules on ``sys.meta_path`` (the development machine sources ROS 2
  globally, so merely observing would test nothing).
* ROS node tests are gated with ``find_spec`` + ``skipif``, never a
  module-level ``importorskip`` (which aborts collection on pytest 7.4).
"""

from __future__ import annotations

import importlib
import json
import math
import os
import sys
from pathlib import Path
from typing import Iterator, List, Optional

import numpy as np
import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]          # the colcon package root
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))
PY_PKG = PKG_ROOT / "blueboat_perception"

BLOCKED_PREFIXES = ("rclpy", "rcl_interfaces", "sensor_msgs", "std_msgs", "vision_msgs",
                    "builtin_interfaces", "launch", "launch_ros", "ament_index_python",
                    "PySide6", "PyQt5", "cv2", "torch", "torchvision", "ultralytics")


def sss_data_dir() -> Optional[Path]:
    env = os.environ.get("BLUEBOAT_DATA_DIR")
    for cand in ([Path(env) / "data" / "SSS_data", Path(env) / "SSS_data"] if env else []) + \
            [Path.home() / "ros2_ws" / "data" / "SSS_data"]:
        if cand.is_dir():
            return cand
    return None


def corpus_image_dirs() -> List[Path]:
    root = sss_data_dir()
    if root is None:
        return []
    return sorted(p for p in root.rglob("seabed_images*")
                  if p.is_dir() and (p / "metadata").is_dir())


class _Blocker:
    def __init__(self, prefixes):
        self.prefixes = prefixes

    def find_spec(self, name, path=None, target=None):
        if any(name == p or name.startswith(p + ".") for p in self.prefixes):
            raise ImportError(f"blocked for the import sweep: {name}")
        return None


@pytest.fixture
def no_ros_qt_cv() -> Iterator[None]:
    """Make ROS, Qt and OpenCV unimportable; restore afterwards."""
    saved = {k: v for k, v in sys.modules.items()
             if any(k == p or k.startswith(p + ".") for p in BLOCKED_PREFIXES)
             or k.startswith("blueboat_perception")}
    for k in saved:
        del sys.modules[k]
    blocker = _Blocker(BLOCKED_PREFIXES)
    sys.meta_path.insert(0, blocker)
    try:
        yield
    finally:
        sys.meta_path.remove(blocker)
        for k in [k for k in sys.modules if k.startswith("blueboat_perception")]:
            del sys.modules[k]
        sys.modules.update(saved)
        importlib.invalidate_caches()


# ---------------------------------------------------------------------------
# Synthetic GCS images (no SSS import needed)
# ---------------------------------------------------------------------------
def make_metadata(H: int = 32, W: int = 40, pitch: float = 0.05, *, image_id: int = 0,
                  stream_id: Optional[str] = "gcs-20261016T000000Z-abcd",
                  schema: int = 5, convention: Optional[int] = 2,
                  geometry: str = "square", attitude: bool = True,
                  t0: float = 100.0, yaw0: float = 0.3, yaw_rate: float = 0.05,
                  altitude: float = 0.6, detections=None) -> dict:
    rows = []
    for i in range(H):                       # row 0 = newest
        t = t0 - 0.05 * i
        yaw = yaw0 + yaw_rate * (t - t0)
        bottom = altitude + 0.02 * math.sin(i / 3.0)
        r = {"t_s": t, "x_m": 2.0 + 0.6 * (t - t0) * math.cos(yaw),
             "y_m": -1.0 + 0.6 * (t - t0) * math.sin(yaw), "yaw_rad": yaw,
             "range_m": (W // 2) * pitch, "speed_mps": 0.6,
             "altitude_m": bottom + 0.15, "bottom_m": bottom,
             "ping_index": i, "along_track_m": (H - 1 - i) * pitch,
             "gain_port_db": 0.0, "gain_stbd_db": 0.0}
        if attitude:
            r.update({"roll_rad": 0.01 * math.sin(i), "pitch_rad": -0.005,
                      "yaw_rate_rad_s": yaw_rate, "attitude_age_s": 0.02,
                      "port_ping_number": 1000 + i, "stbd_ping_number": 1060 + i})
        rows.append(r)
    meta = {"schema": schema, "image_id": image_id, "rows": H, "cols": W,
            "t_start_s": rows[0]["t_s"], "t_end_s": rows[-1]["t_s"],
            "bin_pitch_m": pitch, "row_geometry": geometry,
            "display_model": {"transfer": "power"}, "rows_data": rows,
            "boat": {}, "detections": detections if detections is not None else []}
    if stream_id is not None:
        meta["stream_id"] = stream_id
    if convention == 2:
        meta["georef"] = {"convention": 2, "altitude": "bottom_m",
                          "lever_arm_m": {"x": 0.0, "y_port": 0.285, "y_stbd": 0.285}}
    return meta


def make_db(H: int = 32, W: int = 40, seed: int = 0, *, nan_side: Optional[str] = None,
            nan_rows=()) -> np.ndarray:
    rng = np.random.default_rng(seed)
    db = (30.0 + 5.0 * rng.standard_normal((H, W))).astype(np.float32)
    db[:, W // 2 - 2: W // 2 + 2] = np.nan               # a little nadir blank
    for i in nan_rows:
        if nan_side == "starboard":
            db[i, W // 2:] = np.nan
        elif nan_side == "port":
            db[i, : W // 2] = np.nan
    return db


def envelope(meta: dict) -> str:
    sid = meta.get("stream_id")
    return json.dumps({"schema": "blueboat_sss/seabed_image/1",
                       "key": f"{sid}/{meta['image_id']}", "stream_id": sid,
                       "image_id": meta["image_id"],
                       "stamp": {"sec": int(meta["t_start_s"]), "nanosec": 0},
                       "png_path": None, "metadata_path": None, "metadata": meta})


def write_session(folder: Path, metas, dbs, *, with_npz=True, with_png=True) -> Path:
    """A GCS-shaped ``seabed_images/`` folder (json indent=1 as ``save()``)."""
    d = folder / "seabed_images"
    (d / "metadata").mkdir(parents=True, exist_ok=True)
    for meta, db in zip(metas, dbs):
        stem = f"seabed_{meta['image_id']:05d}"
        with open(d / "metadata" / f"{stem}.json", "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=1)
        if with_npz:
            np.savez_compressed(d / "metadata" / f"{stem}_world.npz", intensity_db=db,
                                world_x=np.zeros_like(db), world_y=np.zeros_like(db))
        if with_png:
            png = np.nan_to_num(np.clip(db, 0, 60) * 4).astype(np.uint8)
            try:
                import cv2
                cv2.imwrite(str(d / f"{stem}.png"), png)
            except ImportError:                                  # pragma: no cover
                (d / f"{stem}.png").write_bytes(b"")
    return folder


@pytest.fixture
def synthetic_session(tmp_path) -> Path:
    metas = [make_metadata(image_id=i, t0=100.0 + 1.6 * i) for i in range(4)]
    dbs = [make_db(seed=i) for i in range(4)]
    return write_session(tmp_path / "sessions" / "2026_10_16-10_00_00", metas, dbs)
