"""Dataset-creation mode: write each seabed image as a training sample.

Layout (``<root>/data/Dataset_data/<name>/``, name from MCS
``<YYYY_MM_DD-HH_MM_SS>-<note>``, claimed exclusively, ``-2`` on a clash)::

    manifest.json        blueboat_perception/dataset_manifest/1
    data.yaml            Ultralytics: ``train``/``val`` placeholders and the
                         class names of the label source (``names: {}`` without one)
    images/<stem>.png    the DETECTOR INPUT (decision 6.4, 2026-10-16): the image the
                         configured preprocessor gives the detector -- the fixed
                         ``fixed_gray`` mapping of the raw dB by default, or the GCS
                         display PNG under ``identity`` -- so training and inference
                         see one mapping
    display/<stem>.png   the GCS display PNG, preserved (copied verbatim offline; live,
                         the same OpenCV encoding of the same mono8 array)
    images/<stem>.json   sidecar: SSS-Dataset-Aug-Studio ``AcquisitionMeta`` fields
                         + ``gcs_metadata`` (the full seabed metadata; the Aug Studio
                         ignores unknown keys, ``extra="ignore"``) + ``perception``
                         (incl. ``labels``: every box's detail, when the source has one)
    labels/<stem>.txt    YOLO labels, one per image (empty = no object): in a
                         simulator world, highlight + shadow boxes from the
                         simulator's ground truth (``sim_labels``); empty with
                         ``NullLabelSource``
    raw/<stem>_db.npz    the raw float32 dB (lossless; the PNG is a display transfer)
    ground_truth_bag/    simulation only: the archival ``ros2 bag record`` the
                         launch file starts (the offline CLI re-labels from it)

``stem = <stream_id>_<image_id:05d>`` (file-name safe). Every file is
created exclusively, never overwritten. Nothing is ever written inside a
recording session (CM-7). No train/val split: consecutive images overlap
50 %, so a split is made later by session or site, never by image
(Sethuraman et al. 2024 §4).

**Intensity mapping.** Under ``fixed_gray`` the training image is a linear dB
window of width ``hi − lo`` after the fixed range normalisation, which is exactly
the Aug Studio's ``log`` mapping inverted with ``log_range_db = hi − lo``; the
sidecar declares it (the inverted intensity is range-normalised, the manifest
says so). Under ``identity`` the image is the GCS display model's ``power``
transfer, which ``linear | gamma | log`` cannot express: ``intensity_mapping``
stays unset and Aug Studio intensity-domain families must not be run on such a
dataset (``docs/REQUESTS.md`` R-3). The pixels are native **slant** bins, so
``res_across_m`` (a ground-range resolution) is left unset in both.
"""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from .. import __version__
from .dataroot import claim_dir, dataset_root, is_inside, safe_name
from .image import SeabedImageInput
from .labels import LabelSource, NullLabelSource

MANIFEST_SCHEMA = "blueboat_perception/dataset_manifest/1"
INTENSITY_NOTE = ("gcs_display_power: the GCS display model's power transfer "
                  "(display_model in each sidecar's gcs_metadata); not expressible "
                  "in the Aug Studio's linear|gamma|log, so intensity_mapping is "
                  "left unset -- do not run intensity-domain families "
                  "(BlueBoat-Perception docs/REQUESTS.md R-3)")


class DatasetError(RuntimeError):
    pass


def _utc(t: Optional[float] = None) -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(time.time() if t is None else t))


def _finite(a) -> np.ndarray:
    a = np.asarray(a, np.float64)
    return a[np.isfinite(a)]


def acquisition_meta(img: SeabedImageInput) -> Dict[str, Any]:
    """The Aug Studio ``AcquisitionMeta`` fields this image supports.

    Only fields the metadata determines are set, within the Aug Studio's
    own bounds (``altitude_m >= 0.1``, positive ranges); the rest stay
    unset so its documented fallbacks apply."""
    r = img.rows
    out: Dict[str, Any] = {"layout": "dual"}
    bottom = _finite(r.bottom)
    if bottom.size and float(bottom.mean()) >= 0.1:
        out["altitude_m"] = float(bottom.mean())
    rng = _finite(r.range)
    if rng.size and float(rng.max()) > 0:
        out["slant_range_m"] = float(rng.max())
    if img.row_geometry == "square" and img.bin_pitch_m > 0:
        out["res_along_m"] = float(img.bin_pitch_m)
    else:
        d = _finite(np.abs(np.diff(r.along)))
        if d.size and float(np.median(d)) > 0:
            out["res_along_m"] = float(np.median(d))
        dt = _finite(np.abs(np.diff(r.t)))
        if dt.size and float(np.median(dt)) > 0:
            out["prf_hz"] = float(1.0 / np.median(dt))
    sp = _finite(r.speed)
    if sp.size and float(np.median(sp)) >= 0:
        out["speed_mps"] = float(np.median(sp))
    yaw = _finite(r.yaw)
    if yaw.size:
        mean_yaw = math.atan2(float(np.mean(np.sin(yaw))), float(np.mean(np.cos(yaw))))
        out["heading_deg"] = float((90.0 - math.degrees(mean_yaw)) % 360.0)  # compass
    wd = _finite(r.altitude)
    if wd.size and float(wd.mean()) > 0:
        out["depth_m"] = float(wd.mean())
    t0 = float(img.meta.get("t_start_s", float("nan")))
    if math.isfinite(t0) and t0 > 1e9:              # wall-clock ROS time, not sim time
        out["timestamp"] = time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(t0)) + \
            f".{int((t0 % 1) * 1000):03d}Z"
    return out


class DatasetWriter:
    def __init__(self, directory: Path, *, name: str, mode: str = "dataset",
                 world_dir: str = "", config: Optional[dict] = None,
                 label_source: Optional[LabelSource] = None,
                 forbidden_roots: Sequence[Path] = (),
                 detector_input: Optional[dict] = None) -> None:
        self.dir = Path(directory)
        for root in forbidden_roots:
            if is_inside(self.dir, root):
                raise DatasetError(f"refusing to write a dataset inside {root} "
                                   f"(recording sessions are write-once, CM-7)")
        self.name = name
        self.mode = mode
        self.world_dir = world_dir
        self.config = dict(config or {})
        self.labels = label_source or NullLabelSource()
        #: {"name", "params", "log_range_db"?} of the preprocessor whose output
        #: the training images are (None: the display PNG, as before 6.4)
        self.detector_input = dict(detector_input) if detector_input else None
        self.stream_ids: List[str] = []
        self.count = 0
        self.skipped_no_png = 0
        self.created = _utc()
        for sub in ("images", "display", "labels", "raw"):
            (self.dir / sub).mkdir(parents=True, exist_ok=True)
        self._write_new(self.dir / "data.yaml", self._data_yaml())
        self.write_manifest()

    @classmethod
    def create(cls, root: Path, name: str, **kw) -> "DatasetWriter":
        """Claim ``<root>/data/Dataset_data/<name>`` (``-2`` ... on a clash)."""
        if not name:
            raise DatasetError("dataset mode needs a dataset_name")
        directory = claim_dir(dataset_root(root), safe_name(name))
        return cls(directory, name=directory.name, **kw)

    # ---- per image ----------------------------------------------------------------
    def stem(self, img: SeabedImageInput) -> str:
        return f"{safe_name(img.stream_id)}_{img.image_id:05d}"

    @staticmethod
    def _encode(arr) -> bytes:
        import cv2  # lazy: the core stays OpenCV-free otherwise
        ok, buf = cv2.imencode(".png", arr)
        if not ok:
            raise DatasetError("PNG encoding failed")
        return buf.tobytes()

    def _display_bytes(self, img: SeabedImageInput) -> Optional[bytes]:
        if img.png is not None:
            return self._encode(img.png)
        if img.png_path and Path(img.png_path).is_file():
            with open(img.png_path, "rb") as src:
                return src.read()
        return None

    def write(self, img: SeabedImageInput, record: Optional[dict] = None,
              inp=None) -> Optional[str]:
        """Write one image; returns its stem, or None when it has no pixels or
        no training image can be made (counted, never invented).

        ``inp`` is the pipeline's ``DetectorInput``: its ``gray`` is the training
        image. Without one (or without ``gray``), the display PNG is used, as
        before decision 6.4."""
        if img.db is None:
            return None
        stem = self.stem(img)
        display = self._display_bytes(img)
        gray = None if inp is None else getattr(inp, "gray", None)
        if gray is not None:
            train = self._encode(np.asarray(gray, np.uint8))
        elif display is not None:
            train = display
        else:
            self.skipped_no_png += 1
            self.write_manifest()
            return None
        self._write_new(self.dir / "images" / f"{stem}.png", train)
        if display is not None:
            self._write_new(self.dir / "display" / f"{stem}.png", display)
        sidecar = acquisition_meta(img)
        di = self.detector_input if gray is not None else None
        if di is not None and di.get("log_range_db"):
            sidecar["intensity_mapping"] = "log"
            sidecar["log_range_db"] = float(di["log_range_db"])
        sidecar["gcs_metadata"] = img.meta
        sidecar["perception"] = {"key": img.key, "stream_id": img.stream_id,
                                 "image_id": img.image_id,
                                 "training_image": ("detector input" if gray is not None
                                                    else "GCS display PNG"),
                                 "detector_input": di,
                                 "intensity_mapping": (self._mapping_note()
                                                       if gray is not None else INTENSITY_NOTE),
                                 "pixel_domain": "native slant bins of bin_pitch_m "
                                                 "(gcs_metadata.pixel_convention)"}
        if record is not None:
            sidecar["perception"]["record_key"] = record.get("key")
        lines = [b.line() for b in self.labels.labels_for(img)]
        detail_for = getattr(self.labels, "detail_for", None)
        if detail_for is not None:
            sidecar["perception"]["labels"] = detail_for(img)
        self._write_new(self.dir / "images" / f"{stem}.json",
                        json.dumps(sidecar, indent=1, sort_keys=True, allow_nan=True))
        self._write_new(self.dir / "labels" / f"{stem}.txt",
                        "".join(f"{ln}\n" for ln in lines))
        raw = self.dir / "raw" / f"{stem}_db.npz"
        with open(raw, "xb") as f:
            np.savez_compressed(f, db=np.asarray(img.db, np.float32))
        if img.stream_id not in self.stream_ids:
            self.stream_ids.append(img.stream_id)
        self.count += 1
        self.write_manifest()
        return stem

    # ---- dataset-level files -------------------------------------------------------
    def _data_yaml(self) -> str:
        names = self.labels.classes
        head = ("# BlueBoat-Perception dataset. No train/val split: split by session\n"
                "# or site later, never by image (consecutive images overlap 50 %).\n")
        if not names:
            return head + "names: {}\n"
        return (head
                + "# train/val (relative to this file's folder) point at the whole set\n"
                + "# until you split it.\n"
                + "train: images\nval: images\n"
                + f"nc: {len(names)}\nnames:\n"
                + "".join(f"  {k}: {v}\n" for k, v in sorted(names.items())))

    def manifest(self) -> Dict[str, Any]:
        return {"schema": MANIFEST_SCHEMA, "name": self.name,
                "perception_version": __version__, "mode": self.mode,
                "created_utc": self.created, "updated_utc": _utc(),
                "world_dir": self.world_dir or None,
                "stream_ids": list(self.stream_ids), "image_count": self.count,
                "skipped_no_png": self.skipped_no_png,
                "label_source": self.labels.name,
                "label_source_info": (self.labels.info() if hasattr(self.labels, "info")
                                      else None),
                "labels": (self.labels.stats() if hasattr(self.labels, "stats")
                           else None),
                "training_images": ("detector input" if self.detector_input
                                    else "GCS display PNG"),
                "detector_input": self.detector_input,
                "intensity_mapping": self._mapping_note(),
                "pixel_domain": "native slant bins (GCS seabed images); "
                                "res_across_m deliberately unset",
                "ground_truth_bag": ("ground_truth_bag" if (self.dir / "ground_truth_bag").exists()
                                     else None),
                "splits": "none (split by session or site, never by image)",
                "config": self.config}

    def _mapping_note(self) -> str:
        di = self.detector_input
        if di and di.get("log_range_db"):
            return (f"log, log_range_db {float(di['log_range_db']):.4f}: a linear dB window "
                    f"after the fixed range normalisation of {di.get('name')} "
                    f"(the Aug Studio's log mapping inverts it exactly; the recovered "
                    f"intensity is range-normalised)")
        if di:
            return INTENSITY_NOTE + " (identity: the training image is the display PNG)"
        return INTENSITY_NOTE

    def write_manifest(self) -> None:
        tmp = self.dir / ".manifest.json.tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.manifest(), f, indent=1, sort_keys=True)
        os.replace(tmp, self.dir / "manifest.json")

    def close(self) -> None:
        self.write_manifest()

    @staticmethod
    def _write_new(path: Path, data) -> None:
        mode = "xb" if isinstance(data, (bytes, bytearray)) else "x"
        kw = {} if "b" in mode else {"encoding": "utf-8"}
        with open(path, mode, **kw) as f:
            f.write(data)
