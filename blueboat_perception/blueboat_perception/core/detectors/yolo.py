"""YOLO detector (Ultralytics, PyTorch) over across-track tiles.

The picture is 256 rows × W slant bins (W up to 2402); a YOLO model takes a
square input, so the adapter **tiles across track** at native resolution
(``core/tiling.py``), predicts every tile of a picture in one batch at
``input_size`` 256, shifts each tile's boxes back into picture pixels,
merges the duplicates of overlapping tiles (``core/merge.py``) and returns
one list of ``RawDetection`` per picture -- what the rest of the pipeline
(calibration, georeferencing, turn flags, uncertainty, records) takes
unchanged. ``tiling="full"`` instead letterboxes the whole strip into one
``input_size`` square (640): the squeeze baseline of the comparison.

Constraints (Redmon et al. 2016, R10): YOLO resizes its input to a fixed
square (fig. 1) -- hence the tiles; its score is ``Pr(Object)·IOU`` or
``Pr(Class)·IOU`` (§2 eq. 1), **not a calibrated probability** -- hence
the calibration stage; it is weaker on small objects (§2.4).

**Optional runtime.** ``ultralytics`` and ``torch`` are imported only when
a ``YoloDetector`` is built with the default runtime, never at module
import, so the core, the dummy detector and the tests run without them.
Tests inject a ``runtime`` (any object with ``predict`` and ``info``).

**Determinism (offline = live, non-negotiable 3).** Fixed seed,
``torch.use_deterministic_algorithms`` (warn-only), one batch per picture
in tile order, CPU by default. Box edges are rounded to 1e-3 px and scores
to 1e-6 before merging, and the pipeline takes inclusive integer pixel
boxes; the same picture and the same weights on the same machine give the
same record. Inference on a GPU, or on another CPU, may differ in the last
bits of a score; the record states the device.

The record names the detector, the weights file and its SHA-256, the model
family (from the ``<weights>.json`` that ``tools/train_yolo.py`` writes
beside it), the library versions, the tiling and the merge rule
(``params``), and the per-picture tile counters (``last_stats``).
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from ..merge import DEFAULT_METRIC, MERGES, Box, merge, ordered
from ..preprocess import DetectorInput
from ..tiling import TilingConfig, cut_tiles, tile_box_to_picture, to_inclusive_px
from .base import RawDetection

TILINGS = ("tiles", "full")
#: Default input size per tiling mode (tiles at native resolution; the
#: squeeze baseline at Ultralytics' default 640).
DEFAULT_INPUT_SIZE = {"tiles": 256, "full": 640}


def file_sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def model_card(model_path: str) -> Optional[Dict[str, Any]]:
    """The ``<weights>.json`` written next to the weights by
    ``tools/train_yolo.py`` (None when absent)."""
    p = Path(model_path)
    for cand in (p.with_suffix(".json"), p.parent.parent / "model.json",
                 p.parent / "model.json"):
        if cand.is_file():
            try:
                return json.loads(cand.read_text(encoding="utf-8"))
            except ValueError:
                return None
    return None


class UltralyticsRuntime:
    """The default runtime: an Ultralytics ``YOLO`` model (lazy import)."""

    def __init__(self, model_path: str, device: str = "cpu", seed: int = 0) -> None:
        import torch                       # lazy: optional dependency
        import ultralytics
        from ultralytics import YOLO
        torch.manual_seed(int(seed))
        try:
            torch.use_deterministic_algorithms(True, warn_only=True)
        except (AttributeError, TypeError):           # pragma: no cover - old torch
            pass
        self._torch = torch
        self.model = YOLO(str(model_path), task="detect")
        self.device = str(device)
        names = self.model.names
        self.names = ({int(k): str(v) for k, v in names.items()} if isinstance(names, dict)
                      else {i: str(v) for i, v in enumerate(names)})
        yaml = getattr(getattr(self.model, "model", None), "yaml", None) or {}
        self.channels = int(yaml.get("channels", 3) or 3)
        self.versions = {"ultralytics": ultralytics.__version__, "torch": torch.__version__}
        self.architecture = str(yaml.get("yaml_file", "") or "")

    def info(self) -> Dict[str, Any]:
        return {"runtime": "ultralytics", **self.versions, "channels": self.channels,
                "architecture": self.architecture, "names": dict(self.names)}

    def _prep(self, img: np.ndarray) -> np.ndarray:
        if self.channels == 1:
            return img[:, :, None]
        return np.repeat(img[:, :, None], 3, axis=2)       # grey copied to 3 channels

    def predict(self, images: Sequence[np.ndarray], imgsz: int, conf: float, iou: float,
                max_det: int) -> List[np.ndarray]:
        """(N, 6) arrays ``[x0, y0, x1, y1, score, class]`` in each input's
        own pixels, one per image."""
        if not images:
            return []
        with self._torch.inference_mode():
            res = self.model.predict(source=[self._prep(np.ascontiguousarray(i))
                                             for i in images],
                                     imgsz=int(imgsz), conf=float(conf), iou=float(iou),
                                     max_det=int(max_det), device=self.device,
                                     batch=len(images), verbose=False)
        out = []
        for r in res:
            b = r.boxes
            if b is None or len(b) == 0:
                out.append(np.zeros((0, 6), np.float64))
                continue
            out.append(np.concatenate([b.xyxy.cpu().numpy().astype(np.float64),
                                       b.conf.cpu().numpy().astype(np.float64)[:, None],
                                       b.cls.cpu().numpy().astype(np.float64)[:, None]],
                                      axis=1))
        return out


class YoloDetector:
    name = "yolo"
    version = "1"
    is_dummy = False

    def __init__(self, model_path: Optional[str] = None, input_size: int = 0,
                 tile_stride: int = 128, score_threshold: float = 0.01,
                 nms_iou: float = 0.7, *, tiling: str = "tiles", tile_size: int = 256,
                 min_valid_fraction: float = 0.001, merge: str = "nms",
                 merge_threshold: float = 0.5, merge_metric: str = "",
                 max_det_tile: int = 300, device: str = "cpu", seed: int = 0,
                 runtime: Any = None) -> None:
        if tiling not in TILINGS:
            raise ValueError(f"tiling must be one of {TILINGS}, got {tiling!r}")
        if merge not in MERGES:
            raise ValueError(f"merge must be one of {sorted(MERGES)}, got {merge!r}")
        self.tiling = tiling
        self.cfg = TilingConfig(tile=int(tile_size), stride=int(tile_stride),
                                min_valid_fraction=float(min_valid_fraction))
        self.input_size = int(input_size) or DEFAULT_INPUT_SIZE[tiling]
        self.score_threshold = float(score_threshold)
        self.nms_iou = float(nms_iou)
        self.merge_rule = merge
        self.merge_threshold = float(merge_threshold)
        self.merge_metric = merge_metric or DEFAULT_METRIC[merge]
        self.max_det_tile = int(max_det_tile)
        self.device = str(device)
        self.seed = int(seed)
        self.model_path = model_path
        model_info: Dict[str, Any] = {"file": None, "sha256": None, "card": None}
        if runtime is None:
            if not model_path:
                raise ValueError("the yolo detector needs a weights file (detector_model)")
            if not Path(model_path).is_file():
                raise FileNotFoundError(f"YOLO weights not found: {model_path}")
            runtime = UltralyticsRuntime(model_path, self.device, self.seed)
        if model_path and Path(model_path).is_file():
            card = model_card(model_path)
            ds = (card or {}).get("dataset") or {}
            model_info = {"file": Path(model_path).name, "sha256": file_sha256(model_path),
                          "card": (None if card is None else
                                   {"name": card.get("name"), "family": card.get("family"),
                                    "model": card.get("model"), "seed": card.get("seed"),
                                    "created_utc": card.get("created_utc"),
                                    "data": card.get("data"),
                                    "dataset": ({"name": ds.get("name"),
                                                 "manifest_sha256": ds.get("manifest_sha256")}
                                                if isinstance(ds, dict) else ds)})}
        self.runtime = runtime
        rinfo = dict(runtime.info())
        self.names: Dict[int, str] = {int(k): str(v) for k, v in
                                      (rinfo.pop("names", None) or {}).items()}
        self._params = {
            "model": model_info, "runtime": rinfo, "device": self.device, "seed": self.seed,
            "tiling": {"mode": self.tiling, **self.cfg.to_json()},
            "input_size": self.input_size, "score_threshold": self.score_threshold,
            "nms_iou": self.nms_iou, "max_det_tile": self.max_det_tile,
            "merge": {"rule": self.merge_rule, "threshold": self.merge_threshold,
                      "metric": self.merge_metric, "score": "max"},
            "rounding": {"box_px": 1e-3, "score": 1e-6},
        }
        self.last_stats: Dict[str, Any] = {}

    @property
    def params(self) -> Dict[str, Any]:
        return json.loads(json.dumps(self._params))

    # ---- detection -----------------------------------------------------------------
    @staticmethod
    def _round(a: np.ndarray) -> np.ndarray:
        a = np.asarray(a, np.float64).reshape(-1, 6).copy()
        a[:, :4] = np.round(a[:, :4], 3)
        a[:, 4] = np.round(a[:, 4], 6)
        return a

    def _boxes_tiles(self, gray: np.ndarray, valid: Optional[np.ndarray], stats: dict):
        tiles = cut_tiles(gray, valid, self.cfg)
        run = [t for t in tiles if not t.skipped]
        stats.update(tiles=len(tiles), tiles_skipped=len(tiles) - len(run))
        preds = self.runtime.predict([t.gray for t in run], self.input_size,
                                     self.score_threshold, self.nms_iou, self.max_det_tile)
        boxes: List[Box] = []
        in_padding = 0
        for t, p in zip(run, preds):
            for x0, y0, x1, y1, score, cls in self._round(p):
                pic = tile_box_to_picture((x0, y0, x1, y1), t)
                if pic is None:
                    in_padding += 1
                    continue
                boxes.append(Box(*pic, score=float(score), class_id=int(cls), tile=t.index))
        stats["boxes_in_padding"] = in_padding
        return boxes

    def _boxes_full(self, gray: np.ndarray, stats: dict):
        H, W = gray.shape
        stats.update(tiles=1, tiles_skipped=0, boxes_in_padding=0)
        (p,) = self.runtime.predict([gray], self.input_size, self.score_threshold,
                                    self.nms_iou, self.max_det_tile) or [np.zeros((0, 6))]
        boxes = []
        for x0, y0, x1, y1, score, cls in self._round(p):
            x0, x1 = max(x0, 0.0), min(x1, float(W))
            y0, y1 = max(y0, 0.0), min(y1, float(H))
            if x1 > x0 and y1 > y0:
                boxes.append(Box(x0, y0, x1, y1, float(score), int(cls), 0))
        return boxes

    def detect(self, inp: DetectorInput) -> List[RawDetection]:
        stats: Dict[str, Any] = {"ran": inp.gray is not None, "tiles": 0, "tiles_skipped": 0,
                                 "boxes_in_padding": 0, "boxes_before_merge": 0,
                                 "boxes_after_merge": 0}
        self.last_stats = stats
        if inp.gray is None:
            return []
        gray = np.asarray(inp.gray, np.uint8)
        H, W = gray.shape
        boxes = (self._boxes_tiles(gray, inp.valid, stats) if self.tiling == "tiles"
                 else self._boxes_full(gray, stats))
        stats["boxes_before_merge"] = len(boxes)
        # one input in "full" mode: Ultralytics' own NMS already ran, only order
        merged = (merge(boxes, self.merge_rule, self.merge_threshold, self.merge_metric)
                  if self.tiling == "tiles" else ordered(boxes))
        stats["boxes_after_merge"] = len(merged)
        out = []
        for b in merged:
            r0, c0, r1, c1 = to_inclusive_px((b.x0, b.y0, b.x1, b.y1), H, W)
            out.append(RawDetection(bbox_px=(r0, c0, r1, c1),
                                    center_px=((r0 + r1) // 2, (c0 + c1) // 2),
                                    class_id=int(b.class_id),
                                    class_name=self.names.get(int(b.class_id),
                                                              f"class_{int(b.class_id)}"),
                                    score_raw=float(round(b.score, 6))))
        return out


#: Node parameter / CLI option -> ``YoloDetector`` keyword.
PARAM_MAP = {
    "detector_model": "model_path", "detector_device": "device",
    "detector_tiling": "tiling", "tile_size": "tile_size", "tile_stride": "tile_stride",
    "tile_min_valid_fraction": "min_valid_fraction",
    "detector_input_size": "input_size",
    "detector_score_threshold": "score_threshold", "detector_nms_iou": "nms_iou",
    "merge": "merge", "merge_threshold": "merge_threshold",
    "detector_max_det_tile": "max_det_tile",
}
#: Their defaults (also ``config/perception.yaml`` and ``docs/topics.md`` §6).
PARAM_DEFAULTS = {
    "detector_model": "", "detector_device": "cpu", "detector_tiling": "tiles",
    "tile_size": 256, "tile_stride": 128, "tile_min_valid_fraction": 0.001,
    "detector_input_size": 0, "detector_score_threshold": 0.01, "detector_nms_iou": 0.7,
    "merge": "nms", "merge_threshold": 0.5, "detector_max_det_tile": 300,
}


def detector_args(detector: str, params: Dict[str, Any]) -> Dict[str, Any]:
    """``detector_args`` for ``PipelineConfig`` from node parameters / CLI
    options: the YOLO keywords for ``yolo``, nothing for the dummy."""
    if detector != YoloDetector.name:
        return {}
    return {kw: params.get(p, PARAM_DEFAULTS[p]) for p, kw in PARAM_MAP.items()}
