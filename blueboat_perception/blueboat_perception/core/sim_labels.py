"""YOLO labels from the simulator's ground truth, for dataset mode only.

**Scope (owner's decision, 2026-10-16).** In dataset mode, in a simulator world,
the Perception node passes the simulator's per-ping ground truth to this module.
This module feeds only the dataset label writer. The detection path (pipeline,
detectors, preprocessing, calibration, records) never imports it
(test-enforced), so detections and records are the same with or without it. This
module names no topic: the ROS side parses the messages and hands over plain
dicts, and the offline CLI reads the same dicts from the dataset's archival bag.

**Inputs**
- **The world folder:**
  - objects from ``metadata.yaml`` (the World Builder's contract), else
    ``scene_manifest.yaml``: ``object_id, type, x, y, yaw, length, width,
    proud_height, burial``;
  - the seabed height from ``depth.npz`` (terrain only), else ``scene.npz``
    (objects stamped in, so it is read around the object, not under it).
- **Every ping cycle, the transducer pose each side was rendered from:**
  ``{"t_sim", "stamp", "sensors": {"port"|"starboard": {"pose": [x, y, z, roll,
  pitch, yaw], "altitude_m", "ping_number", "dropped"}}}``.
- **Every ping cycle with contacts, the renderer's contacts:** ``{"t_sim",
  "contacts": [{"side", "object_id", "type", "slant_range_m", "extent_bins",
  "shadow_bins", "visible", "ping_number", "ghost", "via"}]}``. They say which
  objects a ping saw, whether they are visible, and the shadow length the
  renderer read off its visibility mask.

**Geometry.** Per picture row and side, the simulator renderer's own model
(BlueBoat-SSS-Sim ``sonar/renderer.py``):
- **Joining a row to its ping.** The row's source ping is ``(side,
  ping_number)``, the device counter the GCS stores per row. The sensor entry
  with the nearest ``stamp`` to the row time is taken, which resolves counter
  restarts.
- **The beam strip.** The ping sees ground points at along-track offsets
  weighted by a Gaussian of ``σ = R·θ/2.355``, θ the 0.5° horizontal aperture,
  integrated over lines within ``±R_max·θ/2``. Here that is the strip ``|along|
  <= min(3σ, R_max·θ/2)``. The object's oriented footprint (``L × W`` at its
  ``yaw``) is clipped to the strip, which gives the across-track interval
  ``[a_min, a_max]`` this ping sees. An empty clip means no echo in this row.
- **Heights:** the sensor is ``dz = z_sensor − z_seabed`` above the seabed, and
  the object top is ``H = proud·(1 − burial)`` above it.
- **Highlight:** from the nearest top point ``hypot(a_min, dz − H)`` to the
  farther of ``hypot(a_max, dz − H)`` (far top) and ``hypot(a_min, dz)`` (the
  foot of the near face).
- **Shadow end:** the larger of two estimates:
  - the flat-floor tip ``hypot(a_max·dz/(dz − H), dz)``;
  - the renderer's mask length, ``hypot(a_c + ha, dz) + shadow_bins·bin``.
- **Range envelope:** both ends widen by ``range_margin_bins``, the half-width
  of the renderer's boxcar (5 bins at the default acquisition).
- **Bins:** the device's own, ``[k·L/N, (k + 1)·L/N)``. The GCS's
  ``bin_pitch_m`` is ``L/(N − 1)``, so ``L/N = bin_pitch_m·(N − 1)/N``.
- **Columns:** port ``j = W/2 − 1 − k``, starboard ``j = W/2 + k``.
- **Boxes:** one per object, side and contiguous run of rows: the union of the
  rows' intervals.
- **Not labelled:** multipath ghosts (``ghost`` / ``via``), wall echoes, and
  pings where the object is not visible. Each is counted.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np

from .image import SeabedImageInput
from .labels import YoloBox

LABEL_SOURCE_NAME = "sim_ground_truth/1"
DATA_DIR = Path(__file__).resolve().parent / "data"
DEFAULT_CLASS_MAP = "yolo_classes_v1"
#: Contacts and the sensor message of one ping cycle carry the same ``t_sim``,
#: rounded to 3 and 4 decimals; a ping period is >= 50 ms.
TICK_MATCH_S = 0.02
SIDES = (("port", +1), ("starboard", -1))
#: Omniscan 450 SS horizontal aperture (the simulator's
#: ``horizontal_aperture_deg``, 0.5 in config/default_sonar.yaml).
DEFAULT_BEAM_DEG = 0.5
#: Half-width of the renderer's range envelope at the default acquisition:
#: ``round(c·τ/2/bin + 0.0015·N + 1) = 5`` bins (τ = 44.3 µs, 1200 bins, 20 m).
DEFAULT_RANGE_MARGIN_BINS = 2
#: Half-width of the beam strip in Gaussian sigmas of the along-track beam.
DEFAULT_STRIP_SIGMAS = 3.0


# ----------------------------------------------------------------------- world
@dataclass(frozen=True)
class WorldObject:
    object_id: int
    type: str
    x: float
    y: float
    yaw: float
    length: float
    width: float
    height: float          # effective height above the seabed: proud · (1 − burial)

    @property
    def radius(self) -> float:
        return 0.5 * math.hypot(self.length, self.width)


@dataclass(frozen=True)
class Terrain:
    """A height raster of the world (z up, seabed negative), cell centres at
    ``origin + index·resolution``. ``stamped`` = objects are in it."""

    height: np.ndarray
    origin_x: float
    origin_y: float
    resolution: float
    stamped: bool
    source: str

    def _sample(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        ny, nx = self.height.shape
        fx = np.clip((x - self.origin_x) / self.resolution, 0, nx - 1)
        fy = np.clip((y - self.origin_y) / self.resolution, 0, ny - 1)
        x0, y0 = np.floor(fx).astype(int), np.floor(fy).astype(int)
        x1, y1 = np.minimum(x0 + 1, nx - 1), np.minimum(y0 + 1, ny - 1)
        tx, ty = fx - x0, fy - y0
        h = self.height
        return ((h[y0, x0] * (1 - tx) + h[y0, x1] * tx) * (1 - ty)
                + (h[y1, x0] * (1 - tx) + h[y1, x1] * tx) * ty)

    def seabed_z(self, o: WorldObject) -> float:
        """Seabed height under the object, without the object."""
        if not self.stamped:
            return float(self._sample(np.array([o.x]), np.array([o.y]))[0])
        r0 = o.radius + max(self.resolution, 0.1)
        ang = np.linspace(0.0, 2.0 * math.pi, 48, endpoint=False)
        rr = np.repeat([r0, r0 + 0.2, r0 + 0.4], ang.size)
        aa = np.tile(ang, 3)
        z = self._sample(o.x + rr * np.cos(aa), o.y + rr * np.sin(aa))
        return float(np.median(z))


@dataclass(frozen=True)
class WorldInfo:
    objects: Dict[int, WorldObject]
    path: str
    sha256: str
    terrain: Optional[Terrain] = None


def objects_from_world_doc(doc: Dict[str, Any]) -> Dict[int, WorldObject]:
    """The objects of a parsed ``metadata.yaml`` / ``scene_manifest.yaml``."""
    out: Dict[int, WorldObject] = {}
    for o in (doc or {}).get("objects") or []:
        oid = int(o["object_id"])
        proud = float(o.get("proud_height", 0.0))
        burial = float(o.get("burial", 0.0))
        out[oid] = WorldObject(object_id=oid, type=str(o["type"]),
                               x=float(o["x"]), y=float(o["y"]),
                               yaw=float(o.get("yaw", 0.0)),
                               length=float(o.get("length", 0.0)),
                               width=float(o.get("width", 0.0)),
                               height=max(0.0, proud * (1.0 - burial)))
    return out


def load_terrain(world_dir: str) -> Optional[Terrain]:
    """``depth.npz`` (terrain only), else ``scene.npz`` + its manifest grid."""
    d = Path(world_dir).expanduser()
    p = d / "depth.npz"
    if p.is_file():
        z = np.load(p)
        return Terrain(np.asarray(z["height"], np.float64), float(z["origin_x"]),
                       float(z["origin_y"]), float(z["resolution"]), False, str(p))
    p, m = d / "scene.npz", d / "scene_manifest.yaml"
    if p.is_file() and m.is_file():
        import yaml  # lazy, see load_world
        g = (yaml.safe_load(m.read_text(encoding="utf-8")) or {}).get("grid") or {}
        z = np.load(p)
        return Terrain(np.asarray(z["height"], np.float64), float(g["origin_x"]),
                       float(g["origin_y"]), float(g["resolution"]), True, str(p))
    return None


def load_world(world_dir: str) -> WorldInfo:
    """Objects (``metadata.yaml``, else ``scene_manifest.yaml``) and terrain of a
    world folder. PyYAML is imported here only, as OpenCV is elsewhere in the
    core."""
    import yaml  # lazy: the core imports with the standard library and numpy
    d = Path(world_dir).expanduser()
    for name in ("metadata.yaml", "scene_manifest.yaml"):
        p = d / name
        if p.is_file():
            raw = p.read_bytes()
            doc = yaml.safe_load(raw.decode("utf-8")) or {}
            if not doc.get("objects") and name == "metadata.yaml":
                continue
            return WorldInfo(objects=objects_from_world_doc(doc), path=str(p),
                             sha256=hashlib.sha256(raw).hexdigest(),
                             terrain=load_terrain(str(d)))
    raise FileNotFoundError(f"{d}: no metadata.yaml or scene_manifest.yaml with objects")


def load_class_map(name_or_path: str = DEFAULT_CLASS_MAP) -> List[str]:
    """The frozen YOLO class list (``core/data/<name>.json``, or a path)."""
    p = Path(name_or_path)
    if not p.suffix:
        p = DATA_DIR / f"{name_or_path}.json"
    doc = json.loads(p.read_text(encoding="utf-8"))
    return [str(c) for c in doc["classes"]]


def classes_for(objects: Iterable[WorldObject], base: List[str]) -> Dict[int, str]:
    """Class ids: the frozen list first (stable across worlds), then any type of
    this world the list does not know, sorted."""
    names = list(base)
    names += sorted({o.type for o in objects} - set(names))
    return {i: n for i, n in enumerate(names)}


# ---------------------------------------------------------------- ground truth
@dataclass
class PingTruth:
    side: str
    ping_number: int
    t_sim: float
    stamp: float
    pose: Tuple[float, float, float, float, float, float]
    altitude: float
    contacts: List[dict] = field(default_factory=list)


class GroundTruthBuffer:
    """Ground-truth pings keyed by ``(side, ping_number)``.

    A key holds several entries when the counter restarted; a lookup takes the
    entry nearest in time. Contacts are attached to their ping cycle by
    ``t_sim``, whichever of the two messages arrives first. ``horizon_s`` (None
    offline) bounds the memory to the last that many seconds of sim time."""

    def __init__(self, horizon_s: Optional[float] = 300.0, match_s: float = 5.0) -> None:
        self.horizon_s = horizon_s
        self.match_s = float(match_s)
        self._pings: Dict[Tuple[str, int], List[PingTruth]] = {}
        self._orphans: Dict[Tuple[str, int], List[Tuple[float, dict]]] = {}
        self.latest_stamp = -math.inf
        self.latest_t_sim = -math.inf
        self.counts = {"sensor_messages": 0, "contact_messages": 0, "pings": 0,
                       "contacts": 0, "dropped_pings": 0, "bad_messages": 0}

    # -- input
    def add_sensor(self, msg: dict) -> None:
        try:
            t_sim, stamp = float(msg["t_sim"]), float(msg["stamp"])
            sensors = msg["sensors"]
        except (KeyError, TypeError, ValueError):
            self.counts["bad_messages"] += 1
            return
        self.counts["sensor_messages"] += 1
        for side, s in sensors.items():
            pn = s.get("ping_number")
            if pn is None or s.get("dropped"):
                self.counts["dropped_pings"] += 1
                continue
            key = (str(side), int(pn))
            entry = PingTruth(side=key[0], ping_number=key[1], t_sim=t_sim, stamp=stamp,
                              pose=tuple(float(v) for v in s["pose"]),
                              altitude=float(s.get("altitude_m", 0.0)))
            waiting = self._orphans.get(key)
            if waiting:
                keep = []
                for t, c in waiting:
                    (entry.contacts.append(c) if abs(t - t_sim) <= TICK_MATCH_S
                     else keep.append((t, c)))
                if keep:
                    self._orphans[key] = keep
                else:
                    del self._orphans[key]
            self._pings.setdefault(key, []).append(entry)
            self.counts["pings"] += 1
        self.latest_stamp = max(self.latest_stamp, stamp)
        self.latest_t_sim = max(self.latest_t_sim, t_sim)

    def add_contacts(self, msg: dict) -> None:
        try:
            t_sim = float(msg["t_sim"])
            contacts = list(msg["contacts"])
        except (KeyError, TypeError, ValueError):
            self.counts["bad_messages"] += 1
            return
        self.counts["contact_messages"] += 1
        for c in contacts:
            try:
                key = (str(c["side"]), int(c["ping_number"]))
            except (KeyError, TypeError, ValueError):
                self.counts["bad_messages"] += 1
                continue
            self.counts["contacts"] += 1
            for entry in reversed(self._pings.get(key, [])):
                if abs(entry.t_sim - t_sim) <= TICK_MATCH_S:
                    entry.contacts.append(c)
                    break
            else:
                self._orphans.setdefault(key, []).append((t_sim, c))
        self.latest_t_sim = max(self.latest_t_sim, t_sim)

    # -- query
    def lookup(self, side: str, ping_number: int, t: float) -> Optional[PingTruth]:
        best, best_dt = None, self.match_s
        for entry in self._pings.get((side, int(ping_number)), ()):
            dt = abs(entry.stamp - t)
            if dt <= best_dt:
                best, best_dt = entry, dt
        return best

    def prune(self) -> None:
        if self.horizon_s is None:
            return
        cut, cut_sim = self.latest_stamp - self.horizon_s, self.latest_t_sim - self.horizon_s
        for key in list(self._pings):
            kept = [e for e in self._pings[key] if e.stamp >= cut]
            if kept:
                self._pings[key] = kept
            else:
                del self._pings[key]
        for key in list(self._orphans):
            kept = [(t, c) for t, c in self._orphans[key] if t >= cut_sim]
            if kept:
                self._orphans[key] = kept
            else:
                del self._orphans[key]

    def stats(self) -> Dict[str, Any]:
        return {**self.counts, "buffered_pings": sum(len(v) for v in self._pings.values()),
                "orphan_contacts": sum(len(v) for v in self._orphans.values())}


# -------------------------------------------------------------------- geometry
def _clip_across(corners: List[Tuple[float, float]], h: float) -> Optional[Tuple[float, float]]:
    """Across-track extent of the polygon ``corners`` (along, across) within the
    strip ``|along| <= h`` (Sutherland–Hodgman on the two edges), or None."""
    poly = corners
    for sign in (+1.0, -1.0):                    # keep sign·along <= h
        out: List[Tuple[float, float]] = []
        n = len(poly)
        for i in range(n):
            p, q = poly[i], poly[(i + 1) % n]
            pin, qin = sign * p[0] <= h, sign * q[0] <= h
            if pin:
                out.append(p)
            if pin != qin:
                t = (sign * h - p[0]) / (q[0] - p[0])
                out.append((sign * h, p[1] + t * (q[1] - p[1])))
        poly = out
        if not poly:
            return None
    acr = [c[1] for c in poly]
    return min(acr), max(acr)


def ping_object_bins(pose, sign: int, o: WorldObject, contact: dict, z_seabed: Optional[float],
                     *, bin_m: float, range_max_m: float, n_bins: int,
                     beam_rad: float = math.radians(DEFAULT_BEAM_DEG),
                     margin_bins: int = DEFAULT_RANGE_MARGIN_BINS,
                     strip_sigmas: float = DEFAULT_STRIP_SIGMAS) -> Optional[dict]:
    """The bins object ``o`` occupies in one ping (module docstring), or None
    when the ping's beam strip misses its footprint."""
    sx, sy, sz, _roll, _pitch, yaw = pose
    fwd, look = yaw, yaw + sign * math.pi / 2.0
    rx, ry = o.x - sx, o.y - sy
    l_c = rx * math.cos(fwd) + ry * math.sin(fwd)
    a_c = rx * math.cos(look) + ry * math.sin(look)
    if z_seabed is not None:
        dz = sz - z_seabed
    else:                                         # no terrain: the renderer's own height
        s_c = float(contact["slant_range_m"])
        dz = math.sqrt(max(s_c * s_c - a_c * a_c, 0.0)) + o.height
    dz = max(dz, 1e-3)
    dz_top = max(dz - o.height, 0.0)
    R = math.hypot(a_c, dz)
    h = min(strip_sigmas * R * beam_rad / 2.355, range_max_m * beam_rad / 2.0)
    pa, pc = o.yaw - fwd, o.yaw - look            # object axes in (along, across)
    uL = (math.cos(pa), math.cos(pc))
    uW = (-math.sin(pa), -math.sin(pc))
    hl, hw = 0.5 * o.length, 0.5 * o.width
    corners = [(l_c + i * hl * uL[0] + j * hw * uW[0], a_c + i * hl * uL[1] + j * hw * uW[1])
               for i, j in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
    span = _clip_across(corners, h)
    if span is None:
        return None
    a_min, a_max = max(span[0], 0.0), max(span[1], 0.0)
    s_near = math.hypot(a_min, dz_top)
    s_hl = max(math.hypot(a_max, dz_top), math.hypot(a_min, dz))
    if o.height > 0.0 and dz_top > 1e-3:
        s_tip = math.hypot(a_max * dz / dz_top, dz)
    else:
        s_tip = math.hypot(a_max, dz) if o.height <= 0.0 else range_max_m
    # the renderer's mask length, started at the far edge THIS ping crosses
    s_mask = math.hypot(a_max, dz) + max(float(contact.get("shadow_bins", 0.0)), 0.0) * bin_m
    s_end = max(s_tip, s_mask, s_hl)
    top = n_bins - 1
    k0 = int(math.floor(s_near / bin_m)) - margin_bins
    k1 = int(math.ceil(s_hl / bin_m)) - 1 + margin_bins
    k2 = int(math.ceil(s_end / bin_m)) - 1 + margin_bins
    clipped = k2 > top
    k0 = min(max(k0, 0), top)
    k1 = min(max(k1, k0), top)
    k2 = min(max(k2, k1), top)
    return {"k": (k0, k1, k2), "clipped": clipped, "slant": R, "across": a_c,
            "a_min": a_min, "a_max": a_max, "s_tip": s_tip, "s_mask": s_mask, "s_hl": s_hl,
            "dz": dz, "shadow_m": max(s_end - s_hl, 0.0),
            "aspect": math.degrees(pc) % 180.0}


# ---------------------------------------------------------------------- labels
def _row_side_pings(img: SeabedImageInput):
    r = img.rows
    for side, sign in SIDES:
        pns = r.port_ping_number if side == "port" else r.stbd_ping_number
        if pns is not None:
            yield side, sign, pns


class SimGroundTruthLabelSource:
    """``LabelSource`` for dataset mode in a simulator world (module docstring)."""

    name = LABEL_SOURCE_NAME

    def __init__(self, world: WorldInfo, buffer: GroundTruthBuffer,
                 classes: Optional[Dict[int, str]] = None, *, min_rows: int = 3,
                 split_gap_rows: int = 8, class_map: str = DEFAULT_CLASS_MAP,
                 beam_deg: float = DEFAULT_BEAM_DEG,
                 range_margin_bins: int = DEFAULT_RANGE_MARGIN_BINS) -> None:
        self.world = world
        self.buffer = buffer
        self.class_map = class_map
        self.classes = (dict(classes) if classes is not None else
                        classes_for(world.objects.values(), load_class_map(class_map)))
        self._class_id = {v: k for k, v in self.classes.items()}
        self.min_rows = max(int(min_rows), 1)
        self.split_gap_rows = max(int(split_gap_rows), 1)
        self.beam_rad = math.radians(float(beam_deg))
        self.range_margin_bins = max(int(range_margin_bins), 0)
        self._seabed = {oid: (world.terrain.seabed_z(o) if world.terrain is not None else None)
                        for oid, o in world.objects.items()}
        self._cache: Optional[Tuple[str, dict]] = None
        self.totals: Dict[str, Any] = {
            "images": 0, "images_with_objects": 0, "boxes": 0, "by_class": {},
            "truncated_boxes": 0, "boxes_below_min_rows": 0,
            "images_incomplete_coverage": 0,
            "skipped": {"ghost": 0, "wall": 0, "invisible": 0, "unknown_object": 0,
                        "unknown_class": 0, "outside_beam": 0}}

    # -- protocol
    def info(self) -> Dict[str, Any]:
        t = self.world.terrain
        return {"name": self.name, "world_objects_file": self.world.path,
                "world_objects_sha256": self.world.sha256, "objects": len(self.world.objects),
                "terrain": (None if t is None else
                            {"file": t.source, "objects_stamped_in": t.stamped}),
                "class_map": self.class_map, "min_rows": self.min_rows,
                "split_gap_rows": self.split_gap_rows,
                "beam_deg": math.degrees(self.beam_rad),
                "range_margin_bins": self.range_margin_bins,
                "box": "highlight + shadow, one per object, side and contiguous row run",
                "excluded": "multipath ghosts, wall echoes, pings where the object is "
                            "not visible or outside the beam strip"}

    def _needed(self, img: SeabedImageInput):
        for side, _sign, pns in _row_side_pings(img):
            for i in range(img.shape[0]):
                n = int(pns[i])
                if n:
                    yield side, n, float(img.rows.t[i])

    def coverage(self, img: SeabedImageInput) -> Tuple[int, int]:
        """(source pings found in the ground truth, source pings needed)."""
        found = needed = 0
        for side, n, t in self._needed(img):
            needed += 1
            found += self.buffer.lookup(side, n, t) is not None
        return found, needed

    def ready(self, img: SeabedImageInput) -> bool:
        """Every source ping of the picture is in the ground truth, and so is the
        ping after the newest row (it decides ``truncated_top``), so a label
        made live equals one made offline from the complete record."""
        found, needed = self.coverage(img)
        if found != needed:
            return False
        for side, _sign, pns in _row_side_pings(img):
            n = int(pns[0])
            if n and self.buffer.lookup(side, n + 1, float(img.rows.t[0])) is None:
                return False
        return True

    def labels_for(self, img: SeabedImageInput) -> List[YoloBox]:
        return [YoloBox(b["class_id"], *b["yolo"]) for b in self._analyse(img)["boxes"]]

    def detail_for(self, img: SeabedImageInput) -> Dict[str, Any]:
        return self._analyse(img)

    def stats(self) -> Dict[str, Any]:
        return json.loads(json.dumps(self.totals))

    # -- the computation
    def _analyse(self, img: SeabedImageInput) -> dict:
        if self._cache is not None and self._cache[0] == img.key:
            return self._cache[1]
        out = self._compute(img)
        self._cache = (img.key, out)
        t = self.totals
        t["images"] += 1
        t["images_with_objects"] += bool(out["boxes"])
        t["boxes"] += len(out["boxes"])
        t["truncated_boxes"] += sum(1 for b in out["boxes"]
                                    if b["truncated_top"] or b["truncated_bottom"])
        t["boxes_below_min_rows"] += out["boxes_below_min_rows"]
        t["images_incomplete_coverage"] += out["ground_truth_coverage"] < 1.0
        for b in out["boxes"]:
            t["by_class"][b["type"]] = t["by_class"].get(b["type"], 0) + 1
        for k, v in out["skipped"].items():
            t["skipped"][k] += v
        return out

    def _usable(self, c: dict, skipped: dict) -> Optional[WorldObject]:
        if c.get("ghost") or c.get("via"):
            skipped["ghost"] += 1
            return None
        oid = c.get("object_id")
        if c.get("type") == "wall" or not isinstance(oid, int) or isinstance(oid, bool):
            skipped["wall"] += 1
            return None
        if not c.get("visible", True):
            skipped["invisible"] += 1
            return None
        o = self.world.objects.get(oid)
        if o is None:
            skipped["unknown_object"] += 1
            return None
        if o.type not in self._class_id:
            skipped["unknown_class"] += 1
            return None
        return o

    def _compute(self, img: SeabedImageInput) -> dict:
        H, W = img.shape
        half = W // 2
        # device bins: [k·L/N, (k+1)·L/N); the GCS pitch is L/(N - 1)
        bin_m = float(img.bin_pitch_m) * (half - 1) / half if half > 1 else float(img.bin_pitch_m)
        range_max = bin_m * half
        rows_t = np.asarray(img.rows.t, np.float64)
        skipped = {k: 0 for k in self.totals["skipped"]}
        found = needed = 0
        obs: Dict[Tuple[int, str], List[dict]] = {}
        for side, sign, pns in _row_side_pings(img):
            for i in range(H):
                n = int(pns[i])
                if n == 0:
                    continue
                needed += 1
                ping = self.buffer.lookup(side, n, float(rows_t[i]))
                if ping is None:
                    continue
                found += 1
                for c in ping.contacts:
                    o = self._usable(c, skipped)
                    if o is None:
                        continue
                    g = ping_object_bins(ping.pose, sign, o, c, self._seabed.get(o.object_id),
                                         bin_m=bin_m, range_max_m=range_max, n_bins=half,
                                         beam_rad=self.beam_rad,
                                         margin_bins=self.range_margin_bins)
                    if g is None:
                        skipped["outside_beam"] += 1
                        continue
                    g.update(row=i, ping=n)
                    obs.setdefault((o.object_id, side), []).append(g)
        boxes: List[dict] = []
        below = 0
        for (oid, side), items in sorted(obs.items(), key=lambda kv: (kv[0][1], kv[0][0])):
            items.sort(key=lambda it: it["row"])
            for run in self._runs(items):
                rows = sorted({it["row"] for it in run})
                if len(rows) < self.min_rows:
                    below += 1
                    continue
                boxes.append(self._box(img, oid, side, run, rows, H, W, half, bin_m,
                                       range_max))
        boxes.sort(key=lambda b: (b["box_px"]["r0"], b["box_px"]["c0"], b["object_id"]))
        coverage = 1.0 if needed == 0 else found / needed
        return {"source": self.name, "boxes": boxes,
                "ground_truth_coverage": round(coverage, 6),
                "source_pings": {"needed": needed, "found": found},
                "device_bin_m": bin_m,
                "skipped": skipped, "boxes_below_min_rows": below}

    def _runs(self, items: List[dict]):
        run = [items[0]]
        for it in items[1:]:
            if it["row"] - run[-1]["row"] > self.split_gap_rows:
                yield run
                run = []
            run.append(it)
        yield run

    def _box(self, img, oid, side, run, rows, H, W, half, bin_m, range_max) -> dict:
        o = self.world.objects[oid]
        k0 = min(it["k"][0] for it in run)
        k1 = max(it["k"][1] for it in run)
        k2 = max(it["k"][2] for it in run)

        def cols(a, b):           # bins a..b -> inclusive pixel columns
            return ([half - 1 - b, half - 1 - a] if side == "port"
                    else [half + a, half + b])

        c0, c1 = cols(k0, k2)
        r0, r1 = rows[0], rows[-1]
        pings = sorted({it["ping"] for it in run})
        cx = (c0 + c1 + 1) / 2.0 / W
        cy = (r0 + r1 + 1) / 2.0 / H
        w = (c1 - c0 + 1) / W
        h = (r1 - r0 + 1) / H
        return {
            "object_id": oid, "type": o.type, "class_id": self._class_id[o.type],
            "side": side,
            "box_px": {"r0": r0, "r1": r1, "c0": c0, "c1": c1},
            "yolo": [round(cx, 6), round(cy, 6), round(w, 6), round(h, 6)],
            "highlight_cols": cols(k0, k1),
            "shadow_cols": cols(k1 + 1, k2) if k2 > k1 else None,
            "rows": len(rows), "pings": len(pings),
            "ping_range": [pings[0], pings[-1]],
            "truncated_top": r0 == 0 and self._continues(img, side, oid, 0, +1,
                                                         bin_m, range_max, half),
            "truncated_bottom": (r1 == H - 1 and self._continues(
                img, side, oid, H - 1, -1, bin_m, range_max, half)),
            "clipped_far_range": any(it["clipped"] for it in run),
            "slant_m": round(float(np.median([it["slant"] for it in run])), 4),
            "ground_m": round(float(np.median([it["across"] for it in run])), 4),
            "dz_m": round(float(np.median([it["dz"] for it in run])), 4),
            "shadow_m": round(float(max(it["shadow_m"] for it in run)), 4),
            "aspect_deg": round(float(np.median([it["aspect"] for it in run])), 2),
            "object": {"x": o.x, "y": o.y, "yaw": o.yaw, "length": o.length,
                       "width": o.width, "height": o.height},
        }

    def _continues(self, img, side, oid, row, step, bin_m, range_max, half) -> bool:
        """Does the ping just beyond this edge row still see the object?
        (Row 0 is the newest: beyond it is the next ping, ``step`` +1.)"""
        pns = img.rows.port_ping_number if side == "port" else img.rows.stbd_ping_number
        n = int(pns[row])
        if n == 0:
            return False
        nxt = self.buffer.lookup(side, n + step, float(img.rows.t[row]))
        if nxt is None:
            return False
        sign = +1 if side == "port" else -1
        o = self.world.objects[oid]
        for c in nxt.contacts:
            if c.get("object_id") == oid and not c.get("ghost") and not c.get("via") \
                    and c.get("visible", True):
                return ping_object_bins(nxt.pose, sign, o, c, self._seabed.get(oid),
                                        bin_m=bin_m, range_max_m=range_max, n_bins=half,
                                        beam_rad=self.beam_rad,
                                        margin_bins=self.range_margin_bins) is not None
        return False
