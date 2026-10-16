"""The pipeline: preprocess -> detect -> calibrate -> georeference -> record.

``Pipeline.run(img)`` is pure: the same input and configuration give the
same record, byte for byte (no randomness, no wall clock, no global
state). The caller (node or CLI) adds ``source`` and ``provenance``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from .. import __version__
from . import geometry as geo
from .calibration import Calibrator, make_calibrator
from .detectors import Detector, RawDetection, make_detector
from .image import SeabedImageInput
from .preprocess import Preprocessor, make_preprocessor
from .records import INPUT_SCHEMA, RECORD_SCHEMA, float_list, stats
from .uncertainty import UncertaintyParams, covariance_xy

#: Station uid of detection k of an image (``docs/topics.md`` §3), the range
#: after the GCS's former in-process ``1_000_000 + 16·image_id + k``.
UID_BASE = 2_000_000
UID_PER_IMAGE = 16
#: Detections past the 16th of one image (none today: the dummy makes one)
#: take a second, non-overlapping range rather than colliding.
UID_OVERFLOW_BASE = 1_000_000_000


def station_uid(image_id: int, k: int) -> int:
    if k < UID_PER_IMAGE:
        return UID_BASE + UID_PER_IMAGE * int(image_id) + int(k)
    return UID_OVERFLOW_BASE + 1000 * int(image_id) + int(k)


@dataclass(frozen=True)
class PipelineConfig:
    #: Detector-input mapping (decision 6.4): ``fixed_gray`` (default) or the
    #: baseline ``identity`` (the GCS display PNG as is).
    preprocess: str = "fixed_gray"
    #: ``fixed_gray`` parameter file (packaged name or a path).
    detector_input_mapping: str = "fixed_gray_v1"
    #: Decision 6.5: detector-input mask below this slant range [m]; 0 = off.
    near_range_mask_m: float = 0.0
    #: Decision 6.2: yaw rate averaged over this many seconds of row time for
    #: the turn flags (Phase 4 M1 used 1 s; an engineering choice, not validated).
    turn_window_s: float = 1.0
    detector: str = "dummy_center"
    detector_args: Dict[str, Any] = field(default_factory=dict)
    calibration: str = "identity"
    uncertainty: UncertaintyParams = field(default_factory=UncertaintyParams)
    footprint_row_step: int = 16
    #: Drawn extent of a point detection on the map [m] (the station's
    #: in-process dummy drew 1.0 m).
    point_extent_m: float = 1.0

    @classmethod
    def from_params(cls, *, preprocess="fixed_gray", detector="dummy_center",
                    calibration="identity", sigma_h_m=0.10, sigma_heading_deg=2.0,
                    sigma_nav_m=1.0, footprint_row_step=16,
                    detector_input_mapping="fixed_gray_v1", near_range_mask_m=0.0,
                    turn_window_s=1.0) -> "PipelineConfig":
        return cls(preprocess=str(preprocess), detector=str(detector),
                   calibration=str(calibration),
                   detector_input_mapping=str(detector_input_mapping),
                   near_range_mask_m=float(near_range_mask_m),
                   turn_window_s=float(turn_window_s),
                   uncertainty=UncertaintyParams(
                       sigma_h_m=float(sigma_h_m),
                       sigma_heading_rad=math.radians(float(sigma_heading_deg)),
                       sigma_nav_m=float(sigma_nav_m)),
                   footprint_row_step=int(footprint_row_step))

    def to_json(self) -> dict:
        return {"preprocess": self.preprocess,
                "detector_input_mapping": self.detector_input_mapping,
                "near_range_mask_m": self.near_range_mask_m,
                "turn_window_s": self.turn_window_s, "detector": self.detector,
                "detector_args": dict(self.detector_args),
                "calibration": self.calibration,
                "uncertainty": self.uncertainty.to_json(),
                "footprint_row_step": self.footprint_row_step,
                "point_extent_m": self.point_extent_m}


class Pipeline:
    def __init__(self, config: Optional[PipelineConfig] = None, *,
                 preprocessor: Optional[Preprocessor] = None,
                 detector: Optional[Detector] = None,
                 calibrator: Optional[Calibrator] = None) -> None:
        self.config = config or PipelineConfig()
        self.preprocessor = preprocessor or make_preprocessor(
            self.config.preprocess, mapping=self.config.detector_input_mapping,
            near_range_mask_m=self.config.near_range_mask_m)
        self.detector = detector or make_detector(self.config.detector,
                                                  **self.config.detector_args)
        self.calibrator = calibrator or make_calibrator(self.config.calibration)

    # ---- public ------------------------------------------------------------------
    def detector_input_info(self) -> dict:
        """What the training images are, for the dataset writer's manifest."""
        info = {"name": self.preprocessor.name, "params": dict(self.preprocessor.params)}
        window = info["params"].get("window_db")
        if window:
            info["log_range_db"] = float(window[1]) - float(window[0])
        return info

    def run(self, img: SeabedImageInput) -> Tuple[Dict[str, Any], Optional[List[dict]]]:
        """(record, detections). ``detections`` is None when the image has
        no pixels (the detector could not run); else a list, maybe empty."""
        record, dets, _inp = self.run_full(img)
        return record, dets

    def run_full(self, img: SeabedImageInput):
        """(record, detections, DetectorInput or None) -- the detector input
        too, for the dataset writer (training images = the detector input)."""
        record = self._base_record(img)
        if img.db is None:
            record["processing"]["detector"]["status"] = "no_pixels"
            record["sides"] = None
            record["footprint"] = None
            record["summary"]["one_sided_rows"] = None
            record["detections"] = None
            return record, None, None
        inp = self.preprocessor.run(img)
        record["summary"]["detector_input"] = {
            "gray": inp.gray is not None,
            "valid_share": (None if inp.valid is None else
                            float(np.count_nonzero(inp.valid)) / inp.valid.size)}
        raw = self.detector.detect(inp)
        grids = geo.world_grids(img)
        turn = self._turn(img)
        dets = [self._detection(img, grids, d, k, turn) for k, d in enumerate(raw)]
        spans = geo.side_spans(img)
        record["sides"], record["footprint"] = self._coverage(img, spans)
        pp, ps = spans["port"]["present"], spans["starboard"]["present"]
        record["summary"]["one_sided_rows"] = int(np.count_nonzero(pp ^ ps))
        record["detections"] = dets
        return record, dets, inp

    # ---- record parts ------------------------------------------------------------
    def _turn(self, img: SeabedImageInput, yaw_rate=None):
        """(smoothed yaw rate, r*, inner side, backscan start) per row."""
        r = img.rows
        if yaw_rate is None:
            imu = r.yaw_rate is not None and bool(np.isfinite(r.yaw_rate).any())
            yaw_rate = r.yaw_rate if imu else geo.derived_yaw_rate(r.t, r.yaw)
        rate_s = geo.smooth_in_time(r.t, yaw_rate, self.config.turn_window_s)
        rstar, inner, back = geo.turn_geometry(r.speed, rate_s, r.range)
        return rate_s, rstar, inner, back

    def _base_record(self, img: SeabedImageInput) -> Dict[str, Any]:
        r = img.rows
        h = geo.row_altitude(img)
        look_p, look_s = geo.look_azimuths(r.yaw)
        imu = r.yaw_rate is not None and bool(np.isfinite(r.yaw_rate).any())
        yaw_rate = r.yaw_rate if imu else geo.derived_yaw_rate(r.t, r.yaw)
        finite_t = r.t[np.isfinite(r.t)]
        rate_f = np.abs(yaw_rate[np.isfinite(yaw_rate)])
        cfg = self.config
        # Decision 6.2: turn evidence per row -- flags only, every row kept.
        rate_s, rstar, inner, back = self._turn(img, yaw_rate)
        n_rows = max(r.n, 1)
        return {
            "schema": RECORD_SCHEMA,
            "key": img.key, "stream_id": img.stream_id, "image_id": img.image_id,
            "input": {"schema": INPUT_SCHEMA, "metadata_schema": img.schema,
                      "georef_convention": img.georef.convention,
                      "georef": img.georef.to_json(),
                      "row_geometry": img.row_geometry,
                      "rows": img.shape[0], "cols": img.shape[1],
                      "bin_pitch_m": img.bin_pitch_m},
            "processing": {
                "perception_version": __version__,
                "preprocess": {"name": self.preprocessor.name,
                               "params": dict(self.preprocessor.params)},
                "detector": {"name": self.detector.name,
                             "version": self.detector.version,
                             "dummy": bool(self.detector.is_dummy), "status": "ran"},
                "calibration": {"name": self.calibrator.name,
                                "params": dict(self.calibrator.params),
                                "calibrated": bool(self.calibrator.calibrated)},
                "uncertainty": cfg.uncertainty.to_json(),
            },
            "time": {"t_min_s": float(finite_t.min()) if finite_t.size else None,
                     "t_max_s": float(finite_t.max()) if finite_t.size else None,
                     "n_rows": r.n, "clock": "ros_time_of_sonar_profiles"},
            "track": {"t_s": float_list(r.t), "x_m": float_list(r.x),
                      "y_m": float_list(r.y), "heading_rad": float_list(r.yaw),
                      "speed_mps": float_list(r.speed),
                      "altitude_m": float_list(r.altitude),
                      "bottom_m": float_list(r.bottom),
                      "georef_altitude_m": float_list(h),
                      "roll_rad": float_list(r.roll), "pitch_rad": float_list(r.pitch),
                      "yaw_rate_rad_s": float_list(yaw_rate),
                      "yaw_rate_source": "imu" if imu else "derived",
                      "look_port_rad": float_list(look_p),
                      "look_stbd_rad": float_list(look_s),
                      "port_ping_number": (None if r.port_ping_number is None
                                           else [int(v) for v in r.port_ping_number]),
                      "stbd_ping_number": (None if r.stbd_ping_number is None
                                           else [int(v) for v in r.stbd_ping_number]),
                      "yaw_rate_smoothed_rad_s": float_list(rate_s),
                      "turn_radius_m": float_list(rstar),
                      "turn_inner_side": ["port" if v > 0 else "starboard" if v < 0 else None
                                          for v in inner],
                      "backscan_from_m": float_list(back)},
            "sides": None,
            "footprint": None,
            "summary": {"speed_mps": stats(r.speed), "altitude_m": stats(h),
                        "heading_rad": {"start": float(r.yaw[-1]), "end": float(r.yaw[0])},
                        "max_abs_yaw_rate_rad_s": (float(rate_f.max()) if rate_f.size
                                                   else None),
                        "one_sided_rows": None,
                        "display_model_ready": img.display_model_ready,
                        "turn": {"window_s": cfg.turn_window_s,
                                 "criterion": "r* = v/|w| < range: inner-side backscan "
                                              "beyond r* (Cobra et al. 1992 sec. I.A)",
                                 "rows_rstar_lt_range": int(np.count_nonzero(
                                     np.isfinite(back))),
                                 "rows_rstar_lt_half_range": int(np.count_nonzero(
                                     np.isfinite(rstar) & (rstar < r.range / 2.0))),
                                 "share_rstar_lt_range": float(np.count_nonzero(
                                     np.isfinite(back))) / n_rows}},
            "detections": None,
            "provenance": {},
        }

    def _coverage(self, img: SeabedImageInput, spans) -> Tuple[dict, Optional[dict]]:
        rows_idx = geo.decimated_rows(img.shape[0], self.config.footprint_row_step)
        h = geo.row_altitude(img)
        sides = {}
        far_pts = {}
        for name, is_port in (("port", True), ("starboard", False)):
            sp = spans[name]
            near, far = sp["near"], sp["far"]
            ok = np.isfinite(near)
            if ok.any():
                ground_near = [geo.ground_range(img, near[i], h[i], is_port)
                               for i in np.flatnonzero(ok)]
                ground_far = [geo.ground_range(img, far[i], h[i], is_port)
                              for i in np.flatnonzero(ok)]
                near_pts = geo.edge_points(img, rows_idx, near, is_port)
                far_p = geo.edge_points(img, rows_idx, far, is_port)
                poly = near_pts + far_p[::-1]
                if poly:
                    poly.append(list(poly[0]))
                sides[name] = {
                    "present_rows": int(np.count_nonzero(sp["present"])),
                    "seabed_rows": int(np.count_nonzero(ok)),
                    "slant_range_m": [float(np.nanmin(near)), float(np.nanmax(far))],
                    "ground_range_m": [float(min(ground_near)), float(max(ground_far))],
                    "polygon_xy": poly}
                far_pts[name] = far_p
            else:
                sides[name] = {"present_rows": int(np.count_nonzero(sp["present"])),
                               "seabed_rows": 0, "slant_range_m": None,
                               "ground_range_m": None, "polygon_xy": None}
                far_pts[name] = []
        outline = far_pts["port"] + far_pts["starboard"][::-1]
        if outline:
            outline.append(list(outline[0]))
        footprint = ({"outline_xy": outline, "row_step": self.config.footprint_row_step}
                     if outline else None)
        return sides, footprint

    def _row_pitch(self, img: SeabedImageInput) -> float:
        a = img.rows.along
        if a.size >= 2:
            d = np.abs(np.diff(a))
            d = d[np.isfinite(d)]
            if d.size:
                return float(np.median(d))
        return float(img.bin_pitch_m)

    def _detection(self, img: SeabedImageInput, grids, d: RawDetection,
                   k: int, turn) -> Dict[str, Any]:
        H, W = img.shape
        row = int(min(max(d.center_px[0], 0), H - 1))
        col = int(min(max(d.center_px[1], 0), W - 1))
        r = img.rows
        port_cols, slant_cols = geo.column_slant(W, img.bin_pitch_m)
        is_port = bool(port_cols[col])
        s = float(slant_cols[col])
        h = float(geo.row_altitude(img)[row])
        yaw = float(r.yaw[row])
        look_p, look_s = geo.look_azimuths(np.array([yaw]))
        look = float(look_p[0] if is_port else look_s[0])
        wx, wy = float(grids[0][row, col]), float(grids[1][row, col])
        conf_raw = float(d.score_raw)
        conf = float(self.calibrator.calibrate(conf_raw, d.logit, d.class_id))
        r0, c0, r1, c1 = (int(v) for v in d.bbox_px)
        row_pitch = self._row_pitch(img)
        # A degenerate box (one pixel) is a point detection: no extent term
        # in the covariance, drawn at point_extent_m. Otherwise the inclusive
        # box spans (c1 - c0 + 1) bins across and (r1 - r0 + 1) rows along
        # (slant pitch across-track: the image is in the slant domain).
        point = r0 == r1 and c0 == c1
        extent_across = 0.0 if point else (abs(c1 - c0) + 1) * img.bin_pitch_m
        extent_along = 0.0 if point else (abs(r1 - r0) + 1) * row_pitch
        cov = covariance_xy(self.config.uncertainty, slant=s, h=h,
                            pitch=img.bin_pitch_m, row_pitch=row_pitch,
                            look_rad=look, heading_rad=yaw,
                            extent_across_m=extent_across,
                            extent_along_m=extent_along)
        return {
            "detection_id": f"{img.key}/{k}",
            "uid": station_uid(img.image_id, k),
            "image_id": img.image_id, "stream_id": img.stream_id,
            "class_name": d.class_name, "class_id": int(d.class_id),
            "confidence": conf, "confidence_raw": conf_raw,
            "calibrated": bool(self.calibrator.calibrated),
            "dummy": bool(self.detector.is_dummy),
            "pixel": [row, col], "bbox_px": [r0, c0, r1, c1],
            "world_xy": [wx, wy], "covariance_xy": cov,
            "extent_m": [self.config.point_extent_m if point else extent_across,
                         self.config.point_extent_m if point else extent_along],
            "t_row_s": float(r.t[row]),
            "side": "port" if is_port else "starboard",
            "slant_range_m": s,
            "ground_range_m": geo.ground_range(img, s, h, is_port),
            "look_azimuth_rad": look,
            "grazing_angle_rad": geo.grazing_angle(s, h),
            "boat_heading_rad": yaw, "boat_speed_mps": float(r.speed[row]),
            "boat_altitude_m": h,
            "boat_roll_rad": (None if r.roll is None else
                              (float(r.roll[row]) if np.isfinite(r.roll[row]) else None)),
            "boat_pitch_rad": (None if r.pitch is None else
                               (float(r.pitch[row]) if np.isfinite(r.pitch[row]) else None)),
            # Decision 6.2: a flag, never a rejection. True when the detection
            # lies on the inner side of a turn beyond r* (scanned backwards).
            "turn_radius_m": (float(turn[1][row]) if np.isfinite(turn[1][row]) else None),
            "turn_backscan": bool(np.isfinite(turn[3][row])
                                  and turn[2][row] == (1 if is_port else -1)
                                  and geo.ground_range(img, s, h, is_port) > turn[1][row]),
        }
