#!/usr/bin/env python3
"""Phase 4 measurements behind ``docs/PREPROCESSING_STUDY.md`` §5 (read-only, no ROS).

Dev tool, not installed. It reads the GCS recordings (seabed-image folders and their
``.svlog`` files) and uses the GCS's own code from the superproject checkout
(``BlueBoat-SSS/blueboat_sss``: ``load_svlog``, ``SeabedImager``, the display model) and
this module's core, so every number is what the station itself would compute. It never
writes inside the data tree (CM-7): everything goes to ``--out``.

    /usr/bin/python3 tools/preprocessing_study.py --out DIR [--data ~/ros2_ws/data/SSS_data]
        [--extra-session DIR ...]

Measurements (STUDY §5 numbering):
1. turns: per-row yaw rate, and the share of rows / images where the turn radius
   ``r* = v/|ω|`` lies inside the swath (Cobra et al. 1992 §I.A: backscan on the inner side);
2. along-track scale: per-image row-pose path / nominal span for speed windows 1, 2, 5 s;
3. roll vs per-side brightness: correlation of detrended roll with detrended
   (port − starboard) seabed level, per recording;
5. raw-dB level vs normalised slant range ``x = s/h`` per set (sim vs real), and the PNG
   drift of a live mission (one image's raw dB through every later image's model);
6. live vs offline: nadir NaN share and PNG brightness of live pictures vs the same
   recording regenerated offline.
(4, latency, comes from the live runs: ``--latency`` reads their logs.)
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
HERE = Path(__file__).resolve()
SUPER = HERE.parents[2]
GCS = SUPER / "BlueBoat-SSS" / "blueboat_sss"
PERC = HERE.parents[1] / "blueboat_perception"
for p in (str(GCS), str(PERC)):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402

from blueboat_perception.core import geometry as geo  # noqa: E402
from blueboat_perception.core.image import SeabedImageInput  # noqa: E402
from blueboat_perception.core.session import list_images  # noqa: E402


def pct(a, q):
    a = np.asarray(a, float)
    a = a[np.isfinite(a)]
    return float(np.percentile(a, q)) if a.size else None


def summary(a) -> Dict[str, Optional[float]]:
    a = np.asarray(a, float)
    a = a[np.isfinite(a)]
    if not a.size:
        return {"n": 0}
    return {"n": int(a.size), "min": float(a.min()), "p5": pct(a, 5), "median": pct(a, 50),
            "p95": pct(a, 95), "max": float(a.max())}


def image_sets(data: Path, extra: List[Path]) -> Dict[str, Path]:
    sets = {}
    for p in sorted(data.rglob("seabed_images*")):
        if p.is_dir() and (p / "metadata").is_dir():
            sets[str(p.relative_to(data))] = p
    for e in extra:
        for p in sorted(Path(e).rglob("seabed_images")):
            sets["extra/" + p.parent.name] = p
    return sets


def is_real(name: str) -> bool:
    return name.startswith("merged_sessions/") or name.startswith("beach_sept/")


# ---------------------------------------------------------------- 1. turns
def smooth_in_time(t, w, window_s=1.0):
    """Mean of ``w`` over rows within +/- window_s/2 of each row (sustained rate)."""
    t = np.asarray(t, float)
    w = np.asarray(w, float)
    order = np.argsort(t, kind="stable")
    ts, wsrt = t[order], w[order]
    fin = np.isfinite(wsrt)
    cs = np.concatenate([[0.0], np.cumsum(np.where(fin, wsrt, 0.0))])
    cn = np.concatenate([[0], np.cumsum(fin)])
    lo = np.searchsorted(ts, ts - window_s / 2, "left")
    hi = np.searchsorted(ts, ts + window_s / 2, "right")
    n = cn[hi] - cn[lo]
    mean = np.where(n > 0, (cs[hi] - cs[lo]) / np.maximum(n, 1), np.nan)
    out = np.empty_like(mean)
    out[order] = mean
    return out


def turns(sets: Dict[str, Path]) -> dict:
    out = {}
    for name, d in sets.items():
        rows = imgs = flagged_imgs = imu = 0
        flag_full = flag_half = sus_full = sus_half = sus_imgs = 0
        rates, sus_rates = [], []
        for f in list_images(d):
            with open(f.json, encoding="utf-8") as fh:
                meta = json.load(fh)
            img = SeabedImageInput.from_metadata(meta, None, stream_id="s")
            r = img.rows
            if r.yaw_rate is not None and np.isfinite(r.yaw_rate).any():
                w = r.yaw_rate
                imu += 1
            else:
                w = geo.derived_yaw_rate(r.t, r.yaw)
            v = r.speed
            ok = np.isfinite(w) & np.isfinite(v) & (v > 0.05)
            if not ok.any():
                continue
            rstar = np.where(np.abs(w) > 1e-6, v / np.maximum(np.abs(w), 1e-6), np.inf)
            full = ok & (rstar < r.range)
            half = ok & (rstar < r.range / 2)
            ws = smooth_in_time(r.t, w, 1.0)
            rs = np.where(np.abs(ws) > 1e-6, v / np.maximum(np.abs(ws), 1e-6), np.inf)
            sfull = ok & np.isfinite(ws) & (rs < r.range)
            shalf = ok & np.isfinite(ws) & (rs < r.range / 2)
            rows += int(ok.sum())
            flag_full += int(full.sum())
            flag_half += int(half.sum())
            sus_full += int(sfull.sum())
            sus_half += int(shalf.sum())
            imgs += 1
            if full.sum() >= 0.10 * ok.sum():
                flagged_imgs += 1
            if sfull.sum() >= 0.10 * ok.sum():
                sus_imgs += 1
            rates.extend(np.degrees(np.abs(w[ok])).tolist())
            sus_rates.extend(np.degrees(np.abs(ws[ok & np.isfinite(ws)])).tolist())
        if rows:
            out[name] = {"images": imgs, "rows": rows, "imu_images": imu,
                         "rows_rstar_lt_range_pct": 100.0 * flag_full / rows,
                         "rows_rstar_lt_half_range_pct": 100.0 * flag_half / rows,
                         "images_ge10pct_rows_flagged_pct": 100.0 * flagged_imgs / imgs,
                         "abs_yaw_rate_deg_s": summary(rates),
                         "sustained_1s": {
                             "rows_rstar_lt_range_pct": 100.0 * sus_full / rows,
                             "rows_rstar_lt_half_range_pct": 100.0 * sus_half / rows,
                             "images_ge10pct_rows_flagged_pct": 100.0 * sus_imgs / imgs,
                             "abs_yaw_rate_deg_s": summary(sus_rates)}}
    return out


def ping_turns(logs: List[Path]) -> dict:
    """Per ping, from the .svlog: the IMU yaw rate (ATTITUDE yawspeed, the Q8
    replay field) and the chord speed over ~1 s of ping poses; r* = v/|w| over
    the ping's configured range, instantaneous and sustained (1 s mean)."""
    from blueboat_gcs.core.svlog import load_svlog
    out = {}
    for log in logs:
        m = load_svlog(log)
        ps = [p for p in m.pings if math.isfinite(p.yaw_rate)]
        if len(ps) < 200:
            out[log.name] = {"pings": len(ps), "note": "no ATTITUDE yaw rate"}
            continue
        t = np.array([p.t for p in ps])
        xy = np.array([[p.robot_x, p.robot_y] for p in ps])
        w = np.array([p.yaw_rate for p in ps])
        R = np.array([p.slant_range_m for p in ps])
        v = np.full(t.size, np.nan)
        j = 0
        for i in range(t.size):
            while j < t.size - 1 and t[j] < t[i] + 1.0:
                j += 1
            if t[j] - t[i] > 0.5:
                v[i] = float(np.hypot(*(xy[j] - xy[i]))) / (t[j] - t[i])
        ws = smooth_in_time(t, w, 1.0)
        res = {"pings": int(t.size), "abs_yaw_rate_deg_s": summary(np.degrees(np.abs(w))),
               "abs_yaw_rate_1s_deg_s": summary(np.degrees(np.abs(ws))),
               "speed_mps": summary(v)}
        for lab, ww in (("instant", w), ("sustained_1s", ws)):
            ok = np.isfinite(v) & (v > 0.05) & np.isfinite(ww)
            rs = v[ok] / np.maximum(np.abs(ww[ok]), 1e-6)
            res[lab] = {"pings_rstar_lt_range_pct": 100.0 * float(np.mean(rs < R[ok])),
                        "pings_rstar_lt_half_range_pct": 100.0 * float(np.mean(rs < R[ok] / 2))}
        out[log.name] = res
    return out


# ------------------------------------------------------- 2. along-track scale
def svlogs_of(name: str, d: Path, data: Path) -> List[Path]:
    root = d.parent if d.name == "seabed_images" else d.parent
    if name.startswith("beach_sept/"):
        stem = d.name.replace("seabed_images_", "")
        return [p for p in [data / "beach_sept" / f"{stem}.svlog"] if p.is_file()]
    return sorted(root.glob("*.svlog"))


def scale_ratios(pings, breaks, window_s: float, geometry: str = "square"):
    from blueboat_gcs.config.settings import load_config
    from blueboat_gcs.core.seabed_imager import SeabedImager, feed_pings
    cfg = load_config()
    cfg.along_track.window_s = float(window_s)
    cfg.mosaic.nadir_contrast = False                  # geometry only: no display model
    cfg.seabed.row_geometry = geometry
    imager = SeabedImager(cfg)
    ratios, chords = [], []
    def on(img):
        n = img.intensity_db.shape[0]
        if n < cfg.seabed.rows:
            return
        xy = img.row_pose[:, :2]
        path = float(np.sum(np.hypot(*np.diff(xy, axis=0).T)))
        nominal = (n - 1) * img.bin_pitch_m
        ratios.append(path / nominal)
        chords.append(float(np.hypot(*(xy[0] - xy[-1]))) / nominal)
    imager.image_ready.connect(on)
    feed_pings(imager, pings, breaks)
    return ratios, chords


def row_jitter(pings, breaks, window_s: float, block_rows: int = 32):
    """Row-level jitter of the square rows (Phase 4 follow-up, decision 6.3).

    Per full image, the row-pose path / nominal span over consecutive
    ``block_rows`` blocks. Their std within an image is the short-scale scale
    noise a window adds; the largest |log| ratio between adjacent blocks is
    the worst scale jump inside one picture. Also the share of the image's
    source pings used for more than one row and the pings skipped."""
    from blueboat_gcs.config.settings import load_config
    from blueboat_gcs.core.seabed_imager import SeabedImager, feed_pings
    cfg = load_config()
    cfg.along_track.window_s = float(window_s)
    cfg.mosaic.nadir_contrast = False
    cfg.seabed.row_geometry = "square"
    imager = SeabedImager(cfg)
    block_std, jumps, repeat, skip = [], [], [], []
    def on(img):
        n = img.intensity_db.shape[0]
        if n < cfg.seabed.rows:
            return
        xy = img.row_pose[:, :2]
        step = np.hypot(*np.diff(xy, axis=0).T)
        nb = (n - 1) // block_rows
        r = np.array([step[k * block_rows:(k + 1) * block_rows].sum()
                      / (block_rows * img.bin_pitch_m) for k in range(nb)])
        r = r[np.isfinite(r) & (r > 0)]
        if r.size >= 2:
            block_std.append(float(np.std(r)))
            jumps.append(float(np.max(np.abs(np.diff(np.log(r))))))
        pi = np.asarray(img.row_ping_index)
        u, c = np.unique(pi, return_counts=True)
        span = int(pi.max() - pi.min() + 1)
        repeat.append(float(np.mean(c > 1)))
        skip.append(1.0 - u.size / span)
    imager.image_ready.connect(on)
    feed_pings(imager, pings, breaks)
    return {"images": len(block_std), "block_ratio_std": summary(block_std),
            "max_adjacent_block_jump_log": summary(jumps),
            "pings_repeated_share": summary(repeat), "pings_skipped_share": summary(skip)}


def jitter(sets, data, windows=(1.0, 2.0, 5.0)) -> dict:
    from blueboat_gcs.core.svlog import load_svlog
    out = {}
    for name, d in sets.items():
        for log in svlogs_of(name, d, data):
            if log.stat().st_size < 1_000_000:
                continue
            m = load_svlog(log)
            out[f"{name}::{log.name}"] = {f"window_{w:g}s": row_jitter(m.pings, m.gap_times, w)
                                         for w in windows}
    return out


def along_track(sets, data, windows=(1.0, 2.0, 5.0)) -> dict:
    from blueboat_gcs.core.svlog import load_svlog
    out = {}
    for name, d in sets.items():
        for log in svlogs_of(name, d, data):
            if log.stat().st_size < 1_000_000:
                continue
            m = load_svlog(log)
            key = f"{name}::{log.name}"
            out[key] = {"pings": len(m.pings)}
            for w in windows:
                r, c = scale_ratios(m.pings, m.gap_times, w)
                out[key][f"window_{w:g}s"] = {"path_over_nominal": summary(r),
                                             "chord_over_nominal": summary(c)}
    return out


# ------------------------------------------------- 3. roll vs side brightness
def running_median(a, k=101):
    a = np.asarray(a, float)
    out = np.empty_like(a)
    h = k // 2
    for i in range(a.size):
        seg = a[max(0, i - h): i + h + 1]
        seg = seg[np.isfinite(seg)]
        out[i] = np.median(seg) if seg.size else np.nan
    return out


def side_level(db, bin0, pitch, h, R, gain_db):
    if db is None or pitch <= 0:
        return np.nan
    s = (bin0 + np.arange(len(db))) * pitch
    lo, hi = max(2.0 * h, 2.0), 0.7 * R
    m = (s >= lo) & (s <= hi) & np.isfinite(db)
    if m.sum() < 10:
        return np.nan
    return float(np.median(db[m])) - gain_db


def roll_banding(logs: List[Path]) -> dict:
    from blueboat_gcs.core.svlog import load_svlog
    out = {}
    for log in logs:
        m = load_svlog(log)
        roll, diff = [], []
        for p in m.pings:
            if p.port_db is None or p.stbd_db is None or not math.isfinite(p.roll):
                continue
            h = max(p.bottom_slant_m, 0.3)
            R = p.slant_range_m
            gp = 20 * math.log10(p.port_analog_gain / 74.55) if p.port_analog_gain > 0 else 0.0
            gs = 20 * math.log10(p.stbd_analog_gain / 74.55) if p.stbd_analog_gain > 0 else 0.0
            pitch_s = p.stbd_bin_size_m if p.stbd_bin_size_m > 0 else p.bin_size_m
            lp = side_level(p.port_db, p.port_bin0, p.bin_size_m, h, R, gp)
            ls = side_level(p.stbd_db, p.stbd_bin0, pitch_s, h, R, gs)
            roll.append(math.degrees(p.roll))
            diff.append(lp - ls)
        roll, diff = np.asarray(roll), np.asarray(diff)
        ok = np.isfinite(roll) & np.isfinite(diff)
        roll, diff = roll[ok], diff[ok]
        if roll.size < 500:
            out[log.name] = {"pings": int(roll.size), "note": "too few pings with attitude"}
            continue
        rd = roll - running_median(roll)
        dd = diff - running_median(diff)
        ok = np.isfinite(rd) & np.isfinite(dd)
        rd, dd = rd[ok], dd[ok]
        r = float(np.corrcoef(rd, dd)[0, 1]) if rd.std() > 0 and dd.std() > 0 else None
        slope = float(np.polyfit(rd, dd, 1)[0]) if rd.std() > 0 else None
        out[log.name] = {"pings": int(rd.size), "roll_std_deg": float(np.std(roll)),
                         "roll_detrended_std_deg": float(rd.std()),
                         "side_diff_detrended_std_db": float(dd.std()),
                         "corr_roll_vs_port_minus_stbd": r,
                         "slope_db_per_deg": slope,
                         "explained_db_std": (abs(slope) * float(rd.std())
                                              if slope is not None else None)}
    return out


# ----------------------------------- 5. raw level vs x = s/h, and the PNG drift
X_EDGES = np.geomspace(1.0, 20.0, 15)


def level_curves(sets) -> dict:
    out = {}
    for name, d in sets.items():
        files = list_images(d)
        per_bin = [[] for _ in range(len(X_EDGES) - 1)]
        acq = set()
        for f in files[:: max(1, len(files) // 40)]:          # <= ~40 images per set
            if not f.npz.is_file():
                continue
            img = SeabedImageInput.from_session_files(f.json, f.npz, stream_id="s")
            H, W = img.shape
            port, s = geo.column_slant(W, img.bin_pitch_m)
            h = np.maximum(img.rows.bottom, 0.3)[:, None]
            x = s[None, :] / h
            g = np.where(port[None, :], img.rows.gain_port[:, None], img.rows.gain_stbd[:, None])
            db = img.db - g
            acq.add((round(float(img.rows.range.max())), W))
            for k in range(len(X_EDGES) - 1):
                m = (x >= X_EDGES[k]) & (x < X_EDGES[k + 1]) & np.isfinite(db)
                if m.any():
                    per_bin[k].append(float(np.median(db[m])))
        centers = np.sqrt(X_EDGES[:-1] * X_EDGES[1:])
        out[name] = {"real": is_real(name),
                     "acquisition_range_m_cols": sorted(acq),
                     "x": [round(float(c), 2) for c in centers],
                     "median_db": [float(np.median(v)) if v else None for v in per_bin]}
    return out


def png_drift(sets) -> dict:
    from blueboat_gcs.core.display_model import DisplayModelSnapshot
    out = {}
    for name, d in sets.items():
        if is_real(name):
            continue
        files = [f for f in list_images(d) if f.npz.is_file()]
        if len(files) < 5:
            continue
        ref = SeabedImageInput.from_session_files(files[0].json, files[0].npz, stream_id="s")
        if ref.meta.get("display_model") is None:
            continue
        gains = np.stack([ref.rows.gain_port, ref.rows.gain_stbd], axis=1).astype(np.float32)
        seabed = np.isfinite(ref.db)
        port, s = geo.column_slant(ref.shape[1], ref.bin_pitch_m)
        seabed &= s[None, :] >= ref.rows.bottom[:, None]
        meds, his = [], []
        for f in files:
            with open(f.json, encoding="utf-8") as fh:
                dm = json.load(fh).get("display_model")
            if dm is None:
                continue
            snap = DisplayModelSnapshot.from_json(dm)
            png = snap.to_png8(snap.normalise(ref.db, ref.bin_pitch_m,
                                              ref.rows.bottom, gains))
            meds.append(float(np.median(png[seabed])))
            his.append(float(dm.get("hi_db", np.nan)))
        if meds:
            out[name] = {"images": len(meds),
                         "same_input_png_median": summary(meds),
                         "same_input_png_median_spread": max(meds) - min(meds),
                         "hi_db": summary(his)}
    return out


# ------------------------------------------------------- 6. live vs offline
def live_vs_offline(sets, data) -> dict:
    import cv2
    from blueboat_gcs.config.settings import load_config
    from blueboat_gcs.core.display_model import DisplayModel
    from blueboat_gcs.core.seabed_imager import SeabedImager, feed_pings
    from blueboat_gcs.core.svlog import load_svlog
    out = {}
    for name, d in sets.items():
        if is_real(name) or name.startswith("beach_sept"):
            continue
        logs = [p for p in svlogs_of(name, d, data) if p.stat().st_size > 1_000_000]
        files = [f for f in list_images(d) if f.npz.is_file()]
        if not logs or len(files) < 3:
            continue
        m = load_svlog(logs[0])
        cfg = load_config()
        model = DisplayModel.fit(cfg, m.pings)
        imager = SeabedImager(cfg, model=model)
        off = []
        imager.image_ready.connect(off.append)
        feed_pings(imager, m.pings, m.gap_times)
        if not off:
            continue
        t_off = np.array([float(i.row_t[0]) for i in off])
        nan_live, nan_off, dpng, dfixed = [], [], [], []
        from blueboat_gcs.core import perception_bridge as bridge
        from blueboat_perception.core.preprocess import FixedGray
        fixed = FixedGray()
        for f in files:
            live = SeabedImageInput.from_session_files(f.json, f.npz, stream_id="s")
            k = int(np.argmin(np.abs(t_off - float(live.rows.t[0]))))
            if abs(t_off[k] - float(live.rows.t[0])) > 1.0:
                continue
            o = off[k]
            W = live.shape[1]
            _, s = geo.column_slant(W, live.bin_pitch_m)
            near = s <= 0.75
            nan_live.append(float(np.mean(~np.isfinite(live.db[:, near]))))
            if o.intensity_db.shape[1] != W or not f.png.is_file():
                continue
            nan_off.append(float(np.mean(~np.isfinite(o.intensity_db[:, near]))))
            png_live = cv2.imread(str(f.png), cv2.IMREAD_GRAYSCALE)
            png_off = o.to_png8()
            seab_live = s[None, :] >= live.rows.bottom[:, None]
            seab_off = s[None, :] >= np.asarray(o.row_bottom, float)[:, None]
            dpng.append(float(np.median(png_live[seab_live]))
                        - float(np.median(png_off[seab_off])))
            # the same pair through the fixed detector-input mapping (6.4)
            o_in = SeabedImageInput.from_wire(bridge.envelope(o, None, None),
                                              np.asarray(o.intensity_db, np.float32))
            dfixed.append(float(np.median(fixed.map_db(live)[seab_live]))
                          - float(np.median(fixed.map_db(o_in)[seab_off])))
        out[name] = {"paired_images": len(nan_live),
                     "nadir_nan_share_live": summary(nan_live),
                     "nadir_nan_share_offline": summary(nan_off),
                     "seabed_png_median_live_minus_offline": summary(dpng),
                     "seabed_fixed_gray_median_live_minus_offline": summary(dfixed)}
    return out


def input_levels(sets) -> dict:
    """Per set: median seabed grey level of the two detector inputs (6.4)."""
    import cv2
    from blueboat_perception.core.preprocess import FixedGray
    fixed = FixedGray()
    out = {}
    for name, d in sets.items():
        files = [f for f in list_images(d) if f.npz.is_file()]
        fx, idn = [], []
        for f in files[:: max(1, len(files) // 40)]:
            img = SeabedImageInput.from_session_files(f.json, f.npz, stream_id="s")
            _port, s = geo.column_slant(img.shape[1], img.bin_pitch_m)
            seab = np.isfinite(img.db) & (s[None, :] >= np.maximum(img.rows.bottom[:, None], 0.75))
            if not seab.any():
                continue
            fx.append(float(np.median(fixed.map_db(img)[seab])))
            if f.png.is_file():
                idn.append(float(np.median(cv2.imread(str(f.png), cv2.IMREAD_GRAYSCALE)[seab])))
        out[name] = {"real": is_real(name), "images": len(fx),
                     "fixed_gray_seabed_median": summary(fx),
                     "identity_png_seabed_median": summary(idn)}
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--data", type=Path, default=Path.home() / "ros2_ws" / "data" / "SSS_data")
    ap.add_argument("--extra-session", type=Path, nargs="*", default=[])
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--only", nargs="*", default=None,
                    help="subset of: turns along_track roll levels drift live_offline")
    a = ap.parse_args(argv)
    if str(a.out.resolve()).startswith(str(a.data.resolve())):
        print("refusing an --out inside the data tree (CM-7)")
        return 2
    a.out.mkdir(parents=True, exist_ok=True)
    from PySide6.QtCore import QCoreApplication
    app = QCoreApplication.instance() or QCoreApplication([])  # noqa: F841
    sets = image_sets(a.data, a.extra_session)
    want = set(a.only or ["turns", "ping_turns", "along_track", "roll", "levels", "drift",
                          "live_offline"])
    res = {"sets": sorted(sets)}
    if "turns" in want:
        res["turns"] = turns(sets)
    if "levels" in want:
        res["levels"] = level_curves(sets)
    if "drift" in want:
        res["drift"] = png_drift(sets)
    logs = [a.data / "beach_sept" / n for n in ("bath1.svlog", "bath2.svlog", "bath3.svlog",
                                                 "object_boiteVsDistance.svlog")]
    logs += [p for e in a.extra_session for p in Path(e).rglob("*.svlog")]
    logs = [p for p in logs if p.is_file() and p.stat().st_size > 1_000_000]
    if "roll" in want:
        res["roll"] = roll_banding(logs)
    if "ping_turns" in want:
        res["ping_turns"] = ping_turns(logs)
    if "along_track" in want:
        res["along_track"] = along_track(sets, a.data)
    if "input_levels" in want:
        res["input_levels"] = input_levels(sets)
    if "jitter" in want:
        res["jitter"] = jitter(sets, a.data)
    if "live_offline" in want:
        res["live_offline"] = live_vs_offline(sets, a.data)
    (a.out / "phase4.json").write_text(json.dumps(res, indent=1), encoding="utf-8")
    print(f"wrote {a.out / 'phase4.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
