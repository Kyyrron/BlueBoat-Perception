# BlueBoat-Perception — interface contract

**Status: implemented 2026-10-16 (Phase 2); binding.** Every change from now on is
additive (CM-1).

**Conventions used throughout:**
- **World frame:** local ENU metres of `/blueboat/odom`, the frame of the GCS map's world
  layers (P2 §4.6). x = East, y = North. The real origin is the boat at launch; in
  Gazebo it is the Gazebo origin.
- **Headings and yaw:** radians, ENU, CCW from +x, wrapped to (−π, π].
- **Time:** seconds, the **ROS time stamped on the sonar profiles**. That is system time
  on the real boat and **sim time under Gazebo**. All row and detection times share this
  clock.
- **Wall-clock values** appear only under `provenance` and in the status topic.
- **Pixels:** `(row, col)`. Row 0 is the **newest** row; col 0 is far-range port, the last
  col far-range starboard, and the transducer sits at the centre seam (the GCS
  convention, P4).
- **Absent values:** JSON `null` (never NaN, never invented).

---

## 1. Topics

| Topic | Type | Producer → consumer | QoS | Rate |
|---|---|---|---|---|
| `/sss_ai/seabed_image` | `sensor_msgs/Image` (`32FC1`) | GCS → Perception | RELIABLE, KEEP_LAST 32, VOLATILE | one per seabed image (0.1–0.36 Hz measured; ≤ ~1 Hz) |
| `/sss_ai/seabed_image/png` | `sensor_msgs/Image` (`mono8`) | GCS → Perception | same | same |
| `/sss_ai/seabed_image/metadata` | `std_msgs/String` (JSON, §2) | GCS → Perception | same | same |
| `/sss_ai/detections` | `vision_msgs/Detection2DArray` (§3) | Perception → GCS (`DetectionsListener`, existing) | default (RELIABLE, KEEP_LAST 10), as the listener | one array per image, may be empty |
| `/sss_ai/image_records` | `std_msgs/String` (JSON, §4) | Perception → Belief (future), analysis | RELIABLE, KEEP_LAST 100, VOLATILE | one per image |
| `/sss_ai/perception/status` | `std_msgs/String` (JSON, §5) | Perception → operator, tests | RELIABLE, KEEP_LAST 1, TRANSIENT_LOCAL | 1 Hz |

Notes:
- All of these sit under `/sss_ai/`, which Control's run bag excludes
  (`run_bag.py:52`).
- Existing topics, unchanged: `/sss_ai/seabed_analysis` (GCS, JSON) keeps publishing; its
  `detections` list is always empty since the GCS has no detector of its own (one
  detection source: Perception, 2026-10-16).
- **Perception subscribes to nothing else.** In particular it never subscribes to
  `/side_scan_sonar/*`, `/sss_processor/*`, `/blueboat/*`, or any ground-truth or sea-state
  topic.
- **Image key:** `Image.header.frame_id` = `"<stream_id>/<image_id>"`. This is an image
  key, **not a TF frame**; no module reads TF (P2 §4.6). `Image.header.stamp` = the
  newest row time (`t_start_s`). The same key appears in the metadata envelope.
- **Image layout:** `height` = H rows, `width` = W columns, `is_bigendian` = 0;
  `step` = 4·W (32FC1) or W (mono8). `32FC1` values are dB, the exact
  `_world.npz:intensity_db`; NaN means no sample (water-column cut, nadir blank, absent
  side, beyond the extent). `mono8` is the exact `SeabedImage.to_png8()` (= the session
  PNG).

## 2. `/sss_ai/seabed_image/metadata` (produced by SSS; consumed here)

```json
{"schema": "blueboat_sss/seabed_image/1",
 "key": "gcs-20261015T140231Z-3f9a/12",
 "stream_id": "gcs-20261015T140231Z-3f9a",
 "image_id": 12,
 "stamp": {"sec": 1760537000, "nanosec": 123000000},
 "png_path": ".../seabed_00012.png",            // null when no recording session is active
 "metadata_path": ".../metadata/seabed_00012.json",
 "metadata": { … exactly SeabedImage.metadata(), schema 5 … }}
```

**`metadata`, schema 5** (schema 4 + the additive keys the SSS changes add; schema 4
sessions on disk lack the starred keys):

| Key | Unit | Meaning |
|---|---|---|
| `schema` | — | 5 (4 on older sessions) |
| `image_id`, `rows` (H), `cols` (W) | — | |
| `stream_id`★ | — | stream identifier, generated at each GCS START |
| `t_start_s` / `t_end_s` | s | time of row 0 (**newest**) / of the last row (oldest) |
| `bin_pitch_m` | m | slant metres per column |
| `row_geometry` | — | `square` (default) or `ping` |
| `georef`★ | — | `{"convention": 2, "altitude": "bottom_m", "lever_arm_m": {"x": 0.0, "y_port": 0.285, "y_stbd": 0.285}}`; absent = convention 1 |
| `display_model`, `display_note`, `pixel_convention` | — | as today (P4) |
| `rows_data[i]` | | per picture row (newest first): |
| …`t_s` | s | row time |
| …`x_m`, `y_m` | m | boat position, world frame (row-interpolated in square geometry) |
| …`yaw_rad` | rad | heading (live: compass at reception; replay: ATTITUDE yaw) |
| …`range_m` | m | configured half-range |
| …`speed_mps` | m/s | along-track speed estimate (5 s median) |
| …`altitude_m` | m | `water_depth` (altitude + 0.15 m submersion live; depth-mode value offline). Used by **convention 1** |
| …`bottom_m` | m | tracked transducer altitude (FBR), every depth mode. Used by **convention 2** |
| …`ping_index`, `along_track_m` | —, m | source ping within the window; along-track position |
| …`gain_port_db`, `gain_stbd_db` | dB | receive-gain step removed by the display model |
| …`roll_rad`★, `pitch_rad`★ | rad | from `/mavros/imu/data` (mavros real / shim sim), latest at the ping's reception; `null` if none |
| …`yaw_rate_rad_s`★ | rad/s | IMU body z rate; `null` if none |
| …`attitude_age_s`★ | s | age of that IMU sample at the ping's reception (wall clock) |
| …`port_ping_number`★, `stbd_ping_number`★ | — | device ping numbers of the row's source ping; 0 = side absent |
| `boat` | — | summary (as today) |
| `detections` | — | always `[]` since 2026-10-16 (schema ≤ 4 sessions: the station's former in-process dummy) |

## 3. `/sss_ai/detections` — `vision_msgs/Detection2DArray`

This is the GCS's existing contract (`blueboat_gcs/ros/detections_listener.py`), filled as
follows:

| Field | Value |
|---|---|
| `header.stamp` | the image's newest row time; `header.frame_id` = `"odom"` |
| `detections[k].header.stamp` | the detection's **row time** (`t_row_s`). The GCS uses it for the waterfall marker (additive listener change) |
| `detections[k].id` | decimal string `2000000 + 16·image_id + k` (k < 16; `1000000000 + 1000·image_id + k` beyond), the station uid. The station's former in-process dummy used `1000000 + …` |
| `detections[k].bbox.center.position.x/.y` | world x, y [m] |
| `detections[k].bbox.center.theta` | boat heading at the row [rad] |
| `detections[k].bbox.size_x / size_y` | extent across / along [m] (`extent_m`); a point detection (one-pixel box, the dummy) is drawn at 1.0 / 1.0, as the station's former dummy |
| `detections[k].results[0].hypothesis.class_id` | class name (string), e.g. `dummy_center` |
| `detections[k].results[0].hypothesis.score` | `confidence`: the calibrated probability when `calibrated`, else the raw score (dummy 0.5) |
| `detections[k].results[0].pose.pose.position` | world x, y (z = 0) |
| `detections[k].results[0].pose.covariance` | 6×6 row-major; [0] = σxx, [1] = σxy, [6] = σyx, [7] = σyy [m²]; others 0 |

## 4. `/sss_ai/image_records` — `blueboat_perception/image_record/1`

One record per image, **including images without detections**. Canonical JSON (sorted
keys, compact, no NaN).

```json
{"schema": "blueboat_perception/image_record/1",
 "key": "<stream_id>/<image_id>", "stream_id": "…", "image_id": 12,
 "source": "live",                       // live | offline
 "input": {"schema": "blueboat_sss/seabed_image/1", "metadata_schema": 5,
           "georef_convention": 2,
           "georef": {"convention": 2, "altitude": "bottom_m",
                      "lever_arm_m": {"x": 0.0, "y_port": 0.285, "y_stbd": 0.285}},
           "row_geometry": "square",
           "rows": 256, "cols": 2402, "bin_pitch_m": 0.01668},
 "processing": {"perception_version": "0.1.0",
                "preprocess": {"name": "fixed_gray",       // or "identity" (the display PNG)
                               "params": {"near_range_mask_m": 0.0, "mapping": "fixed_gray_v1",
                                          "mapping_sha256": "…", "window_db": [-31.15, 17.16]}},
                "detector": {"name": "dummy_center", "version": "1", "dummy": true,
                             "status": "ran"},          // ran | no_pixels
                "calibration": {"name": "identity", "params": {}, "calibrated": false},
                "uncertainty": {"model": "geometric_v1", "sigma_h_m": 0.10,
                                "sigma_heading_rad": 0.0349, "sigma_nav_m": 1.0,
                                "beam_azimuth_rad": 0.00873, "assumed": true}},
 "time": {"t_min_s": 32.161, "t_max_s": 44.490, "n_rows": 256,
          "clock": "ros_time_of_sonar_profiles"},
 "track": {"t_s": […], "x_m": […], "y_m": […], "heading_rad": […],
           "speed_mps": […], "altitude_m": […], "bottom_m": […],
           "georef_altitude_m": […],
           "roll_rad": […|null], "pitch_rad": […|null],
           "yaw_rate_rad_s": […], "yaw_rate_source": "imu|derived",
           "look_port_rad": […], "look_stbd_rad": […],
           "port_ping_number": […|null], "stbd_ping_number": […|null],
           "yaw_rate_smoothed_rad_s": […], "turn_radius_m": […|null],
           "turn_inner_side": ["port"|"starboard"|null, …], "backscan_from_m": […|null]},
 "sides": {"port": {"present_rows": 256, "seabed_rows": 256,
                    "slant_range_m": [0.75, 20.0], "ground_range_m": [0.0, 19.97],
                    "polygon_xy": [[x, y], …]},
           "starboard": {…}},
 "footprint": {"outline_xy": [[x, y], …], "row_step": 16},
 "summary": {"speed_mps": {"min": …, "mean": …, "max": …},
             "altitude_m": {"min": …, "mean": …, "max": …},
             "heading_rad": {"start": …, "end": …},
             "max_abs_yaw_rate_rad_s": …,
             "one_sided_rows": 0, "display_model_ready": true,
             "detector_input": {"gray": true, "valid_share": 0.97},
             "turn": {"window_s": 1.0, "criterion": "…", "rows_rstar_lt_range": 0,
                      "rows_rstar_lt_half_range": 0, "share_rstar_lt_range": 0.0}},
 "detections": [ {…}, … ],               // [] = searched, nothing found; null = not run (no_pixels)
 "provenance": {"received_wall_utc": "…", "host": "…", "join_status": "complete",
                "latency_ms": {"join": …, "pipeline": …, "publish": …, "total": …},
                "stream_stats": {"completed": …, "gaps": …, "partial_no_pixels": …,
                                 "partial_no_png": …, "dropped_no_metadata": …,
                                 "duplicates": …, "late_parts": …, "last_image_id": …}}}
```

Offline records carry `"source": "offline"` and `"provenance": {"session": …,
"metadata_path": …}`. Everything outside `source` and `provenance` is byte-identical
between a live record and the offline record of the same saved image.

| Field | Unit / frame | Meaning |
|---|---|---|
| `track.*` | per row, **row 0 newest**, aligned with the image rows | boat state at each row |
| `track.heading_rad` | rad, ENU | = metadata `yaw_rad` |
| `track.georef_altitude_m` | m | the altitude the image's convention uses per row (conv. 1 `altitude_m`, conv. 2 `bottom_m`), ≥ 0 |
| `track.port_ping_number` / `stbd_ping_number` | — | the rows' device ping numbers (schema ≥ 5; 0 = side absent); `null` on older images |
| `track.yaw_rate_smoothed_rad_s` | rad/s | the yaw rate averaged over `turn_window_s` of row time (decision 6.2) |
| `track.turn_radius_m` | m | `r* = v/\|ω\|` (v = row speed, ω = the smoothed rate); `null` when not turning or not moving |
| `track.turn_inner_side` | — | `port` (ω > 0, a left turn) / `starboard` / `null` |
| `track.backscan_from_m` | m | `r*` when `r* < range_m`: beyond it the inner side is scanned backwards [Cobra et al. 1992 §I.A]; `null` otherwise. **A flag: no row is ever removed** |
| `summary.turn` | — | counts of rows with `r* < R` and `r* < R/2`, their share, the window and the criterion |
| `summary.detector_input` | — | whether a detector-input picture was built (`gray`) and the share of valid, unmasked pixels |
| `processing.preprocess` | — | the detector input used: `fixed_gray` (the fixed mapping, with its file name and SHA-256) or `identity`; `near_range_mask_m` (decisions 6.4, 6.5) |
| `track.look_port_rad` / `look_stbd_rad` | rad, ENU | look direction (boat → seabed) = heading ± π/2 |
| `track.yaw_rate_rad_s` | rad/s | IMU (schema ≥ 5) or the finite difference of row yaw (`derived`) |
| `sides.*.present_rows` | rows | rows where that side has ≥ 1 finite sample |
| `sides.*.seabed_rows` | rows | rows where that side has ≥ 1 valid seabed sample (below) |
| `sides.*.slant_range_m` | m | [min, max] slant range of valid seabed samples (finite and `s ≥ altitude`) |
| `sides.*.ground_range_m` | m | the same, through the image's georef convention (lever arm included in conv. 2) |
| `sides.*.polygon_xy` | m, world | near and far valid-seabed edges every `row_step` rows (+ first and last), closed. **The seabed this image searched on that side** |
| `footprint.outline_xy` | m, world | port far edge + starboard far edge ring; includes the nadir gap (not searched) |
| `summary.one_sided_rows` | rows | rows with one side entirely NaN |
| `detections` | | `[]` means searched, nothing found (negative evidence); `null` means pixels missing, so **unknown** |

**Detection object** (inside `detections`):

| Field | Unit / frame | Meaning |
|---|---|---|
| `detection_id` | — | `"<stream_id>/<image_id>/<k>"`, globally unique |
| `uid` | — | station uid (§3) |
| `image_id`, `stream_id` | — | |
| `class_name`, `class_id` | — | dummy: `dummy_center`, 0 |
| `confidence` | [0, 1] | calibrated probability if `calibrated`, else = `confidence_raw` |
| `confidence_raw` | — | detector score; **not a probability** for YOLO-type detectors [R10 §2] |
| `calibrated` | bool | false until a calibrator is fitted (P1 §8.2 ⚠) |
| `dummy` | bool | true for the dummy detector |
| `pixel` | [row, col] | box centre |
| `bbox_px` | [r0, c0, r1, c1] | inclusive pixel box (dummy: degenerate) |
| `world_xy` | m, world | through the image's georef convention: the station's own world grid at that pixel (float32 precision, bit-exact) |
| `extent_m` | m | [across, along] drawn extent: 1.0 / 1.0 for a point detection; else the box's slant-bin width and along-track rows |
| `covariance_xy` | m², world | 2×2 `[[σxx, σxy], [σxy, σyy]]`, model `geometric_v1` (DESIGN §4.6) |
| `t_row_s` | s | time of its row |
| `side` | — | `port` / `starboard` |
| `slant_range_m`, `ground_range_m` | m | of the box centre |
| `look_azimuth_rad` | rad, ENU | direction boat → object at that row |
| `grazing_angle_rad` | rad | `asin(min(h/s, 1))`, flat floor [R6 eq. 1] |
| `boat_heading_rad`, `boat_speed_mps`, `boat_altitude_m` | rad, m/s, m | at its row (altitude = the convention's altitude) |
| `boat_roll_rad`, `boat_pitch_rad` | rad | at its row; `null` if unavailable |
| `turn_radius_m` | m | `r*` at its row, or `null` |
| `turn_backscan` | bool | on the inner side of a turn and beyond `r*`: the object may be re-imaged or folded. **A flag, never a rejection** |

## 5. `/sss_ai/perception/status` — `blueboat_perception/status/1`

```json
{"schema": "blueboat_perception/status/1", "perception_version": "0.1.0", "mode": "normal",
 "dataset": null,                         // dataset mode: {"name", "dir", "images", "skipped_no_png"}
 "detector": {"name": "dummy_center", "dummy": true},
 "streams": {"<stream_id>": {"completed": 120, "gaps": 0, "partial_no_pixels": 0,
             "partial_no_png": 0, "dropped_no_metadata": 0, "duplicates": 0,
             "late_parts": 0, "last_image_id": 119}},
 "pending": 0, "records_published": 120, "input_errors": 0,
 "latency_ms": {"join": {"n", "p50", "p95", "max"}, "pipeline": {…}, "publish": {…},
                "total": {…}},
 "record_log": {"<stream_id>": "~/ros2_ws/data/Perception_data/live/<stream_id>.jsonl"},
 "wall_utc": "…"}
```

## 6. Node parameters and launch arguments

| Name | Default | Meaning |
|---|---|---|
| `mode` | `normal` | `normal` or `dataset` (DESIGN §7) |
| `dataset_name` | `""` | `<YYYY_MM_DD-HH_MM_SS>-<note>`; dataset mode (the launch file time-stamps a missing one) |
| `dataset_path` | `""` | the dataset folder the launch file claimed (dataset mode); else the node claims `<root>/data/Dataset_data/<dataset_name>` |
| `world_dir` | `""` | recorded in the dataset manifest (for the future offline labeller) |
| `data_dir` | `""` | the run-data root `<root>` (else `$BLUEBOAT_DATA_DIR`, then the workspace, then the cwd); outputs under `<root>/data/` |
| `record_log` | `true` | write `Perception_data/live/<stream_id>.jsonl` |
| `join_timeout_s` | 5.0 | release incomplete image keys after this |
| `detector` | `dummy_center` | detector plug-in |
| `preprocess` | `fixed_gray` | the detector input: `fixed_gray` (one fixed mapping of the raw dB, decision 6.4) or `identity` (the GCS display PNG, the baseline). `roll_correction` and the deferred options (geometric correction, despeckling, histogram equalisation, Retinex) raise |
| `detector_input_mapping` | `fixed_gray_v1` | the `fixed_gray` parameter file (packaged `core/data/<name>.json`, or a path) |
| `near_range_mask_m` | 0.0 | mask the detector input below this slant range; 0.75 = the GCS's live nadir blank (decision 6.5) |
| `turn_window_s` | 1.0 | yaw-rate smoothing for the turn flags (decision 6.2) |
| `calibration` | `identity` | calibrator plug-in |
| `sigma_h_m`, `sigma_heading_deg`, `sigma_nav_m` | 0.10, 2.0, 1.0 | **assumed** uncertainty parameters (§7) |
| `footprint_row_step` | 16 | polygon decimation |
| `topic_image`, `topic_png`, `topic_metadata`, `topic_detections`, `topic_records`, `topic_status` | §1 | topic names |

## 7. What the metadata does not provide (missing for Belief or labels)

| Missing | Effect | Path |
|---|---|---|
| Heave | not in records | not measured on the real boat |
| Navigation and heading uncertainty | covariance uses **assumed** σ_nav, σ_ψ, σ_h | measure against USBL / RTK (P1 §8.4) |
| Time-accurate attitude | Q8 attaches the latest IMU sample at the GCS's reception, with its age; mavros header stamps may be 0 on the real boat (P2 §4.1) | Phase 4 latency measurement (the Control request was withdrawn 2026-10-16) |
| Full attitude in `/blueboat/odom` (real) | the ping quaternion is yaw-only on the real boat; the GCS uses `/mavros/imu/data` instead | handled separately by you (2026-10-16) |
| GPS lat/lon per row | records are in local ENU; georeferencing to lat/lon is the GCS anchor's job | later, if Belief needs absolute positions |
| Object height | grazing angle and range assume a flat floor (R6) | inherent to single-beam SSS |
| Ground-truth labels | dataset labels empty | a future offline labeller over the dataset's `ground_truth_bag/` |
| Images produced while not recording | exist only live; offline replay covers recorded images only | by design (NC #9) |
