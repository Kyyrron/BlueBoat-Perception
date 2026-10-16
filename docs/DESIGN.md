# BlueBoat-Perception — design

**Status: built in Phase 2 (2026-10-16), as approved with your answers of 2026-10-16
(§2).** Where the build refined a Phase 1 detail, this file says what was built.

Companion documents:
- [`topics.md`](topics.md): the interface contract, every field;
- [`PREPROCESSING_STUDY.md`](PREPROCESSING_STUDY.md);
- [`REFERENCES.md`](REFERENCES.md);
- [`REQUESTS.md`](REQUESTS.md);
- on the SSS side, `BlueBoat-SSS/blueboat_sss/blueboat_gcs/docs/PERCEPTION_INTEGRATION.md`.

## 1. Purpose and scope

Perception turns **seabed images made by BlueBoat-SSS** into **detection data**:
- detections for the station map;
- one record per image for the future Belief module (P1 §5.1: the belief layer is the only
  object the replanner reads, CM-9).

It is deliberately small:

```
images + metadata  →  [preprocess: fixed_gray]  →  detector: dummy  →  [calibrate: identity]  →  records
```

| In scope now | Later (designed for, not built) |
|---|---|
| ROS-free core; thin ROS node; offline CLI over recorded sessions | YOLO detector loading a model file (`detectors/yolo.py`) |
| Dummy detector reproducing the GCS dummy, labelled dummy everywhere | Confidence calibration on held-out data (temperature / Platt, R11) |
| Detections on `/sss_ai/detections` (existing GCS slot) + per-image records | Labels from simulator ground truth (offline labeller) |
| Dataset-creation **mode** and writer (structure; empty labels hook) | Training (out of scope here) |
| Identity preprocessing stage + the study | Any image-changing preprocessing, after your Phase 4 decision |

**Not Perception's job:**
- reading pings (raw or processed);
- forming images (SSS keeps image formation, the PNG, the display model and the
  windowing);
- reading simulator ground truth online;
- writing into recording sessions (CM-7).

## 2. Decisions (owner's answers, 2026-10-15)

| # | Decision | Consequence here |
|---|---|---|
| Q1 | The GCS publishes every image on ROS topics, recording or not | §3 |
| Q2 | Standard types + JSON; no new interfaces package (CM-1 intact) | §3, §5 |
| Q3 | Dataset mode now; MCS offers *Normal session* / *Dataset creation* on sim launches | §7, §8 |
| Q4 | Preprocessing split by data need: per-ping / geometry steps in SSS, per-image steps in Perception | §4.3 |
| Q5 | Accessible sources only | REFERENCES |
| Q6 | SSS fixes its pixel→world (convention 2: `bottom_m` + 0.285 m lever arm), versioned in the metadata | §4.2 |
| Q7 | MCS starts Perception for personalized-world Gazebo runs (Normal/Dataset) and real-robot sessions (Normal) | §8 |
| Q8 | SSS adds per-row roll / pitch / yaw rate (from `/mavros/imu/data`) and device ping numbers | §4.1, topics.md |

Phase 2 answers (2026-10-16):

| # | Decision | Consequence here |
|---|---|---|
| 1 | The SSS profile-miss fallback fix, separate and labelled | SSS `PERCEPTION_INTEGRATION.md` §5 |
| 2 | Dataset mode also records the simulator ground truth (an archival bag); Perception never reads it | §7, `launch/perception_launch.py` |
| 3 | **One detection source: Perception.** The station's own dummy and any source choice are removed | §3, §8; the dummy (§4.4) is the station's former one, reproduced exactly |
| 4 | MCS changes in this session, additions only, compatible with a later "Replanner mode" row | §8 |
| 5 | Control's live-path-update work must not be affected; Control not edited | it touches nothing Perception uses |
| 6 | Simulation not modified; the simulator and yaw requests withdrawn | `REQUESTS.md` |
| 7 | Dropped sources stay dropped unless Phase 4 needs one | `REFERENCES.md` |

## 3. Data flow and transport

```
BlueBoat-SSS GCS (image formation unchanged; Q6 world-grid fix; Q8 metadata)
  SeabedImager._finish → image_ready → MainWindow._on_seabed_image   (no detector in the GCS)
     ├─ /sss_ai/seabed_analysis (unchanged topic, detections: [])
     └─ every image, recording or not: one single-thread publish job
          /sss_ai/seabed_image            sensor_msgs/Image 32FC1   raw float dB, bit-exact
          /sss_ai/seabed_image/png        sensor_msgs/Image mono8   the session PNG
          /sss_ai/seabed_image/metadata   std_msgs/String JSON      SeabedImage.metadata() + envelope
  GCS map / waterfall markers / left panel / detections.csv  ←  /sss_ai/detections only
  GCS replay "Run AI"  →  this core, in-process, through the same envelope
BlueBoat-Perception node
     JoinBuffer(key) → core.Pipeline.run(image) → ImageRecord (+ DetectionArray)
          /sss_ai/detections        vision_msgs/Detection2DArray  → GCS DetectionsListener (existing)
          /sss_ai/image_records     std_msgs/String JSON          → Belief (future), analysis
          /sss_ai/perception/status std_msgs/String JSON          → operator / tests (1 Hz)
          <data>/Perception_data/live/<stream_id>.jsonl           (record log)
          [dataset mode] <data>/Dataset_data/<name>/              (§7)
Offline CLI: same core over <session>/seabed_images/ → <data>/Perception_data/offline/<session>/
```

### 3.1 Why this transport (Q1/Q2)
- **Pixel-exact.** The 32FC1 image is the same float32 array the GCS saves as
  `_world.npz:intensity_db`. Offline input (npz + JSON) is therefore bit-identical to live
  input, which makes "offline gives the same records as live" a property of the design,
  not a hope.
- **The PNG travels too,** as mono8. Dataset mode *copies* the session picture instead of
  re-implementing the display model (§7). It is also what an annotation tool shows.
- **Standard types only.** `sensor_msgs/Image` for arrays, JSON for evolving metadata
  (the project's pattern for new data, CM-1; `/sss_ai/seabed_analysis` already does this).
  The cost is a three-part join (§3.3), which is unit-tested and counted.
- **QoS: RELIABLE, KEEP_LAST 32, VOLATILE, at both ends (CM-4).** Large samples fragment;
  BEST_EFFORT would lose a whole image to one lost fragment. 32 images is at least 90 s
  of buffer at the measured 0.1–0.36 Hz (STUDY §5).
- **Out of run bags.** Control's run bag excludes `^/(side_scan_sonar|sss_processor|sss_ai)`
  (`run_bag.py:52`), so MB-sized images never enter it. The sonar's archival record stays
  the session (`.svlog` + seabed images), and offline replay regenerates the records (§6).
- **NC #9** ("data leaves the GCS only through recording sessions") concerns persisted
  exports. A live stream is not one; `/sss_ai/seabed_analysis` already streams outside
  sessions. The GCS writes nothing new to disk.

### 3.2 Budget
- One image is 256 × W float32 with W = 1200–2402: **1.2–2.5 MB**, plus 0.3–0.6 MB mono8
  and ~0.1 MB JSON.
- At ≤ 0.36 Hz measured (≈ 1 Hz worst case at 2 m/s, finest pitch) that is **≤ 3 MB/s on
  loopback**, all on the laptop (P2 §1.1).
- `to_png8` + publish run on a dedicated single-thread pool (order kept, GUI thread free);
  Phase 3 measures the cost.

### 3.3 Key, join and loss accounting
- **Key:** `"<stream_id>/<image_id>"`. It is in `Image.header.frame_id` (documented as an
  image key; no module reads TF, P2 §4.6) and in the metadata JSON.
- **`stream_id`:** generated by the GCS at every `SeabedImager.reset()` (START), e.g.
  `gcs-20261015T140231Z-3f9a`. `image_id` restarts at 0 per stream.
- **JoinBuffer (ROS-free, `core/join.py`):**
  - releases a key when `image` and `metadata` are present (and `png` in dataset mode);
  - a key incomplete after `join_timeout_s` (5 s) is released partially:
    - metadata without pixels gives a record with `detector.status = "no_pixels"` and
      `detections: null`. That means *unknown*, not *negative*, for Belief;
    - pixels without metadata cannot be georeferenced: counted and dropped with a warning;
  - counts `completed`, `partial_no_pixels`, `partial_no_png`, `dropped_no_metadata`,
    `duplicates`, `late_parts` (a part arriving after its key was released without it), and
    per-stream `image_id` gaps. They are reported in the status topic, the node log and
    each record's `provenance.stream_stats`.

## 4. Core (ROS-free, deterministic)

Package `blueboat_perception.core`. It imports numpy and the standard library only: no
rclpy, no Qt, no OpenCV in `core/` (PNG encoding happens in the dataset writer via
OpenCV/PIL, lazily). A test sweeps the imports.

### 4.1 Input model — `core/image.py`
`SeabedImageInput` (frozen dataclass), built either `from_wire(meta_json, db_array,
png=None)` or `from_session_files(json_path, npz_path, png_path=None)`. It holds:
- `key`, `stream_id`, `image_id`, `schema`;
- `db: float32[H, W]`, with NaN meaning no sample;
- `png: uint8[H, W] | None`;
- `meta` (the parsed metadata dict);
- per-row arrays extracted once:
  - always: `t, x, y, yaw, altitude, bottom, speed, along, ping_index, gain_p, gain_s`;
  - optional (schema ≥ 5): `roll, pitch, yaw_rate, attitude_age, port_pn, stbd_pn`;
- `bin_pitch_m`, `row_geometry`, and `georef` (convention 1 if absent, else from the
  metadata).

Rules:
- Unknown or newer schemas are accepted when the required keys are present. Missing
  optional keys become `None`, never invented.
- Time ordering: row 0 is the **newest** row; `t_start_s` is the newest time (P4
  convention). Records always report `t_min_s` / `t_max_s`.

### 4.2 Geometry — `core/geometry.py`
- **Pixel → world, one function, two conventions** read from `meta["georef"]`:
  - column `j` of width `W`: `k = W/2 − 1 − j` (port, `j < W/2`) or `j − W/2`
    (starboard); `s = (k + 0.5)·bin_pitch_m`; sign `+` port, `−` starboard;
  - **convention 1** (all sessions before the Q6 fix; `georef` absent):
    `y = sign·sqrt(max(s² − altitude_m², 0))`;
  - **convention 2** (Q6): `y = sign·(lever_y_side + sqrt(max(s² − bottom_m², 0)))`, with
    `lever_y_side` from `georef.lever_arm_m`;
  - both: `wx = x − sin(yaw)·y`, `wy = y_row + cos(yaw)·y` (the flat-seabed projection,
    R1 eq. 3.2; R6 eq. 2 with hp = 0).
- **Tested against the station's own conversion.**
  - Convention 1: the `world_x/world_y` grids of the `_world.npz` files on disk. This
    covers real `beach_sept/seabed_images_object_boiteVsDistance` and
    `merged_sessions/bath1_2`, plus the seven live sim sessions with non-zero altitude.
  - Convention 2: grids produced by the fixed imager (SSS test fixtures and a fresh sim
    session).
  - Tolerance ≤ 1 float32 ulp, with the exact-match share reported. The GCS venv runs
    numpy 2.x, tests run numpy 1.26, so `sin`/`cos` may differ in the last bit.
- **Derived per row and side:**
  - the valid-seabed column span: finite `db` and `s ≥ altitude`;
  - `slant_range_m [min, max]` and `ground_range_m [min, max]`;
  - the footprint polygon (near / far edges every `footprint_row_step` rows, ends kept);
  - look azimuth: port `yaw + π/2`, starboard `yaw − π/2`, ENU CCW from +x;
  - row yaw rate: from the Q8 field when present, else the finite difference of the
    unwrapped row yaw over row time, labelled `derived`.
- **Per detection:**
  - side, slant and ground range;
  - grazing angle `asin(min(h/s, 1))` (R6 eq. 1, flat floor);
  - look azimuth;
  - boat heading, speed, altitude, roll and pitch at its row.

### 4.3 Preprocessing — `core/preprocess.py` (decisions of 2026-10-16, STUDY §6)
- **Protocol:** `Preprocessor.run(img) -> DetectorInput`, plus `name`, `params`
  (recorded in every record). `DetectorInput` carries the originals untouched (`db`,
  `png`), the detector's uint8 input picture `gray`, and `valid` (real, unmasked samples).
- **`fixed_gray` (the default, 6.4):** one fixed mapping of the raw dB,
  `gray = round(255·clip((db − g − R(s) − lo)/(hi − lo), 0, 1))`.
  - The parameters come from a versioned file (`core/data/fixed_gray_v1.json`), fitted
    once on real data by `tools/fit_detector_input.py`.
  - No per-image statistics and no TVG law.
- **`identity` (the baseline, 6.1):** the detector input is the GCS display PNG as is.
- **`near_range_mask_m` (6.5):** independent of the mapping; 0.0 (off) or e.g. 0.75.
- **Disabled boundaries (6.6, 6.7):** `roll_correction=True` and the deferred options
  (geometric correction, despeckling, histogram equalisation, Retinex) raise.
- **Placement (Q4).** Options needing per-ping data or changing geometry belong to SSS's
  imager, so the saved images are the detector input's source. Per-image options live
  here and run identically live, offline and in dataset mode.
- **Rule.** A preprocessor never modifies its input arrays and never invents samples
  without exposing a mask (P5 §5).

### 4.4 Detectors — `core/detectors/`
- **Protocol:** `Detector.detect(inp) -> list[RawDetection]`.
  - `RawDetection = {bbox_px (r0, c0, r1, c1), center_px (row, col), class_id,
    class_name, score_raw, logit (optional)}`.
  - Also `name`, `version`, `is_dummy`.
- **`DummyCenter` (now):**
  - centre pixel `(H//2, W//2)`, class `dummy_center`, score 0.5, a zero-size box;
  - **reproduces `dummy_center_analyzer` exactly** (pixel, then world through the same
    convention, then row time). A test checks this against `detections` in the saved
    metadata on all on-disk sessions;
  - `is_dummy = True` puts `"dummy": true` in every record and detection.
- **`YoloDetector` (stub; interface only).** Constructor takes `model_path`, `input_size`,
  `tile_stride`, `score_threshold`, `nms_iou`; `detect` raises `NotImplementedError`.
  Documented constraints:
  - YOLO resizes its input to a fixed square [R10 fig. 1], and our tiles are 256 × W
    (W up to 2402), so the adapter must **tile** across-track, never squash, and merge
    across tiles;
  - box format: YOLO-normalised `xywh` [R10 §2.2];
  - its score is `Pr(Object)·IOU` or `Pr(Class)·IOU`, **not a calibrated probability**
    [R10 §2 eq. 1], hence §4.5;
  - weaker on small objects and groups [R10 §2.4].
  - There is no ML runtime on this machine (no torch, ultralytics, onnxruntime or GPU).
    Choosing one is a later decision.

### 4.5 Calibration — `core/calibration.py`
- **Protocol:** `Calibrator.calibrate(score_raw, logit, class_id) -> float in [0, 1]`,
  plus `name`, `params`. Default `Identity` (returns `score_raw`, sets
  `calibrated = false`).
- **Future (R11):** temperature scaling (§4.2 eq. 9), Platt or isotonic (§4.1), fitted on
  a **held-out set from the same distribution** (§4). Practical consequences:
  - fit separately for sim-trained-on-sim and real deployments;
  - evaluate with reliability diagrams and ECE/MCE (§2 eq. 3, 5);
  - R11 covers classification; applying it to detector scores is an extension to
    validate, not assume.
- P1 §8.2 ⚠: raw scores must not feed a Bayesian belief grid as probabilities. So every
  detection carries `confidence_raw`, `confidence` and `calibrated`, and Belief must use
  `confidence` only when `calibrated` is true.

### 4.6 Localisation uncertainty — `core/uncertainty.py` (model `geometric_v1`)
The covariance is computed in the boat frame (across = look direction, along = heading)
and rotated to the world frame:
- `σ_across² = pitch²/12 + ((h/g)·σ_h)² + σ_nav²` — bin quantisation; the altitude error
  propagated through `g = sqrt(s² − h²)` (R6 eq. 2);
- `σ_along² = (s·sin θh)²/12 + row_pitch²/12 + (g·σ_ψ)² + σ_nav²` — beam footprint
  R1 eq. 2.10 / R12 eq. 1 with θh = 0.5° (P1 §1); row quantisation; heading error;
- plus the pixel extent of the box for non-point detections.
- Parameters `σ_h` (0.10 m), `σ_ψ` (2°) and `σ_nav` (1.0 m) are **assumptions, not
  measurements**. They are recorded with the covariance (`covariance_model`) and flagged
  in `topics.md` §6 as missing data, to be measured against USBL ground truth (P1 §8.4).
  The flat-floor misplacement of tall objects (STUDY §1.2) is a bias, noted, not added.

### 4.7 Records — `core/records.py`
- One `ImageRecord` per input image, **with or without detections**.
- Fields are given in `topics.md` §4: identity, provenance, time, track per row,
  footprint, per-side ranges, summary, detections.
- **Canonical JSON:** `sort_keys=True`, compact separators, `allow_nan=False`
  (NaN → `null`), Python's shortest-round-trip float repr.
- Everything run-dependent (wall time, latency, host, stream counters) lives under
  `provenance`.
- **Equality** (live vs offline, run vs run) compares records with `provenance` and
  `source` removed. A test asserts byte equality of the rest.

### 4.8 Pipeline — `core/pipeline.py`
`Pipeline(preprocessor, detector, calibrator, uncertainty, record_builder).run(img) ->
(ImageRecord, list[Detection])`. It is pure: same input and config give the same output.
There is no randomness, wall time or global state.

### 4.9 Offline reader — `core/session.py`
Iterates a GCS session (`<session>/seabed_images/`), a merged session, or a "Save
pictures" folder (`seabed_images_<log>/`) in `image_id` order. It builds
`SeabedImageInput` from `metadata/*.json` + `*_world.npz` (+ PNG). `stream_id` comes from
the metadata (schema ≥ 5), else `session:<folder name>`. Read-only (CM-7, NC #6).

## 5. Outputs (summary; full contract in `topics.md`)
- **`/sss_ai/detections`** (`vision_msgs/Detection2DArray`, one array per image, possibly
  empty) is the GCS's existing slot (`detections_listener.py`). Contents:
  - `bbox.center.position` = world x/y in odom metres; `bbox.size_x/y` = extent in metres
    (≥ 0.5 m for drawing, as the listener clamps);
  - `results[0].hypothesis` = class name and **calibrated** confidence;
  - `results[0].pose` = world position + 2×2 covariance (indices 0, 1, 6, 7);
  - per-detection `header.stamp` = the detection's **row time**;
  - `id` = decimal string uid `2_000_000 + 16·image_id + k` (k < 16; a 17th detection of
    one image takes `1_000_000_000 + 1000·image_id + k`), distinct from the station's
    former in-process range `1_000_000 + …`.
- **`/sss_ai/image_records`** (JSON, RELIABLE / KEEP_LAST 100) carries the Belief record
  (§4.7).
- **`/sss_ai/perception/status`** (JSON, TRANSIENT_LOCAL depth 1, 1 Hz) carries mode,
  counters and latency percentiles.
- **Record log:** `<data>/Perception_data/live/<stream_id>.jsonl`, one canonical record per
  line (`record_log: true` by default), so live and offline can be compared (§9).

`<data>` is `<root>/data/`, `<root>` resolved exactly like Control's
`custom_functions.data_root` (`core/dataroot.py`): parameter `data_dir`, then
`$BLUEBOAT_DATA_DIR`, then the parent of the first `$COLCON_PREFIX_PATH` entry, then the
cwd. Normally `~/ros2_ws/data/` (file-interfaces skill). Names are claimed exclusively
(`-2`, `-3`, … on a clash), never overwritten.

## 6. ROS node (thin) and offline CLI
- **`perception_node`** (`ros/perception_node.py`): subscriptions per §3 (RELIABLE /
  KEEP_LAST 32); `JoinBuffer` → `Pipeline` → publish; wall-clock latency measured from
  the receipt of the last part to publish; status at 1 Hz.
  - **Parameters:** `mode` (normal|dataset), `dataset_name`, `world_dir`, `data_dir`,
    `record_log`, `join_timeout_s`, `detector` (dummy), `preprocess` (fixed_gray | identity),
    `detector_input_mapping` (fixed_gray_v1), `near_range_mask_m` (0.0), `turn_window_s` (1.0),
    `calibration` (identity), `sigma_h_m`, `sigma_heading_deg`, `sigma_nav_m`,
    `footprint_row_step`.
  - **The node never subscribes to `/side_scan_sonar/ground_truth/*` or
    `/sim/sea_state/*`.** A test greps the sources (rule: online code never reads sim
    ground truth).
  - SIGINT flushes the record log and the dataset manifest.
- **Launch** `launch/perception_launch.py`:
  - arguments `mode`, `dataset_name`, `world_dir`, `data_dir`;
  - in `mode:=dataset` only, it also starts the archival ground-truth recorder (§7);
  - `use_sim_time` is not used: the node uses the image's own stamps for data and the wall
    clock for latency only.
- **Offline CLI** `blueboat-perception-offline SESSION [--out DIR] [--dataset NAME]`:
  - the same `Pipeline` over `core/session.py`;
  - writes `records.jsonl` + `detections.json` + `summary.json` to
    `<data>/Perception_data/offline/<session>/` by default;
  - refuses an `--out` inside the session tree (CM-7).

## 7. Dataset-creation mode (structure; Q3)
- **Switch:** `mode:=dataset`. Normal processing continues (detections still reach the map)
  and `DatasetWriter` (`core/dataset.py`) also writes each completed image to
  `<data>/Dataset_data/<name>/`. `<name>` comes from MCS:
  `<YYYY_MM_DD-HH_MM_SS>-<note>`.

```
<name>/
  manifest.json          blueboat_perception/dataset_manifest/1: versions, mode, world_dir, stream ids,
                         image count, label source, created (UTC), the run's config
  data.yaml              Ultralytics: names: {} until a label source defines classes
  images/<stem>.png      the DETECTOR INPUT (the configured preprocessor's `gray`; stem = <stream_id>_<image_id:05d>)
  display/<stem>.png     the GCS display PNG, preserved verbatim
  images/<stem>.json     sidecar: Aug-Studio AcquisitionMeta fields (layout "dual", altitude_m, slant_range_m,
                         res_across_m, res_along_m, speed_mps, heading_deg, prf_hz, depth_m, timestamp)
                         + "gcs_metadata": the full seabed metadata (ignored by the Aug Studio, extra="ignore")
  labels/<stem>.txt      empty: NullLabelSource
  raw/<stem>_db.npz      raw float32 dB (lossless)
  ground_truth_bag/      sim only: archival `ros2 bag record /side_scan_sonar/ground_truth/{contacts,sensor}`
```

- **Labels hook:** `LabelSource.labels_for(img) -> list[YoloBox]`. The default is
  `NullLabelSource` (no labels). A future `SimGroundTruthLabelSource` works **offline**
  from `ground_truth_bag/` + `world_dir/metadata.yaml`, joined through the Q8 device ping
  numbers. Whether the recorded ground truth suffices for boxes is that labeller's
  question (the simulator is not modified in this work).
- **The ground-truth recorder** (answer 2) is a separate `ros2 bag record -o
  <dataset>/ground_truth_bag --topics /side_scan_sonar/ground_truth/{contacts,sensor}`
  process started by `launch/perception_launch.py` in `mode:=dataset` when `world_dir` is
  set (a simulated world run). The launch file claims the dataset folder and passes it to
  the node (`dataset_path`), so both write into the same folder. Archival only: the
  Perception node never reads ground truth (test-enforced).
- **Intensity mapping (2026-10-16).** Under `fixed_gray` the training image is a linear dB
  window after the fixed range normalisation: the sidecar declares `intensity_mapping:
  log` with `log_range_db = hi − lo`, which the Aug Studio's `log` inversion matches
  exactly (R-3 resolved for this path). Under `identity`, the gap below remains.
- **Intensity mapping gap (identity).** The GCS PNG is the display model's `power` transfer, which the
  Aug Studio's `IntensityMapping` vocabulary (`linear | gamma | log`, `core/meta.py`)
  cannot express. The sidecar therefore does **not** set `intensity_mapping`, which would
  mis-declare it, and the manifest records `"intensity_mapping": "gcs_display_power
  (not expressible in Aug Studio; REQUESTS R-3)"`. Until R-3 is resolved, Aug Studio
  intensity-domain families must not be run on these datasets. The raw dB in `raw/` keeps
  the dataset invertible.
- **Splits are not made by the writer.** Split by session or site later, never by image:
  consecutive images overlap 50 % (R12 §4 splits by site for the same reason).
- Nothing is ever written into a session folder (CM-7). Re-running the same name gets a
  `-2`, `-3`, … suffix (O_EXCL), never an overwrite.

## 8. MCS changes (ordered by you, Q3/Q7 and answer 4; built 2026-10-16, additions only)
- **`LaunchDialog`, Gazebo + Controller source:** a new **Session** row with *Normal
  session* / *Dataset creation*, its own row after *Robot file*. It is stored as
  `LaunchParameters.perception_mode`, which is **not a launch argument**: `to_cli()`,
  `launch_target()` and the sea companion are unchanged for every launch (smoke-pinned).
  A later "Replanner mode" / path-source row can be added beside it the same way. The
  world is chosen after this dialog, so the row shows for every Gazebo controller run; the
  companion decides at launch (§ below). The dataset name is
  `<YYYY_MM_DD-HH_MM_SS>-<note>` from the launch time and the run's wire note
  (`sim-<note>`), `perception_dataset_name`.
- **`launch_manager.perception_command(cfg, params)`:** a new companion beside the sea and
  guardian ones.
  - It runs for `simulation and world_dir and not replay_bag` (personalized worlds: the
    only sim runs with a sonar) in the chosen mode, and for **every real-robot launch** in
    `normal`.
  - Never for the empty world (no sonar; the console says so) or a bag replay.
  - Command: `ros2 launch blueboat_perception perception_launch.py mode:=…
    dataset_name:=… world_dir:=…`.
  - Best-effort: a missing package (ament index) gives a console line, never an aborted
    mission. It is signalled with the other companions on stop (N1 unaffected: Perception
    has no actuators).
- **Settings:** `mcs/config/settings.py` gets `PerceptionConfig` (`enabled`, `package`,
  `launch_file`).
- **Tests:** `smoke_test.py` gets a `perception ok` block covering which launches get the
  companion, the CLI, the dialog row and the dataset name. `ruff check .` must stay clean.
- **Docs:** `docs/03_ros_integration.md`, `docs/04_user_guide.md`, and MCS `CLAUDE.md` §2
  (companions, launch-argument checklist).
- Nothing in `mcs/` imports Perception.
- **The GCS is separate.** It is started by the operator, not by MCS. It has no detector
  of its own since answer 3, so with Perception running its map shows Perception's
  detections, and only those; there is nothing to configure.

## 9. Live / offline equivalence
For any image saved in a session, the offline input is bit-identical to the live input.
`intensity_db` is lossless in the npz, and the metadata JSON round-trips Python floats
exactly. With `Pipeline` pure (§4.8), the records are identical apart from `provenance`
and `source`. Tests:
- (a) core: wire-path vs file-path construction of the same image gives identical records;
- (b) Phase 3: a live sim run with `record_log`, then the offline CLI over its session,
  compared record by record.

Images produced while no session was recording exist only live; their records exist only
in the live log, and the comparison reports their count.

## 10. Performance
- **Target:** per-image processing (join complete → publish) ≪ the image period, with
  ≥ 10× margin. That is < 250 ms against the ≥ 2.8 s measured period.
- The dummy and geometry are vectorised numpy. The footprint uses `footprint_row_step`
  rows.
- Measured and reported in Phase 3 as p50/p95/max over a run, split into join wait /
  pipeline / publish.

## 11. Testing (Phase 2 gate)
pytest, ROS-free core, run with `/usr/bin/python3` (SSS conftest pattern: ROS blocked on
`sys.meta_path` for the import sweep; node tests skipped via `find_spec` + `skipif`, never
a module-level `importorskip`). Tests:
- geometry vs the station (§4.2), both conventions;
- dummy equals the station dummy on all on-disk sessions;
- one record per image, including images with no detection, and `no_pixels` records;
- record completeness and types against the `topics.md` schema;
- determinism (two runs → byte-identical JSONL);
- wire vs file equivalence;
- JoinBuffer (ordering, timeout, gaps, duplicates);
- dataset writer layout + manifest, suffix on name clash, never inside a session;
- import sweep (no rclpy/Qt in `core/`);
- no ground-truth subscription in the online sources.

Corpus tests skip cleanly without `$BLUEBOAT_DATA_DIR` data.

## 12. Project rules, how each is met

| Rule | How |
|---|---|
| CM-1 | No new message type; standard types + schema-tagged JSON; new topics additive |
| CM-2 | Same topics, types and QoS in sim and real. The GCS is the producer in both graphs, and Q8 attitude comes from `/mavros/imu/data`, present in both (mavros / shim). The ping quaternion (yaw-only on the real boat, full in sim) is never used |
| CM-3 | Perception's package never imports or patches another module. The SSS (task B, Q6, Q8) and MCS (Q3/Q7) edits are made inside those modules by your order and documented there |
| CM-4 | Image topics RELIABLE / KEEP_LAST 32 at both ends; detections keep the listener's default profile |
| CM-7 | Never writes into session folders; dataset and offline outputs in their own roots, O_EXCL names |
| CM-9 | Records are designed for Belief as the only consumer of evidence (footprint, look directions, empty-image records) |
| CM-10 | Input is the boat-relative waterfall tile, never a mosaic crop |
| CM-11 | The dataset mode writes sim datasets; their use is the E7 gap experiment, and the headline detector stays real-trained (P1 §7.6 ⚠) |
| CM-14 | Offline CLI over recorded sessions gives the same records as live |
| "Online code never reads sim ground truth" | The node subscribes to none; the archival recorder is a separate process; test-enforced |

## 13. Alternatives considered and rejected
- **File spool + notification** (GCS writes npz/json per image, Perception watches):
  writes data outside sessions (NC #9 strain); ~50 ms of disk/zlib per image; the current
  `_finish` publishes paths before the asynchronous save completes (a race).
- **Perception rebuilds images from pings:** duplicates image formation and makes
  Perception read pings, against the brief.
- **Own interfaces package (typed `SeabedImage` / `ImageRecord`):** atomic and typed, but
  the first message definitions outside `blueboat_interfaces` (a CM-1 exception), with
  both SSS and Belief depending on it. Revisit when Belief freezes its needs, as HANDOVER
  §3.1bis plans for `seabed_analysis`.
- **Base64 pixels inside JSON:** ×1.33 size plus encode cost, and opaque.
- **Re-implementing the display model in Perception** to get the PNG: two implementations
  that can drift; the PNG is transported instead.
- **Perception subscribing to ground truth for labels:** forbidden online. Labels come
  offline from the archival bag.

## 14. Open items carried to `.claude/TODO.md`
- Phase 4 preprocessing decision (STUDY §6).
- YOLO runtime choice and adapter (tiling, NMS across tiles); the uid's overflow range
  (k ≥ 16) is in place, a stream-aware uid is not (uids repeat across GCS streams, as the
  station's did — SSS `TODO.md`).
- Calibration data split and calibrator fit (R11).
- Offline sim labeller from the dataset's `ground_truth_bag/` (the simulator is not
  modified in this work; the R-2 request was withdrawn).
- Aug Studio intensity-mapping gap (REQUESTS R-3).
- Measure `σ_nav`, `σ_ψ`, `σ_h` against USBL.
- Typed messages once Belief exists.
