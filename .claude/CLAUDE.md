# CLAUDE.md — BlueBoat-Perception

Seabed images in, detections and image records out, for the BlueBoat aspect-aware
side-scan survey. Runs on the operator laptop like every node of the project; the boat
runs nothing of ours.

```
GCS seabed image (raw dB + PNG + metadata) -> join -> preprocess (detector input)
  -> detector (dummy default; YOLO over 256 x 256 tiles, merged) -> calibration (identity)
  -> georeference -> /sss_ai/detections (GCS map) + one record per image (/sss_ai/image_records)
```

**Authority.** `docs/topics.md` is the interface contract; changes are additive only
(CM-1). `docs/DESIGN.md` explains why; the detector-input decisions and their evidence are
`docs/PREPROCESSING_STUDY.md` §6. Where this file and the code disagree, the code wins.

**Simulated data (simulator frozen 2026-11-18).** Simulated results follow `../SIM_VALIDITY_CONDITIONS.md`
(superproject root). For the dataset and the detector:
- **Labels from the rendered picture and the world metadata, never from a contact's `shadow_bins`** (§5.5 of
  that file: `shadow_bins` is geometric, 5–35 bins too long; `slant_range_m` is across-track). **PER-LBL-1
  done (2026-11-18):** the shadow end is the flat-floor tip from the metadata, no box extent comes from a
  contact (`shadow_bins` is only reported as `s_mask`), and without terrain the height is the sensor's
  `altitude_m`; contacts still select which objects a ping saw and their `visible` flag (tests
  `test_sim_labels_shadow.py`).
- Dataset rendered at the **R6 low end** with the production ray-cast `raycast_harbour_fast`; dataset and
  evaluation worlds from **disjoint seed ranges**; split and calibrate by world. Train with **level and
  per-ping gain augmentation** (simulated pictures are 1–2 dB brighter and more uniform than real). Report per
  object family, with an oracle-detector arm. Never claim a field detection rate.
- **CR-PER-2 (transducer lever arm) is applied** in `0d55d76` (2026-11-15; convention 3, `core/geometry.py`
  `fore_aft_arm` / `sensor_point`; re-checked by session 12, `~/ros2_ws/sim_runs/perception/handoff_12_cr_per_2.md`):
  re-run the module's tests and re-check it before any pipeline test.
- The class map `core/data/yolo_classes_v1.json` (29 classes) is legacy: the World Builder's object set
  version 2 has 11 classes since 2026-11-15 (open item 02-CR2).

## 1. Layout
`blueboat_perception/` is the ROS 2 package (`ament_python`), one level below the git root.
- `core/` is ROS-free and deterministic (numpy + stdlib); OpenCV and PyYAML are imported
  lazily, inside functions.
- `core/data/yolo_classes_v1.json` is LEGACY: only `load_class_map` and test fixtures read it;
  datasets take classes from their world.
- `ros/` is the only place rclpy and message types are imported; `ros/ground_truth.py` is the
  only online file naming the ground-truth topics.
- `tools/` is not installed (offline only); its scripts import each other from that folder.

## 2. Interfaces (fields: `docs/topics.md`)
| Topic | Type | Dir | QoS |
|---|---|---|---|
| `/sss_ai/seabed_image` | `sensor_msgs/Image` 32FC1 (raw dB) | in, from the GCS | RELIABLE, KEEP_LAST 32, VOLATILE |
| `/sss_ai/seabed_image/png` | `sensor_msgs/Image` mono8 (display PNG) | in | same |
| `/sss_ai/seabed_image/metadata` | `std_msgs/String` JSON envelope | in | same |
| `/sss_ai/detections` | `vision_msgs/Detection2DArray`, one per image | out, to the GCS `DetectionsListener` | default (10) |
| `/sss_ai/image_records` | `std_msgs/String`, `blueboat_perception/image_record/1` | out (Belief, analysis) | RELIABLE, KEEP_LAST 100 |
| `/sss_ai/perception/status` | `std_msgs/String`, 1 Hz | out | RELIABLE, depth 1, TRANSIENT_LOCAL |

- **Georef conventions** (topics.md §2): 1 (no `georef`), 2 (lateral lever, `bottom_m`), 3
  (since 2026-11-15: 2 placed from the transducers' point, `pose + R(yaw)·(lever_arm_m.x, 0)`,
  the station's operation order, bit-exact with its grids). The arm comes from each picture,
  never hard-coded (−0.345 m today: adopted, not measured). Any other convention is rejected
  (`InputError`). An arm of 0 leaves the row pose untouched, so conventions 1 and 2 are
  byte-identical to before; the record schema stays `image_record/1`.
- Image key `"<stream_id>/<image_id>"`, in `Image.header.frame_id` (a key, not a TF frame)
  and in the envelope. The image QoS must equal the GCS publishers' (CM-4).
- **Perception is the station's only detection source.** The GCS runs no detector; its
  replay "Run AI" imports this core in-process (`blueboat_gcs/core/perception_core.py`).
  This module imports nothing from the GCS.
- Detection uid = `2_000_000 + 16·image_id + k`; from the 17th detection,
  `1_000_000_000 + 1000·image_id + k`; at most 999 per image (cut counted in
  `processing.detector.capped`).
- Only `confidence >= map_min_confidence` (0.10) goes on `/sss_ai/detections`; records
  keep every detection.
- Detector switch: `detector: dummy_center | yolo` + `detector_model` and the YOLO keys
  (`core/detectors/yolo.py` `PARAM_DEFAULTS`, shared by node and CLI; topics.md §6).
- **Files** (root = `data_dir` → `$BLUEBOAT_DATA_DIR` → parent of the first
  `$COLCON_PREFIX_PATH` entry → cwd; normally `~/ros2_ws`), all under `<root>/data/`:
  - `Perception_data/live/<stream_id>.jsonl` (live record log);
  - `Perception_data/offline/<session>/` (`records.jsonl`, `detections.json`,
    `summary.json`);
  - `Dataset_data/<name>/` (`images/` = detector input, `display/`, `labels/`, `raw/`,
    `manifest.json`, `data.yaml`, `ground_truth_bag/`);
  - offline tools: `Tile_data/`, `Perception_models/`, `Perception_eval/`.
  - Names are claimed exclusively (`-2`, `-3` …); nothing is ever overwritten.
- **Started by MCS** (`launch_manager.perception_command`): every real-robot launch
  (`mode:=normal`) and every personalized-world Gazebo launch (Session mode normal /
  dataset, plus `use_sim_time:=true`). Never for the empty world or a bag replay.
- **Clock (2026-11-07).** `use_sim_time` (launch argument, default false) puts the dataset
  label wait (`label_wait_s`) on `/clock` and records the ground-truth bag with
  `--use-sim-time`, so a simulation slowed below real time (the ray-cast sonar) labels
  and records as at real speed. The image join and every latency stay wall (provenance).
  The harbour ray-cast profiles publish `ground_truth/contacts` since 2026-11-11 (before, a ray-cast
  run's label files were empty); datasets use the production `raycast_harbour_fast`, the geometric sonar
  is legacy.

## 3. Non-negotiables
1. **Core ROS-free and deterministic.** No rclpy, Qt, OpenCV, torch or ultralytics at
   import time in `core/` (the import sweep blocks them); no randomness, no wall clock.
   `Pipeline.run` is pure.
2. **One record per image, with or without detections.** `detections: []` = searched,
   nothing found; `null` = unknown (no pixels, or no detector input). Every loss is a
   `JoinBuffer` counter (`late_parts`, `partial_no_pixels`, `dropped_no_metadata`, `gaps`,
   `duplicates`).
3. **Offline = live**: canonical JSON equal once `provenance` and `source` are removed.
   Live inputs are bit-identical to the session's npz + JSON, so anything run-dependent
   goes under `provenance`. Any core refactor must keep records byte-identical.
4. **Simulator ground truth reaches the dataset labels and nothing else; the sonar
   nothing at all.** The node subscribes to the two ground-truth topics only in
   `mode:=dataset` with a `world_dir` (`_open_labels` → `ros/ground_truth.py`). The
   detection path (pipeline, detectors, preprocess, calibration, records) never imports
   `sim_labels`. Enforced by the source sweep, AST and import checks, record identity.
5. **Never write into a recording session or another module's tree** (CM-7, CM-3).
   Needs from neighbours go to `docs/REQUESTS.md`.
6. **The detector input never changes the originals.** A preprocessor builds a separate
   `DetectorInput.gray`; `db` / `png` stay untouched. The record names the mapping and,
   for `fixed_gray`, the parameter file's SHA-256.
7. **A fixed mapping used for training is never refitted in place**: a refit is a new
   file and name (`fixed_gray_v2`, `tools/fit_detector_input.py`), on real data only
   (CM-11).
8. **No unvalidated image processing.** `roll_correction`, geometric correction,
   despeckling, histogram equalisation and Retinex raise (STUDY 6.6, 6.7). Adding one
   needs a controlled comparison against the preserved originals.
9. **Raw scores are not probabilities.** `confidence` is one only when `calibrated` is
   true; today calibration is identity, so false.
10. **Cite nothing from memory.** Every non-software choice traces to `docs/REFERENCES.md`.
11. **The YOLO runtime is optional and lazy**: only building a `YoloDetector` with the
    default runtime imports torch / ultralytics. The dummy stays default, tested, and its
    records byte-identical.
12. **One tiling function for training and inference** (`core/tiling.py`) on the same
    detector input. Tiles are never resized; one record per picture, never per tile.
13. **Split by world, never by image; never tune on the held-out test worlds.** Every
    model and result carries config, dataset + split, seed, model / library / code
    versions in a JSON beside it, and is stated "on simulated data" (CM-11, CM-13).

## 4. Detector input, labels, tiles (STUDY §6; DESIGN §4.10, §7, §15)
- **`fixed_gray`** (default): `gray = round(255·clip((db − g − R(s) − lo)/(hi − lo), 0, 1))`,
  NaN → 0; `g` = row gain step, `R(s)` = one fixed empirical range-reference curve (no TVG
  law: not identifiable, STUDY §1.4), `[lo, hi] = [−31.15, 17.16]` dB from real beach
  data; no per-image statistics. **`identity`** = the GCS display PNG (baseline; its
  display model drifts within a mission, STUDY §5).
- `near_range_mask_m` 0.0 (default) or 0.75; boundary column kept.
- Turn flags, flags only: per row `turn_radius_m` / `turn_inner_side` / `backscan_from_m`,
  per detection `turn_backscan`; `r* = v/|ω|`, ω averaged over `turn_window_s` (1 s).
- Dataset mode writes the detector input as the training image; the sidecar declares
  `intensity_mapping: log`, `log_range_db = hi − lo` (Aug Studio).
- **Labels** (`sim_labels.py`): one YOLO box per object = highlight + shadow, per row from the rendering
  pose, the world metadata and the beam geometry (contacts only select objects, PER-LBL-1); classes = the
  world's types sorted, ids from 0; ghosts, walls, hidden pings skipped and counted (`LIVE_CHECKS.md` check 6).
- **Tiles:** 256 × 256 native, across track, stride 128 (64 = an arm), flush tile at
  `W − T`, nadir kept, short pictures padded below with invalid zeros, a tile skipped only
  when valid share < 0.001 (counted).
- **Adapter:** one batch per picture at `imgsz` 256, threshold 0.01, in-tile NMS 0.7,
  boxes shifted back and merged (`nms` default; `greedy_nmm` = IoS union, max score),
  inclusive integer boxes, 999 cap; the box centre is the georeferenced point. CPU, seed,
  deterministic torch, rounding 1e-3 px / 1e-6.
- **Tile datasets:** at tile edges keep a clipped label if ≥ 50 % of its highlight is in
  and it is ≥ 4 px wide, else drop the label and keep the tile; faint boxes (< 2 dB) drop
  their tiles; ≤ 10 % empty tiles in train; classes = sorted union of names.
- **Undecided** (see TODO): the merge rule and the 2 dB visibility threshold.

## 5. Commands
```bash
# build (from ~/ros2_ws)
colcon build --packages-select blueboat_perception
# tests (from BlueBoat-Perception/blueboat_perception/, system python; do NOT source
# ~/ros2_ws/install/setup.bash first: its venv python lacks pytest)
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest
# run
ros2 launch blueboat_perception perception_launch.py [mode:=dataset dataset_name:=…] [world_dir:=…]
# offline over a session, compared with its live log
python3 -m blueboat_perception.cli.offline <session> [--compare <live.jsonl>] \
    [--preprocess fixed_gray|identity] [--near-range-mask-m 0.75] \
    [--dataset NAME [--world <world dir> --ground-truth-bag <dataset>/ground_truth_bag]]
# dataset labels: overlays, crops, highlight/shadow intensities
/usr/bin/python3 tools/draw_labels.py <dataset> --out DIR [--crops] [--stats]
# status during a run
ros2 topic echo --once /sss_ai/perception/status std_msgs/msg/String \
    --qos-durability transient_local --qos-reliability reliable
# tile datasets, training, evaluation (README has the full comparison recipe)
python3 tools/make_tile_dataset.py <datasets...> --split split.yaml --name tiles128 [--stride 64] [--mode full]
python3 tools/train_yolo.py --data <root>/data/Tile_data/tiles128/data.yaml --model yolo11s.pt|yolo26s.pt --name NAME
python3 tools/evaluate_detector.py <datasets...> --split split.yaml --on val|test --name NAME \
    --detector yolo --detector-model <best.pt> [--merge greedy_nmm] [--detector-tiling full]
# reproduce STUDY §5 (read-only) / fit a NEW mapping
/usr/bin/python3 tools/preprocessing_study.py --out DIR [--only …]
/usr/bin/python3 tools/fit_detector_input.py --name fixed_gray_v2 --out … <real image folders>
```

## 6. Gates (full: `.claude/gates.md`, moved verbatim 2026-11-18; auto-loaded with the tests by `.claude/rules/gates.md`)
- Perception pytest (no lint or type checker): 170 pass / 16 skip with the system python and no ROS, corpus,
  torch or sibling repos (2026-11-15); on the laptop with ROS and the corpus 204 pass / 28 skip (222 / 28 after
  session 12's added tests, 2026-11-18). A new session's pictures must stay bit-exact in `test_corpus.py`.
- SSS `tests/test_perception_golden.py` / `test_perception_bridge.py` whenever the dummy's records could
  change; MCS `smoke_test.py` (`perception ok`). Exact commands: superproject skill `module-gates`.

## 7. Where things are documented
- Open work: `.claude/TODO.md` (first, out of cloud mode: tune the 2 dB visibility
  threshold with `draw_labels.py --stats`, decide the merge rule on held-out worlds).
- Live checks and the real-boat command sheet: `docs/LIVE_CHECKS.md`.
- SSS side: `BlueBoat-SSS/blueboat_sss/blueboat_gcs/docs/PERCEPTION_INTEGRATION.md`.
- Superproject: root `.claude/CLAUDE.md`, skills `module-gates` / `file-interfaces` (REQUESTS R-4).
