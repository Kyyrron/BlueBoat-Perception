# CLAUDE.md — BlueBoat-Perception

Seabed images in, detections and image records out, for the BlueBoat aspect-aware
side-scan survey. Built 2026-10-15/16 (Phases 0–5 of its brief). Runs on the operator
laptop like every node of the project; the boat runs nothing of ours.

```
GCS seabed image (raw dB + PNG + metadata)  ->  join  ->  preprocess (detector input)
   ->  detector (dummy today, YOLO later)  ->  calibration (identity)  ->  georeference
   ->  /sss_ai/detections (GCS map)  +  one record per image (/sss_ai/image_records)
```

**Authority.** `docs/topics.md` is the interface contract, binding since 2026-10-16;
changes are additive only (CM-1). `docs/DESIGN.md` explains why. The detector-input
decisions and their evidence are in `docs/PREPROCESSING_STUDY.md` §6. Where this file
and the code disagree, the code wins: read the file before trusting a description of it.

## 1. Layout
- `blueboat_perception/` — the ROS 2 package (`ament_python`), one level below the git
  root:
  - `blueboat_perception/core/` — ROS-free, deterministic: numpy + stdlib only. OpenCV
    is loaded lazily, only to encode or decode a PNG.
    - `image.py` — `SeabedImageInput`: `from_wire` / `from_session_files`; the georef
      conventions;
    - `geometry.py` — pixel → world (conventions 1 / 2), spans, footprint, turns;
    - `preprocess.py` — the detector input: `fixed_gray` / `identity`, the mask, the
      disabled boundaries;
    - `detectors/` — `dummy` = the station's former dummy; `yolo` = interface stub;
    - `calibration.py`, `uncertainty.py` (`geometric_v1`), `records.py` (canonical
      JSON);
    - `pipeline.py`, `join.py`, `session.py`, `dataset.py`, `labels.py`,
      `dataroot.py`, `recordlog.py`, `stats.py`;
    - `sim_labels.py` — dataset labels from the simulator's ground truth (§4);
    - `data/fixed_gray_v1.json` — the fixed mapping's parameters;
      `data/yolo_classes_v1.json` — the frozen YOLO classes.
  - `ros/perception_node.py` + `ros/convert.py` — the thin node, and the only place
    rclpy and message types are imported. `ros/ground_truth.py` — the dataset mode's
    two ground-truth subscriptions, and the only online file naming them.
  - `cli/offline.py` — `blueboat-perception-offline`.
  - `launch/perception_launch.py`, `config/perception.yaml`, `tests/` (pytest).
- `tools/` (not installed):
  - `fit_detector_input.py` — fits a fixed mapping;
  - `preprocessing_study.py` — reproduces STUDY §5;
  - `draw_labels.py` — draws a dataset's boxes and checks their intensities.
- `docs/` — DESIGN, topics, PREPROCESSING_STUDY, REFERENCES, REQUESTS, LIVE_CHECKS;
  `data/phase4_2026-10-16.json`.

## 2. Interfaces (fields: `docs/topics.md`)
| Topic | Type | Dir | QoS |
|---|---|---|---|
| `/sss_ai/seabed_image` | `sensor_msgs/Image` 32FC1 (raw dB) | in, from the GCS | RELIABLE, KEEP_LAST 32, VOLATILE |
| `/sss_ai/seabed_image/png` | `sensor_msgs/Image` mono8 (display PNG) | in | same |
| `/sss_ai/seabed_image/metadata` | `std_msgs/String` JSON envelope | in | same |
| `/sss_ai/detections` | `vision_msgs/Detection2DArray`, one per image | out, to the GCS `DetectionsListener` | default (10) |
| `/sss_ai/image_records` | `std_msgs/String`, `blueboat_perception/image_record/1` | out (Belief, analysis) | RELIABLE, KEEP_LAST 100 |
| `/sss_ai/perception/status` | `std_msgs/String`, 1 Hz | out | RELIABLE, depth 1, TRANSIENT_LOCAL |

- The key is `"<stream_id>/<image_id>"`, carried in `Image.header.frame_id` (an image
  key, not a TF frame) and in the envelope.
- The image QoS must stay equal to the GCS publishers' (`ros/ros_manager.py`; CM-4).
- **Perception is the station's one detection source** (owner's order, 2026-10-16). The
  GCS runs no detector; its replay "Run AI" imports this module's core in-process
  (`blueboat_gcs/core/perception_core.py`). This module imports nothing from the GCS.
- Detection uid = `2_000_000 + 16·image_id + k`; past the 16th detection of one image,
  `1_000_000_000 + 1000·image_id + k`.
- **Files written** (root `<root>` = `data_dir` → `$BLUEBOAT_DATA_DIR` → the parent of
  the first `$COLCON_PREFIX_PATH` entry → the cwd, as Control's `data_root`; normally
  `~/ros2_ws`):
  - `<root>/data/Perception_data/live/<stream_id>.jsonl` — the live record log;
  - `<root>/data/Perception_data/offline/<session>/` — `records.jsonl`,
    `detections.json`, `summary.json`;
  - `<root>/data/Dataset_data/<name>/` — dataset mode: `images/` (the detector input),
    `display/`, `labels/` (YOLO, from the simulator's ground truth in a world run; else
    empty), `raw/`, `manifest.json`, `data.yaml`, `ground_truth_bag/`.
  - Names are claimed exclusively (`-2`, `-3` …); nothing is ever overwritten.
- **Started by MCS** (`launch_manager.perception_command`): every real-robot launch in
  `mode:=normal`, and every personalized-world Gazebo launch in the launch dialog's
  Session mode (normal / dataset). Never for the empty world or a bag replay.

## 3. Non-negotiables
1. **The core stays ROS-free and deterministic.** No rclpy, Qt or OpenCV at import time
   in `core/` (the import sweep blocks them); no randomness, no wall clock.
   `Pipeline.run` is pure.
2. **One record per image, with or without detections.** `detections: []` means searched
   and nothing found; `null` means no pixels, i.e. unknown. Never drop an image silently:
   every loss is a counter in `JoinBuffer` (`late_parts`, `partial_no_pixels`,
   `dropped_no_metadata`, `gaps`, `duplicates`).
3. **Offline = live.** Equality = canonical JSON with `provenance` and `source` removed.
   Live inputs are bit-identical to the session's npz + JSON, so any run-dependent value
   goes under `provenance`, and nothing else may depend on the path. Checked live:
   `LIVE_CHECKS.md` checks 2 and 5.
4. **Simulator ground truth reaches the dataset labels and nothing else; the sonar
   nothing at all.** Owner's decision, 2026-10-16, revising "online code never reads
   ground truth":
   - the node subscribes to the two ground-truth topics only in `mode:=dataset` with a
     `world_dir`, from `_open_labels`, through `ros/ground_truth.py`;
   - the detection path (pipeline, detectors, preprocessing, calibration, records)
     never imports `sim_labels`, so records are identical with or without labels;
   - tests: the source sweep, the AST checks, the import check, the record identity;
   - the archival `ground_truth_bag/` is still recorded, and re-labels offline.
5. **Never write into a recording session or another module's tree** (CM-7, CM-3).
   Neighbour modules are not edited from here; needs go to `docs/REQUESTS.md`.
6. **The detector input never changes the originals.** SSS forms the pictures. A
   preprocessor builds a separate `DetectorInput.gray` and never modifies `db` or `png`.
   The record names the mapping, and for `fixed_gray` the parameter file's SHA-256.
7. **A fixed mapping, once used for training, is never refitted in place.** A refit is a
   new file and name (`fixed_gray_v2`) via `tools/fit_detector_input.py`, fitted on real
   data only (CM-11).
8. **No unvalidated image processing.**
   - `roll_correction` raises (6.6).
   - Geometric correction, despeckling, histogram equalisation and Retinex are deferred
     options that raise (6.7, "not yet").
   - Adding one needs a controlled comparison against the preserved originals.
9. **Raw detector scores are not probabilities.** `confidence` is a probability only when
   `calibrated` is true (project_synthesis §8.2 ⚠). Today it is identity, so false.
10. **Cite nothing from memory.** Every non-software choice traces to `docs/REFERENCES.md`
    (read in full, or the claim is not used).

## 4. The detector input (STUDY §6, decided 2026-10-16)
- **`fixed_gray`** (default, decision 6.4):
  `gray = round(255·clip((db − g − R(s) − lo)/(hi − lo), 0, 1))`, NaN → 0.
  - `g` = the row's receive-gain step;
  - `R(s)` = one fixed empirical range-reference curve; it assumes no TVG law, because
    the device's own TVG is not identifiable from its output (STUDY §1.4);
  - `[lo, hi] = [−31.15, 17.16]` dB, fitted on the real beach data;
  - no per-image statistics.
- **`identity`:** the GCS display PNG, the old input and the baseline for comparisons.
  The display model drifts within a live mission (STUDY §5, M5–M7).
- **`near_range_mask_m`** (6.5): 0.0 (default) or 0.75, independent of the mapping; the
  boundary column is kept.
- **Turn flags** (6.2): per row `turn_radius_m` / `turn_inner_side` / `backscan_from_m`,
  per detection `turn_backscan`.
  - `r* = v/|ω|` with ω averaged over `turn_window_s` (1 s, an engineering choice).
  - Flags only: no row or detection is removed.
- **Dataset mode** writes the detector input as the training image. The sidecar declares
  `intensity_mapping: log`, `log_range_db = hi − lo` for the Aug Studio.
- **Labels** (dataset mode in a simulator world, `core/sim_labels.py`, DESIGN §7):
  - one YOLO box per object = its highlight + its acoustic shadow;
  - built per picture row from the ping's exact rendering pose and the renderer's
    contacts, with the renderer's own beam geometry;
  - classes = the 29 World Builder classes, frozen and append-only;
  - multipath ghosts, wall echoes and hidden pings are skipped and counted;
  - checked against noise-free renders of the recorded pings (`LIVE_CHECKS.md`
    check 6).

## 5. Commands
```bash
# build (from ~/ros2_ws; never Control's in-progress work with it)
colcon build --packages-select blueboat_perception
# tests (from BlueBoat-Perception/blueboat_perception/, system python)
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest
# run
ros2 launch blueboat_perception perception_launch.py [mode:=dataset dataset_name:=…] [world_dir:=…]
# offline over a session, compared with its live log
python3 -m blueboat_perception.cli.offline <session> [--compare <live.jsonl>] \
    [--preprocess fixed_gray|identity] [--near-range-mask-m 0.75] \
    [--dataset NAME [--world <world dir> --ground-truth-bag <dataset>/ground_truth_bag]]
# look at a dataset's labels (overlays, crops, highlight/shadow intensities)
/usr/bin/python3 tools/draw_labels.py <dataset> --out DIR [--crops] [--stats]
# status during a run
ros2 topic echo --once /sss_ai/perception/status std_msgs/msg/String \
    --qos-durability transient_local --qos-reliability reliable
# reproduce STUDY §5 (read-only) / fit a NEW mapping
/usr/bin/python3 tools/preprocessing_study.py --out DIR [--only …]
/usr/bin/python3 tools/fit_detector_input.py --name fixed_gray_v2 --out … <real image folders>
```
- Do not source `~/ros2_ws/install/setup.bash` before pytest: it switches `python3` to
  the venv, which lacks pytest.
- The corpus tests read `$BLUEBOAT_DATA_DIR/data/SSS_data` or `~/ros2_ws/data/SSS_data`
  and skip without it.
- The Aug Studio sidecar test needs pydantic. Run it with
  `SSS-Dataset-Aug-Studio/.venv/bin/python -m pytest tests/test_dataset.py`.

## 6. Gates (2026-10-16)
- **Perception:** 109 pass / 4 skip. The skips: 3 corpus folders without stored
  detections, and the pydantic test on the system python. The class-map test reads the
  simulator's source beside this module and skips without it.
- **SSS** (shares this module's contract): 353 pass / 50 skip, including the golden
  regression `tests/test_perception_golden.py` and `tests/test_perception_bridge.py`.
- **MCS:** `smoke_test.py` PASSED, including `perception ok`; `ruff check .` clean.
- No lint or type-checker is configured here.
- A fresh clone gets everything except the on-disk corpus.

## 7. Where things are documented
- **Open work:** `.claude/TODO.md`.
- **Live checks and the real-boat command sheet:** `docs/LIVE_CHECKS.md`.
- **The SSS side** (what changed there, what is pinned unchanged):
  `BlueBoat-SSS/blueboat_sss/blueboat_gcs/docs/PERCEPTION_INTEGRATION.md`.
- **This module in the superproject:** the root `.claude/CLAUDE.md` (module table, §4.3
  rows, D7) and the skills `module-gates` / `file-interfaces`, updated 2026-10-16
  (REQUESTS R-4).
