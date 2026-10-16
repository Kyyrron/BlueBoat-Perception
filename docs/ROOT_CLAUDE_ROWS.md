# Root `CLAUDE.md` rows for BlueBoat-Perception (REQUESTS R-4)

Delivered 2026-10-16 for you to apply; **nothing here is applied**. Each item names
where it goes in the superproject `.claude/CLAUDE.md` (or a superproject skill), what is
there now, and the proposed text. Every fact was checked against the code on 2026-10-16;
markers follow the root file's rules (VERIFIED = checked in this tree).

The superproject `.gitmodules` already registers the module (path `BlueBoat-Perception`,
`https://github.com/Kyyrron/BlueBoat-Perception.git`, uncommitted). Its repository has
no commit yet.

---

## 1. §1 System overview

**Diagram.** Current line:
`├─ SSS-Dataset-Aug-Studio   (offline, no ROS)` followed by
`│  [detector · belief layer · replanner]  not implemented`. Proposed:
```
                                           ├─ SSS-Dataset-Aug-Studio   (offline, no ROS)
                                           ├─ BlueBoat-Perception      (detector: dummy today; image records)
                                           │  [belief layer · replanner]  not implemented
```

**Module table**, new row after BlueBoat-SSS-Sim:

| Module | Produces |
|---|---|
| **BlueBoat-Perception** — `blueboat_perception` | Detections for the GCS map (`/sss_ai/detections`) and one record per seabed image for the future belief layer (`/sss_ai/image_records`), from the GCS's seabed images; a ROS-free deterministic core, a thin node and an offline CLI (offline = live, byte-equal records); dataset mode (training images + archival simulator ground-truth bag). Detector: the station's former dummy; YOLO is an interface stub. Since 2026-10-16 |

## 2. §2 Workspace layout and build order

Append to the first paragraph:
> `BlueBoat-Perception`'s ROS package also sits one level below its git root, at
> `BlueBoat-Perception/blueboat_perception/` (`ament_python`). It depends on `std_msgs`,
> `sensor_msgs` and `vision_msgs` only, not on `blueboat_interfaces`, so its build has no
> order constraint (`colcon build --packages-select blueboat_perception`). The GCS
> imports its ROS-free core lazily for the replay window's Run AI (installed package,
> else the superproject checkout); Perception imports nothing from the GCS.

## 3. §3 Git architecture — the frozen-module record

Append after the 2026-10-14 link-guardian sentence:
> **On 2026-10-16 the user ordered the BlueBoat-Perception companion in MCS** (additions
> only, compatible with Control's requested "Replanner mode" row): `PerceptionConfig`
> (`config/settings.py`), `perception_command` / `perception_dataset_name` and the third
> companion (`ros/launch_manager.py`), the launch dialog's **Session** row (*Normal
> session* / *Dataset creation*, field `LaunchParameters.perception_mode`, no launch
> argument), the smoke test's `perception ok` block and the MCS docs. Every existing
> `to_cli()`, launch target and sea companion is unchanged (pinned). No interface
> changed. BlueBoat-Control was not edited.

## 4. §4.3 Sonar acquisition — interface rows

Replace the two `/sss_ai/*` rows and add four. Contract: `BlueBoat-Perception/docs/topics.md`.

| Name | Type | Producer → Consumers | QoS | Notes |
|---|---|---|---|---|
| `/sss_ai/seabed_image` | `sensor_msgs/Image` 32FC1 | GCS → BlueBoat-Perception | RELIABLE, KEEP_LAST 32, VOLATILE (both ends, CM-4) | **Since 2026-10-16.** Every seabed image, recording or not: the raw float dB (= the session's `_world.npz:intensity_db`). `header.frame_id` = the image key `"<stream_id>/<image_id>"`, not a TF frame. |
| `/sss_ai/seabed_image/png` | `sensor_msgs/Image` mono8 | same | same | The display PNG, byte-equal to the saved one. |
| `/sss_ai/seabed_image/metadata` | `std_msgs/String` (JSON envelope) | same | same | The picture metadata (schema 5) in an envelope with the key. |
| `/sss_ai/seabed_analysis` | `std_msgs/String` (JSON) | GCS → *(no consumer in the current tree)* | default | Image metadata, **never pixels**. Schema = the picture metadata's (**5 since 2026-10-16**, 4 before; not "schema 1"). `detections` is always `[]`: the detections are on `/sss_ai/detections`. |
| `/sss_ai/detections` | `vision_msgs/Detection2DArray` | **BlueBoat-Perception → GCS** | default (RELIABLE, 10) | **The station's one detection source** (owner's order, 2026-10-16): one array per image, possibly empty. Every map marker, waterfall marker and `detections.csv` row comes from here; the GCS runs no detector. uid = `2_000_000 + 16·image_id + k`. |
| `/sss_ai/image_records` | `std_msgs/String` (JSON, `blueboat_perception/image_record/1`) | BlueBoat-Perception → *(future belief layer; offline analysis)* | RELIABLE, KEEP_LAST 100 | One per image, with or without detections (`detections: []` = searched, nothing found; `null` = no pixels). Offline over the session = live, byte-equal canonical JSON minus `provenance` / `source`. |
| `/sss_ai/perception/status` | `std_msgs/String` (JSON) | BlueBoat-Perception → operator, tests | RELIABLE, depth 1, TRANSIENT_LOCAL | 1 Hz: counters (`late_parts`, `gaps`, …), latency, config. |

**Stale row to replace — "Transducer lateral offset — disagreement".** It says
`mount_y_abs_m: 0.20` in the sim and `TRANSDUCER_Y_OFFSET_PORT_M / _STBD_M = 0.0` in
`sss_processor_node.py`. Both are 0.285 now (VERIFIED: `sss_processor_node.py:122-125`
since SSS `994a74b`; `BlueBoat-SSS-Sim/blueboat_sss_sim/config/default_sonar.yaml:36`).
Proposed:
> **Transducer lateral offset — agreed at 0.285 m** (BR-101447 hull spacing / 2):
> processor `TRANSDUCER_Y_OFFSET_*`, GCS `core/svlog.py`, simulator `mount_y_abs_m`.
> Submersion 0.15 m is an estimate (= the sim's `sensor_depth_m`); fore/aft offset still
> 0.0, unmeasured. The seabed pictures georeference with it since 2026-10-16 (georef
> convention 2, schema 5).

## 5. §4.1 and §4.4 — attitude

- §4.1 `/blueboat/odom` row: append "The GCS takes per-row roll / pitch / yaw rate for
  the seabed-image metadata from `/mavros/imu/data` (2026-10-16), never from this
  topic's yaw-only real attitude."
- §4.4 MAVROS table, `/mavros/imu/data` row: Module `robot_interface`, **GCS**
  (BEST_EFFORT 10, metadata only, never geometry; since 2026-10-16).

## 6. §4.5 File-level interfaces, and the AI-input statement

Append to §4.5:
> **The detector input (BlueBoat-Perception decision 6.4, 2026-10-16)** is a fixed
> mapping (`fixed_gray_v1`) of the pictures' **raw dB** plus their JSON metadata (the
> per-row gain step and slant pitch), not of the display PNG. The PNG stays the operator
> picture and the `identity` baseline. CM-10 holds: same waterfall-domain, boat-relative
> grid.

Where the root or the `file-interfaces` skill says "the AI is fed the pictures plus their
JSON metadata", add "(since 2026-10-16: their raw dB, `fixed_gray`; the PNG with
`preprocess: identity`)".

## 7. §6 Discrepancies — D7

Current: "No submodule implements them or is designated to own them; the GCS's unwired
`vision_msgs/Detection2DArray` subscription is the only wire slot anticipating any of
it." Proposed:
> **D7 — Unimplemented interface boundary, partly claimed (2026-10-16).**
> BlueBoat-Perception owns the detector boundary: `/sss_ai/detections` for the map,
> `/sss_ai/image_records` for the belief layer. The detector is still the dummy; the YOLO
> adapter is an interface stub; calibration is identity (`calibrated: false`, so
> `confidence` is no probability, §8.2 ⚠). The belief layer, initial CPP, replanner and
> mission manager remain unowned, and §5.2 / §8.1 / §8.2 govern them. Belief's input
> contract is the image record (`BlueBoat-Perception/docs/topics.md` §4), typed messages
> once Belief freezes its needs.

## 8. §7 Where the detail lives

Table row:

| Topic | Read |
|---|---|
| Detector pipeline, detector input (fixed mapping, mask, turn flags), image records, dataset mode, offline = live | `BlueBoat-Perception/.claude/CLAUDE.md` · open items `BlueBoat-Perception/.claude/TODO.md`; evidence `docs/PREPROCESSING_STUDY.md`, `docs/LIVE_CHECKS.md` |

CLAUDE.md map: "module roots: `<module>/.claude/CLAUDE.md`, all **six** submodules".

## 9. Skill `module-gates` — new row

| Module | Gates, current state | Fresh clone |
|---|---|---|
| `BlueBoat-Perception` | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest` from `BlueBoat-Perception/blueboat_perception/`: **79 pass / 4 skip on 2026-10-16** (the skips: 3 corpus folders without stored detections, and the Aug Studio sidecar test, which needs pydantic — run it with `SSS-Dataset-Aug-Studio/.venv/bin/python -m pytest tests/test_dataset.py`, 9 pass). Corpus tests read `$BLUEBOAT_DATA_DIR/data/SSS_data` or `~/ros2_ws/data/SSS_data` and skip without it. No lint or type-checker. SSS's `tests/test_perception_golden.py` and `test_perception_bridge.py` are the other half of the contract (SSS gate). | Everything (tests, `pytest.ini`, tools, docs, the packaged `core/data/fixed_gray_v1.json`); no hooks exist. |

The `BlueBoat-SSS` row: "**353 pass / 50 skip on 2026-10-16**" (was 297 / 50 on
2026-09-28; red 298 / 4 fail / 50 on 2026-10-16 before any edit, repaired as a labelled
baseline repair).

The `BlueBoat-MCS` row: 30 checkpoints (+ `perception ok`, 2026-10-16).

## 10. Skill `file-interfaces` — new rows and one stale statement

| Artifact | Producer → Consumer(s) | Contract |
|---|---|---|
| Perception record logs `<root>/data/Perception_data/live/<stream_id>.jsonl`, offline runs `<root>/data/Perception_data/offline/<session>/` (`records.jsonl`, `detections.json`, `summary.json`) | `perception_node` (live) / `blueboat-perception-offline` → belief layer (future), offline analysis | One canonical JSON record per image (`blueboat_perception/image_record/1`). Equality = byte-equal minus `provenance` / `source`. Names claimed exclusively (`-2` …); a session is opened read-only and no output may sit inside it (CM-7). `<root>` resolved as Control's `data_root`. |
| Perception dataset `<root>/data/Dataset_data/<name>/` | `perception_node` in `mode:=dataset` (MCS **Session: Dataset creation**, personalized-world runs) / the CLI's `--dataset` → detector training, Aug Studio | `images/` = the **detector input** (`fixed_gray`) + a per-image `<stem>.json` sidecar of Aug Studio `AcquisitionMeta` fields (`intensity_mapping: log`, `log_range_db` = the window width; validated against the Aug Studio's own model); `display/` = the GCS PNG; `raw/<stem>_db.npz`; `labels/` empty (no label source yet); `manifest.json`, `data.yaml`; `ground_truth_bag/` = the simulator's ground-truth topics, recorded by a separate `ros2 bag record` the launch file starts. Perception's node never reads it. Write-once. |

**Stale in the `sss_aug_dataset.yaml` row:** "The GCS seabed imager writes dB with a
per-image 2.0 / 98.0 stretch". Since the display model (2026-09-05), that stretch is only
the legacy fallback with `mosaic.nadir_contrast: false` (VERIFIED,
`blueboat_gcs/core/seabed_imager.py:246`). The default PNG is the display model's
`power` transfer. Also add: "BlueBoat-Perception's datasets declare the mapping per image
in sidecars instead (above); still no in-project producer of `sss_aug_dataset.yaml`."

## 11. Also found (no row of this module, reported for your judgement)

`project_synthesis.md` §13's "Bhandari" (see `docs/REFERENCES.md`, discrepancies).
