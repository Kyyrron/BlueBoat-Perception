# TODO — BlueBoat-Perception

Open items only, and the phase task list. Settled design lives in `docs/DESIGN.md` and
the module's `.claude/CLAUDE.md`; the detector-input decisions in
`docs/PREPROCESSING_STUDY.md` §6.

## Phase status
- [x] **Phase 0** — understand, references, questions (answers Q1–Q8, 2026-10-15).
- [x] **Phase 1** — design documents (2026-10-15), approved 2026-10-16 with answers 1–7.
- [x] **Phase 2** — build (2026-10-16):
  - [x] SSS gate baseline re-measured before any edit: **298 / 4 fail / 50**. The four were
        stale expectations after SSS `994a74b`, repaired as a labelled baseline repair
        → 302 / 0 / 50.
  - [x] SSS golden captured from the unmodified code (`tests/fixtures/perception_golden.json`,
        146 images, deterministic, reproduced byte for byte).
  - [x] Perception core + tests (59 pass / 4 skip); ROS node; launch (`mode:=dataset` claims
        the folder and starts the archival ground-truth bag); offline CLI; dataset writer.
        `colcon build --packages-select blueboat_perception` OK; both launch modes start and
        stop cleanly.
  - [x] SSS: one detection source (the in-process dummy removed), image publishing,
        `stream_id`, schema 5, georef convention 2 (pictures, clicks, overlay), Q8 attitude
        + ping numbers (live and replay), the per-detection stamp, Run AI through this core,
        and the separate profile-miss fallback fix. Gate **351 / 0 / 50**.
  - [x] MCS: `PerceptionConfig`, `perception_command`, the Session row; `smoke_test.py`
        PASSED (+ `perception ok`), `ruff check .` clean.
  - [x] Docs: REQUESTS (R-1 and R-2 withdrawn), DESIGN, topics, SSS and MCS docs.
- [x] **Phase 3** — live checks (2026-10-16), `docs/LIVE_CHECKS.md`:
  1. a sim world mission launched through MCS's `LaunchManager`, with the GCS running:
     - Normal: 47 images → 47 records → 47 detections back on the GCS map, 0 gaps;
     - Dataset: 27 / 27 / 27; the dataset PNGs are byte-identical to the session's, the
       Aug Studio loads it, and the ground-truth bag holds 5297 messages;
  2. offline = live: 47 / 47 and 27 / 27 records identical;
  3. latency: Perception 25.5 ms p50; image → detection back in the GCS 51 ms p50 / 62 ms
     p95; image 0 of each stream ~3.06 s (below);
  4. the real-boat commands, for you.
  - Found and fixed: a late PNG was counted as a duplicate (now `late_parts`).
- [x] **Phase 4** — measurements M1–M6 + recommendation (`PREPROCESSING_STUDY.md` §5–§6,
      `tools/preprocessing_study.py`, `docs/data/phase4_2026-10-16.json`).
- [x] **Decisions §6.1–§6.7** (yours, 2026-10-16), implemented the same day
      (STUDY v2 §6; `LIVE_CHECKS.md` check 5):
  - 6.1 originals preserved, detector input separate (`DetectorInput.gray`; dataset
    `images/` + `display/`);
  - 6.2 turn flags per row and detection, nothing excluded;
  - 6.3 GCS speed window 5 s → 2 s (SSS, `along_track.window_s`);
  - 6.4 `fixed_gray` (`fixed_gray_v1`) is the default detector input, `identity` the
    baseline;
  - 6.5 `near_range_mask_m` 0.0 / 0.75, default 0.0;
  - 6.6 roll correction behind a disabled boundary (raises), roll kept in the records;
  - 6.7 geometric correction, despeckling, HE, Retinex deferred (raise).
- [x] **Phase 5** — handover (2026-10-16): `.claude/CLAUDE.md`, this file, SSS `.claude`
      updates, the root `CLAUDE.md` and the `module-gates` / `file-interfaces` skills (applied
      by your order), the report.

- [x] **Tiled YOLO task (2026-10-23)** — brief "tile the long seabed images for YOLO":
  - [x] Phase 0–1: verification, questions (answers: visibility 2 dB as recommended;
        tile edges B + (i) after a literature check; pad short pictures; box centre kept;
        `map_min_confidence` 0.10; cap 999; worlds by the owner; Ultralytics a
        superproject dependency later);
  - [x] Phase 2: `core/tiling.py`, `core/merge.py` + tests;
  - [x] Phase 3: `detectors/yolo.py` (Ultralytics, lazy), pipeline / node / CLI / config,
        records + status (additive), offline = live tested on CPU with real Ultralytics;
  - [x] Phase 4: `tools/make_tile_dataset.py` + class-drift checks (all three creation
        paths use `yolo_classes_v1`; the simulator `dev` CATALOG = v1, 29, same order);
  - [x] Phase 5: `tools/train_yolo.py`, `tools/evaluate_detector.py`, docs; a synthetic
        smoke run (plumbing only);
  - [ ] **Phase 6 — run the comparison (owner):** build the simulated worlds in dataset
        mode, then the README recipe (tile datasets → YOLO11s / YOLO26s × arms (a) full
        640, (b) 256/128, (c) 256/64; merge nms vs greedy_nmm on (b); 3- vs 1-channel →
        evaluate on val, choose, then report once on test). Results "on simulated data".

## Open technical items
- [ ] **Tune the visibility threshold (to do once out of cloud mode, owner 2026-10-23).**
      2 dB is provisional (`tools/label_visibility.py`). On the first simulated datasets:
      `tools/draw_labels.py <dataset> --out DIR --stats`, look at the
      `highlight − background` / `shadow − background` histograms and the faint share,
      and set the threshold from them (a new value is a parameter, recorded in each tile
      manifest).
- [ ] **Decide the merge rule** (`nms` default vs `greedy_nmm`) on the val worlds, write it
      with its evidence in DESIGN §4.10, then set `merge` in `config/perception.yaml`.
      Known: greedy anchors on the best-scored box of a group; at the 0.01 detector
      threshold weak boxes can widen a union.
- [ ] **Calibration data:** once trained weights exist, the held-out simulated worlds
      (never the test worlds used for the comparison) are ready for a calibrator fit
      (temperature / Platt, R11, DESIGN §4.5). Not started; `calibrated` stays false.
- [ ] **Not now (listed only):** upscaled tiles (2× to 512); ground-range resampling (R26:
      consistent feature width across range, never evaluated on detection); a
      calibrator.
- [ ] Tile sidecars are not Aug Studio inputs (a tile does not start at the nadir). To
      augment, run the Aug Studio on the full-picture dataset first; the tile writer
      then needs the augmented dataset to keep `perception.labels` (highlight columns) —
      not checked.
- [ ] Option (i) leaves fragments with < 50 % of their highlight unlabelled in a tile;
      Ultralytics' tips say partial labelling "will not work". Measure how often it
      happens on the first tile datasets (`counts.*.dropped_partial` in the manifest).
- [ ] Live check 7 (the tiled path in a world mission, `LIVE_CHECKS.md`): needs weights.
- [ ] The simulator's GitHub default branch `main` (0225a7f) has **13** World Builder
      classes; `dev` (pinned by the superproject) has the 29 of `yolo_classes_v1`. A
      checkout of `main` beside this module fails the class-map test.
- [ ] **The first image of every stream arrives ~3.06 s late** (one part; all three runs).
      Consistent with a DDS heartbeat repair of the first large RELIABLE sample (Fast DDS
      default period 3 s); not verified, nothing lost. Check on the real boat; a warm-up
      sample or a shorter heartbeat in the QoS / an XML profile would remove it.
- [ ] **Train and compare the detector on both inputs** (`fixed_gray` vs `identity`) and
      both masks (0.0 / 0.75) on the same real split, before claiming either helps
      (STUDY §6.4–6.5, next experiments). Neither is shown to improve detection today.
- [ ] **The 1 s turn window and a 1 s GCS speed window are not validated on real data.**
      The real logs have no reference for ω or v (STUDY §6.2–6.3, M2b); check against an
      EKF / RTK track.
- [ ] **TVG of the Omniscan 450 SS is not identifiable** from its output (STUDY §1.4).
      `fixed_gray_v1` assumes no law, so a firmware or setting change that alters it needs
      a new mapping file, not a refit in place.
- [ ] **Sim/real level gap** under `fixed_gray`: real medians 163–172, simulated 165–193
      (up to ~4 dB brighter; STUDY M7). Synthetic training data inherit it (CM-11).
- [ ] Roll (6.6): measure on a Gazebo wave scenario and a real log with `/mavros/imu/data`
      before any correction is enabled.
- [ ] Station uids repeat across GCS streams (image ids restart at every START), as the
      station's own did; a stream-aware uid needs a GCS listener + contract change.
- [ ] Calibration: held-out split per domain, temperature/Platt, reliability + ECE
      (DESIGN §4.5, R11).
- [x] **Dataset labels from the simulator's ground truth** (2026-10-16, your order and
      decision: read in the node, dataset mode only). `core/sim_labels.py`,
      `ros/ground_truth.py`, the offline `--world --ground-truth-bag` re-labelling,
      `tools/draw_labels.py`. Checked against noise-free renders of recorded pings
      (`LIVE_CHECKS.md` check 6).
- [ ] **Label geometry approximations** (measured, check 6):
  - rested or tilted objects use their `length × width` footprint at `yaw`, ignoring
    roll/pitch;
  - round objects (tyres, drums) use the bounding rectangle, a little wide at 45°;
  - the far edge runs a median 8 bins (up to ~27 cm) past the rendered shadow: the
    conservative max of the flat-floor tip and the renderer's mask length.
- [ ] **Faint objects get labels the picture barely shows.** Ropes, chains and small
      plastic items at long range: in 67 of the 245 pings where the geometry places a
      single object (live run, check 6), the noise-free render changes by < 1 dB.
      **Partly addressed 2026-10-23:** the tile writer applies a measured-contrast rule
      (2 dB on the raw dB, `tools/label_visibility.py`; threshold to tune, above). A
      renderer-side contrast would need the simulator to publish it.
- [ ] **Label range margin: code 2 bins, comments 5.** `sim_labels.py`:
      `DEFAULT_RANGE_MARGIN_BINS = 2` (DESIGN §7: "±2 bins") while the module docstring and
      the constant's own comment derive 5 bins from the renderer's boxcar at the default
      acquisition. Check 6 was measured with 2. Found 2026-10-23, not changed (labels are
      not this task's); decide which is meant before the next labelled datasets.
- [ ] **SSS vs simulator bin convention.** The GCS pitch is `L/(N − 1)`
      (`models/sonar.py:186`, sample 0 at the start, the last at the end). The
      simulator renders bin k over `[k·L/N, (k + 1)·L/N)`. That is 0.08 %, under one bin
      at full range. The labels use the simulator's, and the GCS georeferencing carries
      the 0.08 %.
- [x] `data.yaml` train/val placeholders: superseded by the tile datasets' `data.yaml`
      (`images/{train,val,test}`), trained with Ultralytics 8.4.184 in the smoke run.
- [ ] Aug Studio intensity mapping for `identity` datasets only (REQUESTS R-3; `fixed_gray`
      datasets declare `log`, resolved).
- [ ] Measure σ_nav, σ_ψ, σ_h against USBL / RTK. They are assumptions today
      (DESIGN §4.6).
- [ ] Typed messages for the record once Belief freezes its needs (DESIGN §13).
- [ ] When MCS gains Control's "Replanner mode" row: its own row and field beside Session
      (compatible by construction, REQUESTS coordination note).
