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
      updates, `docs/ROOT_CLAUDE_ROWS.md` for you to apply, the report.

## Open technical items
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
- [ ] YOLO runtime and adapter. No torch, ultralytics, onnxruntime or GPU on this machine.
      Tile across-track (256 × up to 2402), NMS across tiles (DESIGN §4.4, §5).
- [ ] Station uids repeat across GCS streams (image ids restart at every START), as the
      station's own did; a stream-aware uid needs a GCS listener + contract change.
- [ ] Calibration: held-out split per domain, temperature/Platt, reliability + ECE
      (DESIGN §4.5, R11).
- [ ] Offline sim labeller over a dataset's `ground_truth_bag/` (the simulator is not
      modified in this work).
- [ ] Aug Studio intensity mapping for `identity` datasets only (REQUESTS R-3; `fixed_gray`
      datasets declare `log`, resolved).
- [ ] Measure σ_nav, σ_ψ, σ_h against USBL / RTK. They are assumptions today
      (DESIGN §4.6).
- [ ] Typed messages for the record once Belief freezes its needs (DESIGN §13).
- [ ] When MCS gains Control's "Replanner mode" row: its own row and field beside Session
      (compatible by construction, REQUESTS coordination note).
