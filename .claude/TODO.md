# TODO — BlueBoat-Perception

Open items only, and the phase task list. Settled design lives in `docs/DESIGN.md` and
the module's `.claude/CLAUDE.md`; the detector-input decisions in
`docs/PREPROCESSING_STUDY.md` §6.

## Done (details in DESIGN, STUDY §6, LIVE_CHECKS)
Phases 0–5 (2026-10-15/16: design, build, live checks 1–6, preprocessing study and
decisions 6.1–6.7, handover); tiled YOLO task phases 0–5 (2026-10-23/24: tiling, merge,
adapter, tile datasets with per-world classes, training and evaluation tools).
CR-PER-2 and CR-PER-1 (2026-11-15).
- **CR-PER-2:** georef convention 3 is placed from the transducers' point. It is bit-exact
  with the station's grids on 259/259 rebuilt pictures, and conventions 1 and 2 are
  byte-identical on 1,065 pictures.
- **CR-PER-1:** the dataset labels use the transducer pose, confirmed by a run.
- **R-5:** the convention-3 twin test was added to BlueBoat-SSS by your order.
- Evidence: `~/ros2_ws/sim_runs/sim_acceptance/06_sss_followup_perception/handoff_perception.md`.

## Next
- [ ] **Phase 6 — run the comparison (owner):** build the simulated worlds in dataset
      mode, then the README recipe (tile datasets → YOLO11s / YOLO26s × arms (a) full
      640, (b) 256/128, (c) 256/64; merge nms vs greedy_nmm on (b); 3- vs 1-channel →
      evaluate on val, choose, then report once on test). Results "on simulated data".

## Open technical items
- [x] **Owner: rebuild the install** (checked 2026-11-18: done; the installed `core/geometry.py` has
      `fore_aft_arm`, so the installed node places convention-3 pictures from the transducers' point).
- [ ] **The fore/aft arm −0.345 m is adopted from the simulator's profile, not measured**
      (session 06, CR-OWN-1). On the real boat it counts from the navigation fix. Perception
      reads it from each picture's `georef`, so a measured value needs no change here.
- [ ] **Ray-cast runs: label box about 0.1 m ahead of the highlight** (measured 2026-11-15,
      CR-PER-1 check b, ladder runs, along-track-symmetric objects).
      - Box centre minus highlight centroid, along the heading, both passes the same sign:
        `harbour_fast` +0.10 / +0.04 m, `harbour_live` +0.12 / +0.09 m (east / west).
      - The geometric run gives +0.01 / +0.02 m.
      - This is not the fore/aft arm: a navigation-pose label would sit 0.345 m behind, the
        opposite sign.
      - Largest for flat, wide objects (pallet, bicycle, mesh crate; (2026-11-18) all three are among the 22
        classes deleted in the World Builder's object set version 2).
      - Cause not investigated: the ray-cast's echo timing or beam, or the labeller's
        ray-cast path. Check before training on ray-cast datasets.
- [ ] **A session rebuilt from its `.svlog` cannot be labelled from its run's ground-truth
      bag.**
      - The rebuild stamps rows on the `.svlog`'s clock: 5.86 s before the live ROS clock
        on the ladder run.
      - The labeller pairs a row with its ping only within `GroundTruthBuffer.match_s` = 5 s.
      - The offline tool then writes empty labels (sidecar `ground_truth_coverage: 0.0`)
        and prints no warning.
      - Possible fixes: warn when the coverage is 0; accept a clock offset.
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
      (temperature / Platt, R11, DESIGN §4.5), one held-out split per domain (sim, real),
      judged by reliability diagrams + ECE. Not started; `calibrated` stays false.
- [ ] **Not now (listed only):** upscaled tiles (2× to 512); ground-range resampling (R26:
      consistent feature width across range, never evaluated on detection).
- [ ] Tile sidecars are not Aug Studio inputs (a tile does not start at the nadir). To
      augment, run the Aug Studio on the full-picture dataset first; the tile writer
      then needs the augmented dataset to keep `perception.labels` (highlight columns) —
      not checked.
- [ ] Option (i) leaves fragments with < 50 % of their highlight unlabelled in a tile;
      Ultralytics' tips say partial labelling "will not work". Measure how often it
      happens on the first tile datasets (`counts.*.dropped_partial` in the manifest).
- [ ] Live check 7 (the tiled path in a world mission, `LIVE_CHECKS.md`): needs weights.
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
- [ ] **Label geometry approximations** (measured, check 6):
  - rested or tilted objects use their `length × width` footprint at `yaw`, ignoring
    roll/pitch;
  - round objects (tyres, drums) use the bounding rectangle, a little wide at 45°;
  - the far edge runs a median 8 bins (up to ~27 cm) past the rendered shadow: the
    conservative max of the flat-floor tip and the renderer's mask length. (2026-11-18: the
    end is now the flat-floor tip only, PER-LBL-1; on the production ray-cast it still ends
    +9.1 / +14.4 bins past the picture's half-level crossing on the two measurable boxes of the
    C2 mission. A shadow end read off the rendered picture itself would close that; a ray over
    the terrain-only seabed was tried and was worse, +41.5 bins on the mooring block.)
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
- [ ] Aug Studio intensity mapping for `identity` datasets only (REQUESTS R-3; `fixed_gray`
      datasets declare `log`, resolved).
- [ ] Measure σ_nav, σ_ψ, σ_h against USBL / RTK. They are assumptions today
      (DESIGN §4.6).
- [ ] Typed messages for the record once Belief freezes its needs (DESIGN §13).
- [ ] When MCS gains Control's "Replanner mode" row: its own row and field beside Session
      (compatible by construction, REQUESTS coordination note).
