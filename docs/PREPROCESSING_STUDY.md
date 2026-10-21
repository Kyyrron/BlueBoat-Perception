# Preprocessing study — what the detector sees, and what boat motion does to it

**Status: v2 (Phase 4, 2026-10-16).** This version contains:
- §1, the current chain, raw ping to detector input;
- §2, the motion analysis;
- §3, attitude availability;
- §4, the candidate options;
- §5, the data, with the Phase 4 measurements M1–M7;
- §6, the owner's decisions on the recommendation (2026-10-16) and what was implemented.

**Implemented (§6):** the default detector input is `fixed_gray` (`fixed_gray_v1`, a
fixed mapping of the raw dB); `identity` (the GCS display PNG) stays one switch away as
the baseline. The near-range mask and the turn flags are in, the originals are never
modified, and roll correction, geometric correction, despeckling, histogram
equalisation and Retinex are not implemented (requesting one raises).

Sources: [`REFERENCES.md`](REFERENCES.md), pointers `[R# §…]` and `[P# §…]`.
"Verified" means read in the code at the path given, in the tree as of 2026-10-15. Numbers
marked *derived* are first-order geometry computed here from the cited formulas. Numbers
marked *measured* come from the recorded sessions in `~/ros2_ws/data/SSS_data` (§5 lists
them).

**Why the question matters.** The future detector is trained on simulated images and later
run on real ones (CM-11 keeps the headline detector real-trained; a synthetic-trained one
is for the E7 transfer gap [P1 §7.6]). So **any preprocessing must be identical for
simulated and real data, and justified**. The detector must also see boat-relative
waterfall images, never world-frame mosaic crops [P1 §5.2 ⚠, CM-10].

---

## 1. The current chain, raw ping → detector input

For each step: what it does, where it happens, and whether simulated data go through the
same code.

### 1.1 Acquisition and per-ping scaling
- **What:** each Omniscan profile carries `pwr_results` (u16) normalised **per ping** onto
  `[min_pwr_db, max_pwr_db]` (span ≤ 90 dB) [P2 §4.3]. The driver
  (`BlueBoat-SSS/blueboat_sss/src/sss_node.py`) sends range, pulse, gain index and
  `num_results` to the device. **No TVG parameter is sent**; none exists in the driver
  (verified). `sss_helper.scale_to_db` inverts the normalisation exactly (verified).
- **Gain:** the real device runs auto-gain (`gain_index −1`), and the index walks during a
  recording. The simulator resolves `−1` to index 4 and holds it [P2 §4.3; P7 §6]. Both
  report `analog_gain`.
- **Same for sim?** Encoding: yes, the simulator reproduces the per-ping scaling (sim
  `sonar/encoder.py`, P9 §4). Gain *behaviour*: **no** (fixed vs auto). It is neutralised
  only in the display model, via `analog_gain` (§1.4).

### 1.2 Altitude, slant range and ground range
- **FBR / altitude:** `sss_helper.detect_fbr_slant_m` uses noise + 8 dB with 3-sample
  persistence. `_SideTracker` bootstraps over 10 pings within 0.30 m and rejects outliers
  > 1 m. `FBRTracker` fuses `max(port, stbd)`, falling back to provisional, then
  last-known, then 0 (verified, `sss_processor_node.py:131-144`). This is the classical
  threshold family [R9 §I]. The FBR is "the sensor's primary altitude" and is needed for
  ground-range and speed correction [R9 §I]. Objects in the water column corrupt it, and
  R9 removes them by median-filtering the along-track FBR [R9 §III]; **our tracker has no
  such filter**.
- **Slant → ground:** `project_side` computes `y = ±(0.285 + sqrt(s² − h²))` for
  `s > h` (verified, `sss_helper.py:176-233`). This is the flat-seabed formula
  [R1 §3.2.1 eq. 3.2, §4.5.1 eq. 4.3; R6 eq. 2 with hp = 0]. **It feeds only
  `ProcessedSSSPing` and the mosaic.** The seabed image stays in the **slant domain**
  (one column per device bin, `models/sonar.py` `build_slant_row`). Ground range appears
  only in its world grids (§1.7). Keeping slant range is accepted practice for target
  search [R1 §4.5.1, p. 65] and is what the AI4Shipwrecks detector input is [R12 §3.6].
- **Flat-floor error for our targets** *(derived, R6 eq. 3)*: a 0.5 m-tall object point
  (P1 §3.1 floor) at altitude h = 1 m is misplaced in ground range by **0.08 m at 5 m slant**
  and **0.20 m at 2 m slant**. That is 3–12 range bins at 16.7 mm. It is not a preprocessing
  issue (it is geometry) but it belongs in the localisation uncertainty (DESIGN §5.4).
- **Same for sim?** Yes: one processor for both.

### 1.3 Water column, nadir and ringing
- **In the image:** the GCS **keeps the water column**, darkened by the display model and
  never erased [P4 *Seabed images*; P6 §1, §9.3]. Burguera & Oliver instead discard the
  blind zone as uninformative [R6 "Blind zone removal"]. Blondel notes a near-nadir
  high-backscatter band on all sidescan images that "cannot be removed short of masking"
  [R1 §10.4.1, p. 236].
- **Nadir blank:** live images have **NaN in the first 0.75 m of slant range**
  (`live_native.attach_native`, verified). Images generated offline from a `.svlog` do
  **not** (`core/svlog.py` blanks only the ground projection). So **live and
  offline-generated pictures differ** in the first 0.75 m. For training data built offline
  vs inference live, that is a domain difference, small and confined to the ringing core.
- **Same for sim?** Yes for live runs.

### 1.4 Gain, TVG and across-track intensity normalisation
- **On the data: none.** `intensity_db` in the image (and on the wire to Perception) is
  the device dB after `scale_to_db`. No TVG, gain step, beam pattern or grazing-angle
  correction is applied (verified, `seabed_imager.py`, `display_model.py`).
- **Does the device apply TVG? Not identifiable from its output** (re-examined
  2026-10-16, code and configuration):
  - *what is verified:* the driver exposes six acquisition parameters and none is a
    gain law (`sss_node.py`); the profile message carries `gain_index` / `analog_gain`
    but no TVG field (`OmniscanProfile`); the GCS removes the reported gain step, then
    `TL = 40·log r + 2αr`, then a fitted curve `A` (`display_model.py`);
  - *what the data show:* the median real seabed level falls **~44 dB per decade** of
    slant range between 2.4 and 17.6 m on the beach sets (the `fixed_gray_v1` reference
    curve, §6.4); P6 §8 reports 60–70 dB per decade on its logs;
  - *why the two documents disagree:* [P6 §2] read the stream as pre-TVG. The simulator
    fits `tvg_compensation: 0.30` on two altitudes [P7 §3, §6], but its own §6 table
    says only `spreading_exponent · (1 − tvg_compensation)` is determined. Both readings
    explain the same falloff. Like Ye et al., who note that TVG is usually hardware and
    its parameters "cannot be easily obtained" from images [R20 §2.1], we cannot separate
    a partial device TVG from the seabed's own angular response.
  - *consequence:* no part of the detector input assumes a TVG law (§6.4). P6 §2's
    stale "simulator uses 0" was corrected on 2026-10-16; the simulator's wording is left
    as it is (the simulator is not modified in this work).

- **In the display only:** `DisplayModel.normalise` computes
  `e = db + TL(r) − A_side(r/h) − g`. TL is `40·log r + 2αr`; `A` is an empirical per-side
  curve in `r/h`, fitted by histogram mode plus a consensus line; `g` is the receive-gain
  step `20·log10(analog_gain/74.55)`. Then a power transfer γ 0.5 with a knee [P4 *One
  display model*; P6 §9]. It is stored with every picture and invertible.
- **It drifts during a live mission.** The live model is **re-published every 300 rows,
  never frozen** (`display_model.py:624-642`, since 2026-09-19; P4 still says "freezes").
  Before `ready`, pictures use a per-image 2–98 % stretch. Offline exports fit the model
  once over the whole log. So **the same seabed can map to different PNG values live vs
  offline, and early vs late in a mission.** The raw dB does not drift.
- **Same for sim?** Same code. The *input* differs only as far as the simulator's acoustic
  model differs from the device: the TVG fraction is one fitted parameter of that model.

### 1.5 Along-track sampling
- **Ping spacing and beam footprint** *(derived)*: at 20 Hz, consecutive pings are
  `v/20` apart, **2.5 cm at 0.5 m/s and 5 cm at 1 m/s**. The along-track beam footprint is
  `R·sin θh` with θh = 0.5° [R1 §2.5 eq. 2.10; R12 §3.4 eq. 1; P1 §1]: **8.7 cm at
  10 m, 17.5 cm at 20 m, 26 cm at 30 m**. Along-track is therefore oversampled except in
  the first ~3–6 m.
- **The GCS "square" geometry is anamorphosis.** Each picture row is one bin pitch of
  along-track distance (16.7 mm at 20 m / 1200 bins), copying the **nearest ping's bins
  verbatim**: replication and sub-sampling, no interpolated samples
  (`seabed_imager._emit_square`, verified). This is Blondel's along-track speed correction
  [R1 §4.5.2, fig. 4.15]. Interpolation would risk "creating new data or distorting
  existing features" [R1 §2.5, p. 31].
- **Its speed is a 5 s median** of chord speeds × the median ping period
  (`core/along_track.py`). Blondel's warning is that a wrong or constant assumed speed
  stretches or compresses structures [R1 §10.4.2 fig. 10.13; §10.5.1]. *Measured:* the
  per-image ratio of row-pose path length to the nominal span has median 1.00 but ranges
  **0.66–1.31** over the 9 recorded sets (§5), so individual pictures are up to ±30 %
  off in along-track scale.
- **Same for sim?** Yes.

### 1.6 Interpolation
- None in the waterfall or the seabed images. Densification and gap fill exist only in the
  mosaic, and gap fill is off by default (verified). The project rule is that the AI must
  not see interpolated data [P5 §5], consistent with stating any interpolation explicitly
  [R1 §3.4, p. 46].

### 1.7 Pose, heading and transducer offsets
- **Pose per ping:** nearest `/blueboat/odom` sample, **no interpolation**, 1 s tolerance
  (`_OdomBuffer.nearest`, verified). Live, the GCS then replaces yaw with the
  **latest compass heading at reception** (`MainWindow._restamp_ping_heading`) and may
  re-stamp frozen poses (`_align_ping_pose`).
- **Real `/blueboat/odom` is yaw-only.** `robot_interface.odom_callback` writes
  `yaw_to_quaternion(yaw)` (verified). The simulator's odom carries full 6-DOF (§3).
- **Lever arm:**
  - the processor uses Y offsets of 0.285 m and 0.15 m submersion since SSS `994a74b`
    (2026-10-13; root P2 §4.3 and P3 still say 0.0);
  - the simulator uses mount 0.285 m and depth 0.15 m (verified,
    `config/default_sonar.yaml:34-36`; P7 §4 still says ±0.20).
  - **The seabed-image world grids (georef convention 1)** use `altitude_m` =
    `water_depth` = altitude + 0.15 as the slant→ground altitude and **no lateral offset**.
    Offline-generated pictures use the depth-mode altitude, 0 by default. → **Fixed in SSS
    by your order (Q6): convention 2** uses `bottom_m` and the 0.285 m offset, recorded in
    the metadata. Perception reads the convention from the metadata.
- **Same for sim?** Same code. The simulator's own lever arm now matches the processor's.

### 1.8 Display picture vs detector input
- **The PNG** is the display transfer (§1.4). It is the decided "AI feed" for annotation
  and training (P4, decided 2026-09-01), but it drifts live.
- **The wire / npz:** raw float32 dB plus the full metadata, including the display model.
  Perception receives exactly these values (DESIGN §3).
- **The detector input (2026-10-16, §6.4):** a separate picture built by Perception's
  preprocessor from the raw dB — `fixed_gray`, one fixed mapping (default) — or, as
  the baseline, the display PNG itself (`identity`). One configuration value switches
  them, and the record names the one used. The dummy detector uses only the image shape.

---

## 2. Motion effects

Parameters of our boat, all cited:
- altitude h: 0.6–1.6 m at the beach (*measured* session medians 0.82–1.61 m), ~5 m
  harbour (P6 §8);
- range: 20 m default, 15/30 m reserved [P1 §8.5; P2 §4.3];
- speed: 0.5–0.77 m/s *measured*, 1.0 m/s nominal [P1 §8.5];
- ping rate: 20 Hz [P1 §1; P2 §4.3];
- beam: 0.5° azimuth, 50° vertical, tilted 20° [P1 §1; P7 §3];
- field attitude (ATTITUDE 25 Hz, three logs) [P8]: roll std 0.93–1.96°, pitch std
  0.88–2.76°, roll/pitch-rate std 5–16 °/s, **yaw-rate std 5–14 °/s**. Simulated light
  chop has similar roll and pitch but **yaw rate 1.2–1.3 °/s, 4–10× lower**.

Each subsection answers four questions: what the literature says; the expected size for
us; whether our pipeline handles it; whether it can be measured from our logs and whether
it matters for detection.

### 2.1 Turns and yaw (the differential mode)
- **Literature:**
  - in turns, inside-corner swath portions overlap and outside-corner portions spread with
    incomplete coverage [R1 §4.7.2 fig. 4.17; R6 fig. 22];
  - yaw re-images features inside the turn from a different look angle and images others
    farther away outside it [R1 §10.4.2 fig. 10.18];
  - yaw makes the beams scan ahead or back alternately on the two sides, and severe yaw
    **backscans**, putting objects in triplicate [R14 §I.A fig. 2];
  - the along-track sampling displacement grows linearly with range in the yaw increment
    [R14 eq. 3b];
  - yaw is the antisymmetric (differential) mode [R8 §III.A.2];
  - yaw distorts more than equal pitch because range ≫ altitude [R14 §VI].
- **Size for us** *(derived)*:
  - *Turns.* The inner side folds (backscans) beyond `r* = v/ω`. At v = 0.5 m/s and
    ω = 0.15–0.3 rad/s (a U-turn), r* = 1.7–3.3 m, so almost the whole inner swath is
    re-imaged backwards. The outer side's along-track sampling spreads by `1 + ωr/v`, up
    to ~7× at 15 m. *Measured:* the largest per-image mean |ω| is 0.15–0.33 rad/s in the
    sim sessions and 0.29–1.35 rad/s in the real ones (§5).
  - *Yaw jitter on straight lines.* Ping-to-ping heading change δψ = ω/20. The field's
    5–14 °/s gives 0.25–0.7° per ping, so the along-track misregistration `y·δψ` is
    **4–12 cm at 10 m and 9–24 cm at 20 m per ping**. That is 2–10× the ping spacing
    (§1.5) and comparable to the beam footprint. In simulation it is 4–10× smaller (P8).
- **Handled?** No:
  - the square geometry uses the boat-centre along-track speed only (§1.5);
  - row yaw is lerped between pings;
  - nothing flags or corrects turn rows.
- **Measurable?** Yes:
  - row `t_s`/`yaw_rad` in every image's metadata;
  - per-row yaw rate after Q8, from `/mavros/imu/data`;
  - `.svlog` ATTITUDE at 25 Hz (with a caveat in §3).
- **Matters for detection?** Likely the most:
  - objects in turn rows are duplicated, mirrored or missing [R14];
  - on straight lines, real-only yaw jitter breaks thin highlights and shadow edges at far
    range, with a sim/real gap in exactly this effect (P8);
  - for Belief, outer-turn coverage has holes and inner-turn coverage is re-imaged from
    other look angles [R1 fig. 4.17].

### 2.2 Roll
- **Literature:**
  - roll does **not** produce geometric distortion, "the beam rotates within its own plane";
    it modulates intensity near nadir and at maximum range [R14 §I.A];
  - it "mainly modulates backscatter intensity via incidence-angle variation" [R8
    §III.A.1];
  - for systems that map to an assumed swath, unprocessed roll gives asymmetric
    across-track misplacement ≈ `H·tan φ` [R1 §10.4.2, p. 240];
  - on a small towed-USV sonar looking at a vertical wall, roll dominated over pitch and a
    per-ping roll correction cut object-size error by ~15 points [R7 §3.3, §4.2]. That is a
    different geometry (R7 caveat in REFERENCES);
  - a lever-arm error induces heave `dY·sin r` [R15 §(D,E)].
- **Size for us** *(derived)*:
  - geometric `H·tan φ`: **3.5 cm at H = 1 m, φ = 2°**; 17 cm at 5 m;
  - lever-arm lift `0.285·sin φ`: **1.0 cm at 2°**, 4.9 cm at 10° (P7 §4 states 35 mm on the
    old ±0.20 m mount);
  - radiometric: the 50°-wide vertical beam [P1 §1] shifts by φ, so ±1–2° (P8) moves the
    pattern by a few percent of its width. That gives port/starboard anticorrelated
    brightness banding [P7 §4].
- **Handled?** No (no attitude in the image path today).
- **Measurable?** After Q8 (per-row roll), or from `.svlog` ATTITUDE: correlate per-row
  side brightness with roll.
- **Matters for detection?** Geometrically, no, at our altitude. Radiometrically,
  possibly: along-track banding changes the local contrast a detector keys on [R12 §2.1].
  Sim and real roll magnitudes match (P8), so the sim/real gap here is small.

### 2.3 Pitch (the common mode)
- **Literature:**
  - pitch scans ahead or back on both sides simultaneously [R14 fig. 2]: the scan-line
    offset is `z·tan(pitch)` [R14 eq. 13b], and equivalently `D_at = h·tan θ`
    [R8 eq. 30];
  - pitch replicates or anticipates swath lines; abrupt pitch interrupts structures at all
    ranges [R1 §10.4.2 fig. 10.17];
  - pitch is the symmetric mode [R8 §III.A.2].
- **Size for us** *(derived)*:
  - `h·tan θ` with field pitch std 0.88–2.76° gives **1.5–4.8 cm at h = 1 m** (about one
    picture row) and 7.7–24 cm at 5 m. P8 measures the "pitch-like footprint wobble" at
    0.07 m at 1.4 m (beach) and 0.11–0.14 m at 7.3 m;
  - a static trim of +1.2…+2.8° [P8] is a constant shift and harmless.
- **Handled?** No. **Measurable?** Yes (Q8, `.svlog`). **Matters?** Little at the beach;
  more in harbour-depth water.

### 2.4 Heave and waves
- **Literature:**
  - heave gives symmetric across-track distortion; the altimeter can lock on false echoes
    [R1 §10.4.2, p. 238–239];
  - the FBR edge wobbles with vehicle motion [R9 §I];
  - the ping period is short against wave periods, so the error is constant within a ping
    [R15 "Manifestation"].
- **Size for us** *(derived)*:
  - sim light-chop heave std 0.035–0.039 m [P8];
  - the ground-range error from an altitude error is `Δg ≈ (h/g)·Δh`: about 2 cm at
    g = 2 m and 1 cm at 4 m for Δh = 4 cm;
  - in the slant-domain image, heave only moves the water-column edge by a few bins.
- **Handled?** Yes, per ping, through the tracked altitude (`bottom_m`), which the
  convention-2 world grid uses. **Measurable?** Yes (`bottom_m` per row). **Matters?**
  Little, except near nadir.

### 2.5 Speed variation
- **Literature:** a wrong assumed speed stretches or compresses along-track
  [R1 §10.4.2 fig. 10.13]; speed and course changes vary the aspect ratio [R14 §I.A];
  position and speed fluctuations look like pitch [R8 §III.A.1].
- **Size for us** *(measured)*: per-image along-track scale 0.66–1.31 (§1.5), so a 1 m
  object reads 0.66–1.31 m along-track.
- **Handled?** Partly: anamorphosis with a 5 s median speed.
- **Measurable?** Yes (row poses vs `along_track_m`).
- **Matters?** Moderately (object shape and aspect).

### 2.6 Position, heading and timing errors (georeferencing)
- **Literature:**
  - GPS CA ≈ 15 m, DGPS/RTK 1–5 m above water [R1 §4.2.1];
  - navigation records carry noise, drift, timing delay and filtering artefacts
    [R8 §I];
  - an attitude or heading delay δt gives an error = rate × δt, and sparse attitude must be
    **interpolated**, not taken as the latest sample [R15 §(B)].
- **Size for us** *(derived)*:
  - heading bias δψ gives an across-range error `y·δψ`: **0.7 m at 20 m for 2°**;
  - latency L in the "latest heading at reception" re-stamp gives ω·L: 0.25–0.7° for
    L = 50 ms at the field's 5–14 °/s, i.e. 9–24 cm at 20 m;
  - nearest-odom snapping is ≤ 25 ms × v ≈ 2.5 cm.
- **Handled?** No (no interpolation, no latency compensation).
- **Measurable?** The heading latency in Phase 3 (ping stamp vs reception time);
  absolute accuracy only against USBL / RTK ground truth.
- **Matters?** For Belief and localisation, more than for detection: the image is
  slant-domain and unaffected. It enters every detection's covariance (DESIGN §5.4).

---

## 3. Attitude available at ping time

| | Real boat | Simulation |
|---|---|---|
| `/blueboat/odom` orientation | **yaw only** (`robot_interface` uses `yaw_to_quaternion`); event-driven at the mavros odom rate (not configured in the repo) | full 6-DOF, 20 Hz (`blueboat.xacro` odometry plugin) |
| `ProcessedSSSPing.robot_orientation` | yaw only (copied from odom) | full (copied from odom). **The same field has different content in the two graphs: a latent CM-2 asymmetry**; nobody reads roll/pitch from it today |
| `/mavros/imu/data` | mavros (roll, pitch, yaw, body rates); ~25 Hz on the corpus [P8]; header stamps may be 0 on real runs [P2 §4.1] | `mavros_shim_node` from the odom quaternion and twist (verified, `mavros_shim_node.py:58,93`) |
| `/mavros/global_position/compass_hdg` | mavros | shim |
| `.svlog` ATTITUDE (id 150) | written by the processor from `/mavros/imu/data` while recording; **frozen `time_boot_ms` in 3 of 7 corpus files**, so pairing is by file order only (sim `TODO.md`) | same writer |
| heave | not measured (odom z discarded) | Gazebo pose; `/sim/sea_state/boat_pose` (100 Hz) is **simulation-only and must not be read online** |
| seabed-image metadata today | yaw (compass live, ATTITUDE yaw on replay), 5 s median speed, altitude, bottom | same |

**Missing, per our order Q8, to be added in SSS:**
- per-row roll, pitch and yaw rate, from `/mavros/imu/data` in **both** graphs, never from
  the ping quaternion, plus the attitude sample's age;
- device ping numbers.

**Timing caveat [R15 §(B)]:** the GCS attaches the *latest* compass/IMU sample at ping
reception. With unreliable mavros stamps on the real boat, a stamp-matched interpolation is
not available today. The age of the sample is recorded instead, and Phase 3/4 measures the
latency.

---

## 4. Options (evaluated in Phase 4, §5; decided and implemented as in §6)

Where each option would live follows the decided split (Q4): steps needing per-ping data
or changing image geometry go in SSS's imager; per-image steps computable from image +
metadata go in Perception's stage. **Every option must run identically on sim and real
input, live and offline.**

| | Option | Benefit for detection | Risk | Cost | Where | How to test |
|---|---|---|---|---|---|---|
| A | **Leave images as they are; give the detector and Belief motion metadata** (per-row yaw rate, roll, pitch, speed, flags) | No invented pixels; sim/real identical by construction; Belief can down-weight turn rows | The detector still sees distorted turn imagery | Low (Q8 already adds the metadata) | records (Perception) + metadata (SSS) | Record completeness; Phase 4 statistics of affected rows |
| B | **Flag or exclude rows in strong turns** (e.g. `r* = v/ω` inside the swath) from detection and dataset labels | Removes backscanned / duplicated objects [R14] | Loses coverage in turns; a threshold choice | Low | flag in SSS metadata (per-ping ω) or in Perception from row yaw | Fraction of rows flagged; re-run the offline records |
| C | **Better along-track resampling** (shorter speed window, per-ping chord speed) | Fixes the ±30 % per-image scale error (§2.5) | Noisier speed gives row jitter; still nearest-ping, no interpolation | Medium | SSS imager | Path/nominal ratio distribution before/after (§5) |
| D | **Attitude-based geometric correction** (yaw/pitch per ping, R8 / R14 style) | Reduces jitter misregistration at far range (§2.1) | Needs **time-accurate** attitude (§3 caveat); correction creates holes and duplicates, and hole-filling invents values [R8 §III.F.3], against [P5 §5]; methods were validated on AUV/towed data [R8 Table I] | High | SSS imager (per-ping) | Synthetic jitter in sim with known truth (`ground_truth/sensor`, offline only), then real |
| E | **Fixed across-track intensity normalisation for the detector** (a frozen, mission-independent TL + side curve or a Lambert / beam model [R6 eq. 12; R1 §2.2.3]) instead of the drifting display PNG | Stationary input; same seabed reads the same live, offline, sim and real (§1.4) | A model mismatch (the TVG fraction disagreement, §1.4) bakes a sim/real bias into the input | Medium | Perception stage (computable from dB + metadata) | Per-range intensity histograms sim vs real, before/after (P7 calibration logs) |
| F | **Near-nadir / water-column masking** for detection [R1 §10.4.1] | Removes the systematic nadir band and the live/offline 0.75 m blank difference (§1.3) | Loses near-range targets (≤ ~1.5 h) | Low | Perception stage | Fraction of labels affected |
| G | **Identity** (today) | Baseline; nothing invented | Detector sees drift (if PNG-trained) and motion artefacts | None | — | — |

Not proposed:
- **despeckling**, which "can lose valuable data" in target search [R1 §4.8.3; P6 §3];
- **histogram equalisation**, which is scene-dependent [P6 §5].

---

## 5. Data used here, and the Phase 4 measurements

**Recorded sets on this machine** (`~/ros2_ws/data/SSS_data`, read-only):
- the live sim sessions `sessions/2026_09_16-22_48_06`, `2026_09_17-11_27_57`,
  `2026_09_28-10_27_47`, `2026_09_28-10_40_17`, `2026_09_28-15_49_34`, `beach_sim`;
- the real, offline-generated `merged_sessions/bath1_2`, `bath1_2_3` and
  `beach_sept/seabed_images_object_boiteVsDistance`.

All of them predate 2026-10-13, so they are georef convention 1 with no lever arm.

**v1 measurements** (from the image metadata):

| Measurement | Values |
|---|---|
| Image cadence | 2.8–10.6 s between images |
| Per-image along-track scale (row-pose path / nominal span) | median 1.00, range 0.66–1.31 |
| Largest per-image mean \|yaw rate\| | sim 0.15–0.33 rad/s, real 0.29–1.35 rad/s |
| Session altitude medians | 0.82–1.61 m (sim and beach) |

**Phase 4 measurements (2026-10-16).**
- **Tool:** `tools/preprocessing_study.py`, read-only, using the GCS's own code. Raw
  results: `docs/data/phase4_2026-10-16.json`.
- **Data:**
  - the 9 recorded sets above;
  - the three Phase 3 live sessions (`docs/LIVE_CHECKS.md`: schema 5, IMU attitude per
    row, convention 2, the simulator's default world at ~3.5–4 m, a lawnmower with
    U-turns);
  - the real `.svlog`s `bath1–3` and `object_boiteVsDistance`, whose ATTITUDE is now
    decoded per ping (Q8).
- "Real" means the beach field data (0.8–1.6 m altitude, 20 m range).

**M1 — Turns and yaw.** Backscan on the inner side starts beyond `r* = v/|ω|`
[R14 §I.A].

| | Real (beach, 4 logs, IMU per ping) | Sim, recorded sessions | Sim, Phase 3 lawnmower |
|---|---|---|---|
| \|ω\| median, p95 (instantaneous) | 2.7–5.1, 11.6–25.6 °/s | 0.03–4.6, 16–19 °/s (row yaw) | 0–3.5, 5.1–5.4 °/s |
| \|ω\| over 1 s, median, p95 | 2.0–3.8, 10.4–24.7 °/s | — | ≈ the same (no jitter) |
| Pings with sustained (1 s) `r* < R` | **58–74 %** | 14–72 % of rows (mission-dependent) | 29–55 % (the U-turns) |
| Pings with sustained `r* < R/2` | **39–53 %** | 12–56 % (one 1-image session: 5 %) | 28–53 % |

Reading:
- On the real boat, more than half of all pings have some inner-side backscan inside the
  20 m swath, on straight lines too. The 1 s-averaged heading still swings at 2–4 °/s,
  so `r* ≈ 10–20 m` at 0.5–0.8 m/s.
- On straight lines the simulated boat has essentially no yaw, so the sim flags only its
  turns. This is the sim/real gap of §2.1, now measured per ping.
- Per ping the backscan is small (at 4 °/s and 20 m, ~4 cm backwards per ping, less than
  the 17.5 cm beam footprint). It is the *sustained* swing that re-images or folds the far
  range on one side.

**M2 — Along-track scale vs the speed window** (option C). Per full image (256 rows):
row-pose path / nominal span, p5–p95 [min–max]. The GCS's own `SeabedImager`, only
`along_track.window_s` changed:

| | 5 s (today) | 2 s | 1 s |
|---|---|---|---|
| Real (3 sets, 461 images) | 0.88–1.10 [0.66–1.31] ² | 0.94–1.07 [0.85–1.28] | **0.95–1.06** [0.89–1.14] |
| Sim recorded (6 sets) | 0.86–1.14 [0.52–1.25] | 0.97–1.07 [0.89–1.12] | **0.98–1.05** [0.93–1.08] |
| Sim Phase 3 (constant-speed controller) | 1.000–1.004 | 1.000–1.002 | 1.000–1.002 |

² Excluding one degenerate image of `bath1_2_3` (ratio 0.012 in every window: the boat
almost stationary over the whole tile).

The 5 s numbers reproduce v1's 0.66–1.31. A 1 s window narrows the per-image scale error
about threefold.

Caveat: on real data the reference is the autopilot's EKF pose track, so a shorter window
also follows its noise. The simulator's truth pose shows the same gain (0.86–1.14 →
0.98–1.05), which argues it is genuine. Not measured: the row jitter (repeated / skipped
pings per row) a shorter window adds.

**M3 — Roll vs side brightness.** Per ping, the gain-compensated median seabed dB per
side over `[max(2h, 2 m), 0.7R]`, port − starboard, against the IMU roll. Both are
detrended by a 101-ping running median.

| | Real: bath1 | bath2 | bath3 | object_boiteVsDistance | Sim Phase 3 (calm) |
|---|---|---|---|---|---|
| roll std | 2.3° | 2.4° | 2.0° | 2.5° | 0.003° |
| corr(roll, port − stbd) | **0.58** | 0.43 | 0.46 | 0.50 | ≈ 0 |
| slope | 0.53 dB/° | 0.59 | 0.62 | 0.61 | — |
| banding explained (rms) | 1.2 dB | 1.4 | 1.2 | 1.5 | — |

Roll is a real, consistent, anticorrelated per-side brightness modulation of
**0.53–0.62 dB per degree**. On the real boat that gives 1.2–1.5 dB rms along-track
banding: at the shipped γ = 0.5 that is ±15 % pixel brightness, and roughly a fifth to a
third of the side-difference variance. The calm simulation has none; a wave scenario
would be needed to see whether the simulator reproduces the slope.

**M4 — Attach latency** (Phase 3, `LIVE_CHECKS.md`):
- a ping's processed row reaches the graph 13.8 ms p50 / 18.1 ms p95 after its profile;
- the attitude sample the GCS attaches is 15 ms p50 / 93 ms p95 / 373 ms max old;
- with the real boat's yaw rates (p95 ~20 °/s), a "latest sample" heading is off by up to
  ~2° at p95, ~0.8 m at 20 m range;
- this is a localisation (Belief / covariance) error, not an image error: the picture is
  slant-domain. Interpolating on stamps [R15 §(B)] needs usable mavros stamps on the real
  boat (handled separately, REQUESTS).

**M5 — Raw level vs normalised range, and the picture's drift.**
- **Level.** Median raw dB, gain-compensated, at `x = s/h`:

  | x | 1.4 | 2.1 | 4.0 | 9.5 |
  |---|---|---|---|---|
  | Real beach (3 sets) | 57.7–61.8 | 54.8–59.2 | 43.8–49.7 | 24.2–31.1 |
  | Sim, post-2026-09-18 calibration, beach altitude (4 sets) | 63.8–71.6 | 58.7–67.2 | 47.3–56.8 | 30.8–39.6 |

  The simulator reads **~6–8 dB hot** on the set medians (at x = 2.1: sim 64, real 58 dB),
  with a similar slope; set to set, from level with the real data (2026-09-28-15_49_34) to
  12 dB above it. The pre-recalibration 2026-09-17 sessions read ~20 dB hot, as
  `sonar_model.md` records.
- **Drift of a live mission.** One image's raw dB rendered through every later image's
  stored model, as seabed-pixel median PNG grey level:
  - **6–29 grey levels** over a live sim mission (Phase 3: 6–12; the 2026-09-28 sessions:
    8–29);
  - `hi_db` moves by up to 12 dB.

  That is the model being re-published every 300 rows (P4 *One display model*, life
  cycle). The sessions recorded before 2026-09-19, when the live model still froze after
  its warm-up, show 0–1 grey level: a consistency check. Offline (fitted once per log)
  there is no drift by construction.

**M6 — Live vs offline**, the same recording (Phase 3 sessions, the same code), images
paired by time:
- **Nadir.** The first 0.75 m of slant range is ~98 % NaN live and 0 % offline.
- **Seabed PNG median, live − offline: −27 … +30 grey levels** (16, 2 and 4 images paired
  within 1 s over the three sessions; medians +6, −5.5, −3).
  - The older sessions show −9 … −81, but they mix in display-model fixes made since they
    were recorded (2026-09-18, 2026-09-28), so they are not a clean live/offline
    measurement.
- The *records* are identical live and offline (check 2): the difference lies in the
  pictures the GCS made live vs the pictures a replay would make.

**M2b — Row-level jitter of the square rows vs the speed window** (decision 6.3,
2026-10-16). Per full picture: the row-pose path / nominal span over consecutive 32-row
blocks. Its std within an image is short-scale scale noise; the largest |log ratio|
between adjacent blocks is the worst scale jump inside one picture.

| | 5 s | 2 s | 1 s |
|---|---|---|---|
| Real (3 sets), block std median / p95 | 0.09–0.18 / 0.40–0.44 | 0.10–0.17 / 0.32–0.44 | 0.09–0.14 / 0.27–0.38 |
| Real, worst jump p95 | 1.00–1.32 | 0.85–1.47 | 0.68–0.99 |
| Sim with truth poses (6 sets), worst jump p95 | 0.22–2.40 | 0.22–0.78 | 0.16–0.68 |
| Sim Phase 3 (constant speed) | ≈ 0.001 | ≈ 0.001 | ≈ 0.001 |

Reading:
- **No window adds jitter.** In simulation, where the pose is the truth, shorter windows
  lower the worst jump (e.g. 0.82 → 0.32 → 0.20 on 2026-09-28-10_27_47).
- On real data the block noise is dominated by the pose track itself: the median is
  about the same at every window, and p95 improves at 1 s.
- **1 s is "measured not worse", not validated.** The real reference is the autopilot's
  EKF pose, not an independent truth.
- The shares of pings repeated or skipped are set by speed against bin pitch, not by the
  window.

**M7 — The two detector inputs on the same data** (decision 6.4, 2026-10-16).

| | `identity` (display PNG) | `fixed_gray_v1` |
|---|---|---|
| Live − replay of the same recording, seabed median grey per paired image | −81 … +30 (Phase 3 sessions: −27 … +30) | **−16 … +14, medians 0 to ±4** |
| Real beach sets, seabed median grey | 31–70 (set to set) | 163–172 |
| Sim, post-2026-09-18 sessions at beach altitude | 43–84 | 165–193 |
| Sim Phase 3 (4 m world) | 112–114 | 210–212 |

Reading:
- The fixed mapping removes the display-model offset between live and replay. The
  residual spread is that the paired images do not hold exactly the same pings (their
  poses differ), plus the live nadir blank.
- It does **not** remove the simulator's level offset (M5). A simulated seabed reads up
  to ~20 grey levels (≈ 4 dB) brighter than the real one, more at other altitudes. That
  is an acoustic-model difference, not a representation one.

**What still cannot be measured:** the effect of any of this on *detection performance*.
The dummy detector cannot see it. Every "matters for detection" below is argued from
geometry and literature, and says so.

## 6. Decisions and what was implemented (2026-10-16)

The owner's decisions on the Phase 4 recommendation, the evidence behind each, and what
the code does now. Labels: **[published]** = a source read in full (REFERENCES);
**[measured]** = this project's data (§5); **[engineering]** = a choice made here;
**[open]** = an experiment not done.

| # | Proposal | Decision | Implemented | Where |
|---|---|---|---|---|
| 6.1 | Preserve the original images | Yes: the baseline | Yes — SSS image generation unchanged; detector input built separately; `identity` = the old input | Perception `core/preprocess.py` |
| 6.2 | Turn flags, not row exclusion | Yes | Yes — per row `turn_radius_m`, `turn_inner_side`, `backscan_from_m`; per image counts; per detection `turn_backscan`; no row or detection removed | Perception `core/pipeline.py`, `geometry.py` |
| 6.3 | GCS speed window 5 s → 2 s; 1 s after a jitter check | 2 s now | Yes — 2 s, one config value; jitter measured (M2b), 1 s **not validated** | SSS `config`, `core/along_track.py` |
| 6.4 | Stable detector-input grayscale mapping | Yes, priority | Yes — `fixed_gray_v1` from the raw dB, fitted on real data, default; `identity` one switch away | Perception `core/preprocess.py`, `core/data/fixed_gray_v1.json`, `tools/fit_detector_input.py` |
| 6.5 | 0.75 m near-range mask | Switchable, compare both | Yes — `near_range_mask_m` (0.0 default, 0.75), independent of the mapping | Perception `core/preprocess.py` |
| 6.6 | Roll-related radiometric correction | Only if established and validated | **Not implemented**: no validated method for this geometry; disabled behind `roll_correction` (raises); roll kept in records | Perception `core/preprocess.py` |
| 6.7 | Geometric correction, despeckling, histogram equalisation | **Not yet** (was "No") | Not added; named as deferred options that raise; originals kept for later comparison | Perception `core/preprocess.py` |

**6.1 — Preserve the original images.**
- **[published]** SSS processing chains differ from paper to paper:
  - Sethuraman et al. feed a vendor waterfall with a dataset-wide normalisation only
    [R12 §3.6];
  - Zhang et al. add a residual radiometric correction and interpolated slant-range
    correction [R19 §3.3];
  - Ye et al. a per-image Retinex correction [R20 §4].
  No single pipeline is established beyond the textbook corrections (TVG, angle-varying
  gain, slant-range correction [R1 §3.1, §4.4.2]).
- **Implemented:**
  - SSS's pictures, display PNGs and raw dB are unchanged (the SSS golden test still
    proves it);
  - Perception's `Preprocessor.run` never modifies its inputs (tested);
  - the detector input is a separate `DetectorInput.gray`, and `identity` reproduces
    the old input exactly;
  - `tools/preprocessing_study.py` compares the two inputs (M7);
  - the dataset writer keeps the display PNG beside the training image (`display/`).

**6.2 — Turn flags instead of row exclusion.**
- **[published]** On the inner side of a turn, a seabed point at lateral distance `y`
  moves along-track at `v − ω·y`. Beyond `r* = v/|ω|` it is scanned backwards [R14 §I.A].
- **[measured]** That covers 58–74 % of real pings (M1), so exclusion would discard most
  of the data.
- **Implemented**, in the records (`docs/topics.md` §4), with no row and no detection
  ever removed:
  - per row: the yaw rate averaged over `turn_window_s`, `r*`, the inner side, and the
    distance beyond which the inner side is backscanned (`r* < range`);
  - per image: the row counts with `r* < R` and `r* < R/2`;
  - per detection: `turn_radius_m`, and `turn_backscan` = on the inner side and beyond
    `r*`.
- **Inputs reused:** the IMU yaw rate (Q8), or the row-yaw derivative for older
  sessions, and the GCS's row speed.
- **[engineering]** `turn_window_s = 1.0` s, as in M1; not validated against detection
  outcomes. The `r*` criterion itself is geometry, not a tuned threshold.

**6.3 — GCS speed window.**
- **Implemented:** `along_track.window_s = 2.0` s (was 5.0) in `config/default.yaml` and
  `AlongTrackConfig`. The waterfall and the imager both take it from the config through
  `AlongTrackEstimator.from_config` (it was written in five places); tested.
- **[measured]** M2 (per-image scale p5–p95 0.88–1.10 → 0.94–1.07 real) and M2b (no
  added row jitter).
- **[open]** 1 s is measured "not worse" on the EKF pose and better on simulated truth.
  It is **not validated on real data**: that needs an independent position reference
  (RTK or the USBL), next step.
- The SSS golden test pins the 5 s window it was captured with, so it keeps proving the
  rest of the imaging code unchanged.

**6.4 — A stable detector-input grayscale mapping.** **Implemented:** the `fixed_gray`
preprocessor, the default.
- **The mapping:**
  - `gray = round(255·clip((db − g − R(s) − lo)/(hi − lo), 0, 1))`, NaN → 0;
  - it works from the raw dB (the 32FC1 on the wire = the session npz), never from the
    display PNG;
  - `g` is the reported receive-gain step (the GCS's measured
    `20·log10(analog_gain/74.55)`);
  - `R(s)` is **one fixed empirical range-reference curve** (median seabed level per
    log-slant-range bin, both sides pooled, held flat outside 0.8–36 m);
  - `[lo, hi] = [−31.15, +17.16] dB` are the 0.5 / 99.5 percentiles of `db − g − R(s)`
    over seabed samples.
- **The fit:**
  - **[engineering]** by `tools/fit_detector_input.py` on the real beach data only
    (CM-11): `bath1_2_3` and `object_boiteVsDistance`, 120 images, 2.4 M seabed samples;
  - stored with its provenance in `core/data/fixed_gray_v1.json`, whose SHA-256 every
    record carries;
  - it is fixed for training, validation, offline and live inference;
  - a refit takes a new name (`fixed_gray_v2`), never an overwrite.
- **Why this form:**
  - **[published]** dataset-wide, not per-image, normalisation is what the benchmark
    detector uses [R12 §3.6];
  - TVG-type corrections are standard, "fixed in hardware, or ad hoc" [R1 §4.4.2], but
    an assumed TVG law can over- or under-correct [R20 §2.1];
  - the device's own TVG is not identifiable (§1.4), so `R(s)` is fitted on the
    device's output and assumes no law. Nothing is compensated twice.
  - There is no per-image min/max stretch and no histogram equalisation: a pixel depends
    only on its own dB, its row gain and its column (tested).
- **[measured]** It removes the display-model offset between live and replay inputs (M7).
- **Limitations:**
  - it does not remove real-vs-simulated acoustic differences (M5, M7: sim up to ~4 dB
    brighter at beach altitude);
  - it is fitted at beach altitude (0.8–1.6 m) and pools both sides, so the
    altitude-dependent near-nadir response and port/starboard asymmetry are not
    compensated (see 6.6);
  - whether it helps a detector is **[open]** until a detector is trained and compared
    on both inputs.
- **Datasets:** dataset mode writes the detector input as the training image
  (`images/`), keeps the display PNG (`display/`) and the raw dB (`raw/`). The sidecar
  declares `intensity_mapping: log` with `log_range_db = hi − lo`, which the Aug
  Studio's `log` inversion matches exactly (REQUESTS R-3 resolved for this path; the
  recovered intensity is range-normalised).

**6.5 — Near-range mask, switchable.**
- **Implemented:** `near_range_mask_m` (default 0.0 = off; 0.75 = the GCS's live nadir
  blank), applied after either mapping and independent of it. Masked pixels are 0 and
  invalid; the column at exactly the distance is kept; tested at both settings and at
  the boundary.
- One setting is used live, offline and in dataset mode, and the record states it.
- **[open]** No labelled **real** data exists here (real datasets still get empty
  labels; simulated datasets are labelled from the simulator's ground truth since
  2026-10-16, DESIGN §7), so the number of real targets affected is unknown. The mask is
  not claimed to improve detection.

**6.6 — Roll-related radiometric correction: investigated, not implemented.**
- **What is already applied:**
  1. range / TVG: the device (unknown, §1.4), then `TL + A` in the display, or `R(s)` in
     `fixed_gray`;
  2. across-track beam pattern / grazing: the display model's per-side curve `A(r/h)`
     (an empirical angle-varying gain, refitted per mission), and nothing in
     `fixed_gray` beyond the pooled `R(s)`;
  3. ping to ping: the reported receive-gain step only;
  4. roll: nothing.
- **[published]**
  - angle-varying corrections from per-angle averages are textbook [R1 §4.4.2–4.4.3],
    but they assume the seabed type and topography are stable over the averaging
    [R20 §2.6];
  - the roll literature read here concerns **geometry** (no distortion [R14 §I.A];
    `H·tan φ` misplacement [R1 §10.4.2]; a per-ping correction of object size on a towed
    sonar [R7 §3.3]), not a radiometric correction for this geometry.
- **[measured]** 0.53–0.62 dB per degree of side difference on four logs from one beach
  site (M3). That is a slope for this boat, site and altitude band, **not a calibration
  constant**.
- **Decision:** no correction. `roll_correction=True` raises; roll is kept per row and per
  detection. Implementing one would need:
  - data where roll varies over the same seabed (repeat lines), or a simulated wave
    scenario that reproduces the slope;
  - then a held-out comparison.

**6.7 — Geometric correction, despeckling, histogram equalisation: not yet.**
- **[published]**
  - histogram equalisation over-enhances and amplifies noise [R20 §2.2]; global or local
    equalisation leaves background structure or creates discontinuities [R21 §III.A];
  - denoising effects depend on the method and data: wavelet shrinkage helped (+6 %
    detections), a Gaussian filter hurt [R21 §V];
  - Zhang et al. filter before shadow segmentation without an ablation [R19 §5.2];
  - despeckling "can lose valuable data" in target search [R1 §4.8.3];
  - geometric correction needs time-accurate attitude and fills gaps by interpolation
    [R8 §III.F.3; R19 §3.3], against [P5 §5].
- **Status:** not in the detector-input pipeline. Named as deferred options in
  `core/preprocess.py` (asking for one raises), with the original images preserved for a
  controlled comparison later.

**Next experiments:**
1. Train a detector on real labelled data under both inputs (`identity`, `fixed_gray`)
   and both masks, on a split by site [R12 §4]. This is what decides 6.4 and 6.5 on
   evidence.
2. Validate the 1 s speed window against RTK or USBL positions on a real run (6.3).
3. Run a simulated wave scenario to check the simulator's roll-to-brightness slope
   against M3 before any roll correction (6.6).
4. Settle the device TVG question with the vendor's receiver documentation or a
   calibrated target at known ranges (§1.4).
