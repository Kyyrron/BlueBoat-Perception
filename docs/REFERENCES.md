# References — BlueBoat-Perception

Every non-software choice in this module (the preprocessing study, motion effects, the
detector-input domain, the detector family, confidence calibration) traces to a source in
this file. The rules this file follows:

1. **A source is used only after it has been read in full.** A book counts as read when the
   chapters named in its status were read in full (`[partial: …]`). Sources that could not
   be read in full were **dropped** (owner's decision, 2026-10-15: "accessible sources
   only"). They are listed at the end with the reason, and nothing in this module rests on
   them.
2. **Pointers.** A citation reads `[R8 §III.F eq. 30]`: source, section, figure or
   equation. The same pointer is used in the docs and in code comments.
3. **Project documents are sources too** (`P#`) and are cited by section. They were read
   in the tree as of 2026-10-15.
4. **Nothing is cited from memory.** Every bibliographic field below was checked against
   the document itself or its publisher record on 2026-10-15 (R19–R22 on 2026-10-16).
5. **One paper using a method does not make it standard.** "Established" below means
   described as standard practice in a textbook read here (R1), not used in one study.

Status legend: `[full]` read in full · `[partial: …]` the named parts read in full ·
`[abstract]` abstract/record only · `[not accessed]`.

---

## Kept sources

### R1 — Blondel 2009 `[partial: §2.2.3, §2.5, Ch. 3, Ch. 4, §10.4–§10.5 read in full]`
Blondel, P. (2009). *The Handbook of Sidescan Sonar*. Springer–Praxis, Chichester.
ISBN 978-3-540-42641-7.

Read from the local copy `~/Downloads/research_paper/blondel2009.pdf`. The PDF text layer
is not extractable, so the pages were read as images. Book page = PDF page − 25 up to
p. 47, then −24 (Ch. 4), then −21 (Ch. 10).

| Used | Where applied |
|---|---|
| §2.2.3, p. 16: backscatter depends, in decreasing order, on sensor–target geometry (local incidence angle, slope), surface physics (roughness), intrinsic nature. p. 18: Lambert's law = backscatter linear in cos² of incidence | STUDY §1.4 (across-track normalisation), §4 option E |
| §2.5 eq. 2.10 `Δx = R·sin θh` (along-track resolution = min(distance travelled per reception interval, beam width on ground)); eq. 2.11 `Δy = cτ/2 · 1/cos β`; p. 31: pixel larger than sample spacing averages, smaller needs interpolation "likely to produce errors by creating new data or distorting existing features" | STUDY §1.5, §2 (scales), §4 option C risk |
| §3.1 p. 37: TVG of the form `40 log R + 2αR` compensates attenuation, "even if this gain is applied, the sonar signals might still need further processing"; at constant speed and altitude over flat terrain the two image axes correspond to ground dimensions, otherwise conversion must be done in processing | STUDY §1.4, §1.5 |
| §3.2.1 eq. 3.1 `r = ct/2`, eq. 3.2 `d = sqrt(r² − h²)` with a flat-ground assumption | STUDY §1.2; DESIGN §geometry |
| §3.4 p. 46: if missing data are masked or replaced, the interpolation must be stated | STUDY §1.6 |
| §4.2 p. 50 and §4.2.3 p. 56: locating echoes needs position and attitude (heave, roll, pitch, yaw) of the sonar platform; these "need to be explicitly recorded for the sonar platform" | STUDY §3; DESIGN (attitude in metadata, Q8) |
| §4.2.1 p. 51–52: GPS CA ≈ 15 m; DGPS/RTK < 10 m, often 1–5 m above water | STUDY §2.7 (position error) |
| §4.3.1 p. 57: pre-processing / processing / post-processing definitions | STUDY §1 structure |
| §4.3.3 p. 58–59: seafloor features "can be arbitrarily merged, distorted, or truncated if navigation and attitude are not correctly processed" | STUDY §2 |
| §4.3.4 p. 61–62: attitude records are noisy; smoothing (moving average, Kalman) and spline interpolation of attitude before use | STUDY §4 option D |
| §4.4.2 p. 63: across-track corrections = TVG (fixed in hardware, or ad hoc) | STUDY §1.4 |
| §4.4.3 p. 63–65: along-track striping (dropout lines, vehicle motion), lines shifted across-track when the altimeter loses the bottom; AVG from per-angle along-track averages | STUDY §1.4, §2.4 |
| §4.5.1 p. 65–66 eq. 4.3: slant-range correction assumes a flat seafloor; "some specific applications do not use slant range correction because they need to keep the information just below the sonar's track: fisheries and some target search surveys" | STUDY §1.2; DESIGN (detector input stays slant-domain) |
| §4.5.2 p. 66–67, fig. 4.15: anamorphosis = along-track correction for platform speed by replicating / sub-sampling lines to a 1:1 aspect ratio | STUDY §1.5 (the GCS "square" geometry is this) |
| §4.7.2 fig. 4.17 p. 69: in turns the inside-corner swath portions overlap, the outside-corner portions are spread and coverage is incomplete | STUDY §2.1 |
| §4.8.3 p. 73: despeckling "can lose valuable data" in target searches at the resolution limit | STUDY §4 (no despeckling) |
| §4.10 p. 79–81: unprocessed attitude variations "drastically change the shape of objects … or even separate them into what appears to be distinct pieces"; checklist fig. 4.25 (heading, speed, attitude; TVG/AVG; slant-range correction; anamorphosis); eq. 4.7 shadow height `h = ΔR·H/R` | STUDY §2, §4; DESIGN (record carries the acquisition / processing facts) |
| §10.4.1 p. 236–237: a high-backscatter band near nadir on all sidescan images, "no way to directly remove this artifact short of masking it"; AVG leaves non-systematic beam-pattern variations | STUDY §1.3, §4 option F |
| §10.4.2 p. 237–243: speed error stretches/compresses along-track (fig. 10.13); heave gives symmetric across-track distortion (p. 239); unprocessed roll gives asymmetric across-track distortion (100 m altitude: 10° → 17.6 m, 1° → 1.74 m, i.e. H·tan φ); pitch replicates or anticipates swath lines (fig. 10.17); yaw re-images features inside the turn from a different look angle and images others farther away outside it (fig. 10.18); attitude sensors can reach < 0.1° | STUDY §2.1–§2.6 |
| §10.4.4 p. 244–245: residual attitude spikes create local geometric distortions; inappropriate TVG/AVG show as darker/lighter swaths or patches | STUDY §2 |
| §10.5.1 p. 246: anamorphosis with an assumed constant speed elongates or compresses structures | STUDY §1.5 |

### R6 — Burguera & Oliver 2016 `[full]`
Burguera, A., Oliver, G. (2016). High-resolution underwater mapping using side-scan sonar.
*PLoS ONE* 11(1): e0146396. doi:10.1371/journal.pone.0146396.

| Used | Where applied |
|---|---|
| eq. 1 grazing angle `θs = asin((h − hp)/rs)`; eq. 2 `rg = sqrt(rs² − (h−hp)²)`; flat-floor error model eq. 3–7, table 2 (negligible errors for object heights ±16 cm at short slant range, ±66–71 cm at long, for their 5 m / 12 cm system) | STUDY §1.2 (flat-floor error at our altitudes); DESIGN (grazing angle in records) |
| eq. 8, eq. 12: Lambertian intensity model `I = K·φ(p)·R·cos β`; reflectivity `R = I/(φ·cos β)` with a beam-pattern model (eq. 9) | STUDY §4 option E |
| "Blind zone removal" (eq. 14): bins inside ±r_FBR discarded as uninformative | STUDY §1.3 (contrast with the GCS keeping the water column) |
| "Localization": roll and pitch assumed zero for the AUV ("reasonable … where the robot control is designed to keep these angles to that value") | STUDY §3 (not true for a surface vessel) |
| eq. 28–30: probability that a map cell was observed, `P(q) = 1 − Π(1 − Pm(q))`; fig. 7: cells unobserved because of motion between swaths; conclusion: the probabilistic layer tells which areas deserve revisits | DESIGN §5.3 (per-image footprint + per-row geometry is what Belief needs to compute coverage) |
| fig. 22: gaps in turns, filled by geometric extrapolation | STUDY §2.1 |

### R7 — Kim & Ryou 2020 `[full]`
Kim, Y., Ryou, J. (2020). A Study of Sonar Image Stabilization of Unmanned Surface Vehicle
Based on Motion Sensor for Inspection of Underwater Infrastructure. *Remote Sensing*
12(21): 3481. doi:10.3390/rs12213481.

mdpi.com refused direct fetches; the full text came from the publisher file server
(mdpi-res.com).

| Used | Where applied |
|---|---|
| Abstract, §1: a small USV "is more affected by the waves and swells than a common boat", which distorts sonar data | STUDY §2 (motivation) |
| §2.2: sonar **towed** under the USV specifically to reduce roll and pitch coupling; §3.2: roll from a motion sensor (OS-5000, 1° inclination accuracy, ≤ 40 Hz), correction applied **per ping** as a shift of `distance·tan(Δroll)` (±40.2 cm at 1.5 m for 15°) | STUDY §2.2, §4 option D |
| §3.3: "the change amount of the roll more greatly affects the distortion … than the change of the pitch" | STUDY §2.2 |
| §4.2: object-size error reduced 21.58 % → 5.84 % (1250 kHz, roll −6.5° to 6.4°) and 18.91 % → 3.91 % (2 MHz) after roll correction | STUDY §2.2 |

**Applicability caveat (STUDY §2.2).** Their geometry is a sonar rotated to look at a
**vertical quay wall** 1–2 m away. There, roll sweeps the beam across the wall face, which
is a geometric shift. For a down-looking seabed survey with time-of-flight ranging, R14 and
R8 treat roll as mainly radiometric. The paper does not show that its roll correction
transfers to our geometry, and it does report object-dimension effects (your prompt's
claim) only for the wall case.

### R8 — Lei et al. 2026 `[full]`
Lei, C., Franchi, V., Rajani, H., Gracias, N., Garcia, R., Wang, H. (2026). Geometric
Correction of Side-Scan Sonar Images with Image-Consistent Attitude Refinement.
arXiv:2604.19901v1 [physics.ao-ph].

| Used | Where applied |
|---|---|
| §I: the raw waterfall stacks pings and is "highly sensitive to time-varying platform motion"; navigation records suffer "high-frequency noise, drift, timing delay, and filtering artifacts" and miss ping-scale perturbations | STUDY §2, §3 |
| §III.A.1: roll "mainly modulates backscatter intensity via incidence-angle variation" with a minor along-track contribution; altitude from bottom tracking kept fixed; position/speed fluctuations produce the same symmetric distortion as pitch | STUDY §2.2–§2.5 |
| §III.A.2, fig. 1: yaw is antisymmetric (advances one side, delays the other); pitch is symmetric across sides | STUDY §2.1, §2.3 |
| §III.F eq. 27–30: effective altitude `h/cos θ`; ground range `sqrt(ℓ² − a²)`; **pitch along-track displacement `D_at = h·tan θ`** (eq. 30); yaw rotation (eq. 32) | STUDY §2.3; DESIGN (convention-2 geometry is the θ = 0 case) |
| §III.F.3: grid holes filled by normalized convolution, i.e. invented values | STUDY §4 option D risk |
| Table I: datasets are 900 kHz / 500 kHz towed or AUV systems at ~10 % altitude-to-range, not a surface vessel | STUDY §4 (transfer caveat) |
| §V: thresholds empirically chosen; compression not modelled | STUDY §4 |

### R9 — Al-Rawi et al. 2017 `[full]`
Al-Rawi, M., Elmgren, F., Frasheri, M., Çürüklü, B., Yuan, X., Martínez-Ortega, J.-F.,
Bastos, J., Rodriguez, J., Pinto, M. (2017). Algorithms for the Detection of First Bottom
Returns and Objects in the Water Column in Sidescan Sonar Images. *OCEANS 2017 – Aberdeen*.
doi:10.1109/OCEANSE.2017.8084587.

| Used | Where applied |
|---|---|
| §I: the FBR is the sensor's primary altitude, needed for "across-track correction filters, speed correction, ground-range correction"; the nadir edge is not straight because of vehicle motion | STUDY §1.2 |
| §II–III: spline / moving-average FBR detectors; objects in the water column spike the along-track FBR and are removed by median filtering it; 22/25 images succeeded | STUDY §1.2 (our tracker is a threshold-and-persistence one, R9's §I family) |

### R10 — Redmon et al. 2016 `[full]`
Redmon, J., Divvala, S., Girshick, R., Farhadi, A. (2016). You Only Look Once: Unified,
Real-Time Object Detection. *CVPR 2016*, 779–788. doi:10.1109/CVPR.2016.91.

| Used | Where applied |
|---|---|
| fig. 1: the input is resized (448×448), one network pass, then thresholding / NMS | DESIGN §detector (input adapter: our tiles are 256 × 1200–2402, so a resize would squash them and tiling is needed later) |
| §2, eq. 1: the box confidence is defined as `Pr(Object)·IOU`, and the class score as `Pr(Class)·IOU`. **This is not a calibrated probability by construction** | DESIGN §calibration (raw score ≠ probability; calibration hook) |
| §2.2: box x, y, w, h normalised to the image | DESIGN §dataset (YOLO label format) |
| §2.4, §4.2: weaker on small objects and groups; localization is the main error source | DESIGN §covariance (pixel uncertainty term); TODO |

### R11 — Guo et al. 2017 `[full; the main paper, not the supplement]`
Guo, C., Pleiss, G., Sun, Y., Weinberger, K.Q. (2017). On Calibration of Modern Neural
Networks. *Proc. 34th ICML*, PMLR 70:1321–1330.

| Used | Where applied |
|---|---|
| §2 eq. 1 (definition of calibration), reliability diagrams, eq. 3 ECE, eq. 5 MCE | DESIGN §calibration (evaluation metrics for the future calibrator) |
| §4: every method needs a **hold-out validation set from the same distribution** as test | DESIGN §calibration (calibrate per domain: sim vs real are different distributions, CM-11) |
| §4.1 Platt scaling, histogram binning, isotonic; §4.2 eq. 9 temperature scaling (one scalar, preserves the argmax); §5: temperature scaling is the most effective on vision tasks | DESIGN §calibration (interface: raw score/logit in → calibrated probability out; identity now) |
| Scope: multiclass **classification**. Applying it to detector scores is an extension that must be validated, not assumed | DESIGN §calibration caveat |

### R12 — Sethuraman et al. 2024 `[full; arXiv v2]`
Sethuraman, A.V., Sheppard, A., Bagoren, O., Pinnow, C., Anderson, J., Havens, T.C.,
Skinner, K.A. (2024). Machine Learning for Shipwreck Segmentation from Side Scan Sonar
Imagery: Dataset and Benchmark. arXiv:2401.14546v2. The official version is in *IJRR*,
doi:10.1177/02783649241266853, per the arXiv acknowledgements.

| Used | Where applied |
|---|---|
| §2.1: beam pattern → intensity varies along the horizontal axis; resolution depends on altitude, range and grazing angle, "all of which can change within a single image" | STUDY §1.4 |
| §3.4 eq. 1 `Rx = R·sin θh`; eq. 2 `Ry = c/(2·BW)` | STUDY §2 (scales; agrees with R1 eq. 2.10) |
| §3.6: detector input = vendor-exported **slant-range waterfall** with vendor TVG + normalisation, **no slant-range correction, no GPS processing**; square crops with a 100 px stride (eq. 3); dataset-wide mean/std normalisation only | DESIGN (waterfall, slant-domain input is established practice; CM-10) ; STUDY §4 |
| §4, §4.2: train/test split **by site**, so overlapping imagery never crosses the split | DESIGN §dataset (split by session/site, never by image: our tiles overlap 50 %) |
| §4.2.2: shallow surveys are vulnerable to wave-induced drift (fig. 6d); objects in the nadir are disrupted | STUDY §2 |

This is a segmentation dataset on an AUV, not a detection dataset on a USV. That limits
what it can support.

### R14 — Cobra, Oppenheim & Jaffe 1992 `[full]`
Cobra, D.T., Oppenheim, A.V., Jaffe, J.S. (1992). Geometric distortions in side-scan sonar
images: a procedure for their estimation and correction. *IEEE J. Oceanic Eng.*
17(3):252–268. doi:10.1109/48.153442.

Read from the local copy `~/Downloads/research_paper/cobra1992.pdf`.

| Used | Where applied |
|---|---|
| §I.A, fig. 2: pitch and yaw make the beams scan ahead or back (both sides together for pitch, alternately for yaw); **roll does not produce geometric distortion** ("the beam rotates within its own plane") and only causes intensity distortion near nadir and at maximum range; severe pitch/yaw **backscanning** puts objects in triplicate | STUDY §2.1–§2.3 |
| §I.A, fig. 3: speed and course variations change the aspect ratio; stretched where sampling is denser, skewed where lines shift across-track | STUDY §2.5 |
| §II.D eq. 3b: the along-track sampling displacement grows **linearly with range** in the yaw increment, `Δy' = Δy0' + m·(cTs/2)·Δθ` | STUDY §2.1 |
| §IV.B eq. 13b, 14: the scan-line offset along-track = `z·tan(pitch)` | STUDY §2.3 (agrees with R8 eq. 30) |
| §V–VI: correction is more reliable locally than absolutely; aliasing where along-track sampling falls below Nyquist cannot be undone; pitch produces less distortion than equal yaw "because the altitude of the towfish above the seabed is typically only a fraction of the maximum range" | STUDY §2.1, §2.3, §4 option C/D risk |
| §VII: high-frequency (≥ 100 kHz) small sonars without attitude sensors are the most susceptible | STUDY §2 |

### R15 — Hughes Clarke 2003 `[full]`
Hughes Clarke, J.E. (2003). Dynamic Motion Residuals in Swath Sonar Data: Ironing out the
Creases. *International Hydrographic Review* 4(1):6–23.

Read from journals.lib.unb.ca. The subject is **multibeam**; only the integration
principles transfer.

| Used | Where applied |
|---|---|
| "Manifestation …": when the ping period is short against the wave period (≥ 10 Hz vs ~0.1 Hz), the motion error is near-constant within a ping | STUDY §3 |
| §(B): a motion time delay δt gives an error = rate × δt (sinusoid: max rate 2πA/T; 10 s, ±3° → ~0.019°/ms, so 5 ms gives 0.1°); low-rate attitude streams must be **interpolated**, not taken as the latest sample | STUDY §3 (the GCS attaches the latest heading/attitude at reception) |
| §(D,E): lever-arm errors induce heave `IH_err = −dX sin p + dY sin r cos p + dZ(1 − cos r cos p)` | STUDY §2.2 (our 0.285 m Y lever arm) |
| "Dual Head Complications": lever arms and alignment can differ per head, so the error may affect one side only | STUDY §2 |

### R19 — Zhang et al. 2024 `[full; main text]`
Zhang, Z., Wu, R., Li, D., Lin, M., Xiao, S., Lin, R. (2024). Image stitching and target
perception for Autonomous Underwater Vehicle-collected side-scan sonar images.
*Frontiers in Marine Science* 11:1418113. doi:10.3389/fmars.2024.1418113 (publisher
record and PDF checked 2026-10-16; open access, CC BY). Read 2026-10-16; the segmentation
appendix was not needed.

| Used | Where applied |
|---|---|
| §3.3: "modern side-scan sonar equipment incorporates its own Time-Variant Gain (TVG) adjustment, it still necessitates some degree of radiometric correction due to discrepancies between TVG compensation and actual attenuation"; they use a "Mean Amplitude Gain Compensation" (**no formula given**), then slant-range correction with **linear interpolation** of the resulting gaps | STUDY §1.4, §6.4 (a residual range correction after the device is common; the paper does not validate one for detection) |
| §5.1–5.2, §6: target presence by an EfficientNet classifier (86 % accuracy on a public test set), shadow segmentation after non-local-means filtering, K-means, RSF; ES1000, 900 kHz, on an AUV; **no ablation of the preprocessing on detection** | STUDY §6.7 (despeckling-type filters appear in pipelines without evidence of their effect on detection) |

### R20 — Ye et al. 2019 `[full]`
Ye, X., Yang, H., Li, C., Jia, Y., Li, P. (2019). A Gray Scale Correction Method for
Side-Scan Sonar Images Based on Retinex. *Remote Sensing* 11(11):1281.
doi:10.3390/rs11111281 (checked against the PDF header 2026-10-16; open access).

| Used | Where applied |
|---|---|
| §2.1: TVG is "usually implemented using hardware"; its parameters are hard to determine from images alone, "unrealistic gain parameters cause secondary gray distortion", and different images need different parameters | STUDY §1.4, §6.4 (do not apply an assumed TVG law on top of the device's) |
| §2.2: histogram equalisation "often increases image noise", blurs weak edges and over-enhances regions with large histogram peaks | STUDY §6.7 |
| §2.4, §2.6: function-fitting corrections (per-column mean curves; MIRA; Al-Rawi et al.) and beam-pattern correction by the average energy per angle, which "needs to consider the change in seabed topography and seabed sediments" | STUDY §6.6 (the established across-track corrections and their assumption) |
| §4–5: their Retinex correction divides each image by its own smoothed version plus a constant (`a ≈ 15`, `A = 140`, empirical; "different batches may require fine-tuning"); it is evaluated with **image-quality indices only** (PSNR, entropy, standard deviation, average gradient), not detection | STUDY §6.4 (a per-image adaptive normalisation is the opposite of the stationarity 6.4 needs), §6.7 |

### R21 — Neretti, Intrator & Huynh `[full]`
Neretti, N., Intrator, N., Huynh, Q. *Target detection in side-scan sonar images: expert
fusion reduces false alarms.* Manuscript, 9 pp., undated (its references reach 1995;
supported by ONR code 311). https://www.math.tau.ac.il/~nin/papers/SSSfinal.pdf (read
2026-10-16). **No venue could be verified** (publisher databases not searchable here), so
it is cited as a manuscript.

| Used | Where applied |
|---|---|
| §III.A, fig. 2: global histogram equalisation leaves the background structure; a local (cell-wise) equalisation creates discontinuities; a local standardisation (mean/std over a neighbourhood) removes it | STUDY §6.7 (normalisation choices change the image; none is neutral) |
| §III.B, §V: wavelet shrinkage raised correct detections by ~6 % (two targets) at a low false-alarm rate; a Gaussian filter increased false alarms; DoG results too poor to report — on two NSWC 8-bit image sets (60 and 80 images) | STUDY §6.7 (denoising effects depend on the method and the data: "not yet", not "no") |

### R22 — Chang, Hsu & Tsai 2010 `[not accessed: bibliographic record only]`
Chang, Y.-C., Hsu, S.-K., Tsai, C.-H. (2010). Sidescan sonar image processing: correcting
brightness variation and patching gaps. *Journal of Marine Science and Technology
(Taiwan)* 18(6). doi:10.51400/2709-6998.1935 (journal record checked 2026-10-16; the page
range differs between indexes, so none is given). The PDF server answers every request
here with a challenge page (HTTP 403). **Not used for any claim.** Ye et al. (R20 §2.6)
describe a per-angle energy normalisation by "Chang and colleagues [6,7,9]"; whether
those references include this paper was not checked, so that description is R20's and is
cited as R20's only. Provide the PDF if you want R22 itself used.

**Tiled detection** (added 2026-10-23, read for the tiled YOLO adapter): R13, R23–R26,
S1, S2.

### R13 — Akyon, Altinuc & Temizel 2022 `[full; arXiv v5]`
Akyon, F.C., Altinuc, S.O., Temizel, A. (2022). Slicing Aided Hyper Inference and
Fine-tuning for Small Object Detection. arXiv:2202.06934v5 [cs.CV], 24 Oct 2022 (the
paper states it was published at IEEE ICIP 2022 via its GitHub; not checked here).
Read 2026-10-23 (dropped on 2026-10-15 as not accessed; now read in full).

| Used | Where applied |
|---|---|
| §3, fig. 2: slicing in fine-tuning (patches extracted from the training images, together with the originals) and in inference (overlapping patches, independent forward passes) | `core/tiling.py` (design); DESIGN §4.4 |
| §3: "as the patch sizes decrease, larger objects may not fit within a slice and the intersecting areas" | DESIGN §4.10 (wide highlight + shadow boxes; the merge comparison) |
| §3: the overlapping predictions "are merged back into to original size using NMS ... boxes having higher IoU ratios than a predefined matching threshold Tm are matched"; §4: Tm = 0.5; **no merging (NMM) rule is described in the paper** | `core/merge.py` `merge_nms` (the paper's rule, threshold 0.5); the greedy merge is attributed to the SAHI *software* (S2), `[engineering]` |
| §4, tables 1–2: 25 % patch overlap at inference raised small/medium AP and slightly lowered large-object AP (false positives from slices matching large boxes) on VisDrone / xView | DESIGN §4.10 (why the overlap is a compared parameter, stride 128 vs 64) |
| Scope: aerial optical imagery, FCOS / VFNet / TOOD, patches **resized**. Nothing here is sonar; our tiles are never resized | DESIGN §4.10 caveat |

### R23 — Yu et al. 2021 `[full]`
Yu, Y., Zhao, J., Gong, Q., Huang, C., Zheng, G., Ma, J. (2021). Real-Time Underwater
Maritime Object Detection in Side-Scan Sonar Images Based on Transformer-YOLOv5.
*Remote Sensing* 13(18): 3555. doi:10.3390/rs13183555. Read from the publisher file server
(mdpi-res.com) 2026-10-23.

| Used | Where applied |
|---|---|
| §2.1.2, fig. 2: a sample spans the full swath width, height `d` = the detector's input (320) | DESIGN §4.10 (our picture height = the tile size, 256) |
| §2.1.2 eq. 2, fig. 4: across-track **down-sampling** to square pixels (0.1 × 0.5 m → 0.1 × 0.1 m); §3.3.1: without it the shipwreck was missed | DESIGN §4.10 (the GCS's square rows already give square pixels; we never resize across track) |
| §2.1.2, fig. 5; §3.3.1: `d × d` patches by a sliding window across track, adjacent patches sharing 40 % "to prevent the target from being divided into two parts" | `core/tiling.py` (stride ≤ half the tile) |
| §3.3.2: the detections of all patches go through NMS in the picture's coordinates | `core/merge.py` `merge_nms` |
| §3.1.1: dataset split 6:2:2 **by image** after mixing; augmentation includes rotation, zoom, translation | **Not followed**: we split by world (R12 §4, CM-11) and switch those augmentations off (`tools/train_yolo.py`) |
| Scope: towed / hull systems at 100–150 kHz, shipwrecks and containers, 286 training images | caveat |

### R24 — Yang et al. 2025 (SS-YOLO) `[full]`
Yang, N., Li, G., Wang, S., Wei, Z., Ren, H., Zhang, X., Pei, Y. (2025). SS-YOLO: A
Lightweight Deep Learning Model Focused on Side-Scan Sonar Target Detection. *J. Mar.
Sci. Eng.* 13(1): 66. doi:10.3390/jmse13010066. Read 2026-10-23 (mdpi-res.com).

| Used | Where applied |
|---|---|
| §2.1: part of the dataset collected from a USV (SS900F); the processed images were cropped to 640 × 640 around each target | DESIGN §4.10 (crop practice in USV SSS detection: target-centred crops, not a tiling rule) |
| §2.2: random stratified split 80/15/5 **by image**; §4.2 table 2: trained from scratch at 960 × 960 | **Not followed** (split by world); context only |
| §5: deployment on the USV is **future work** — the model was not run on the vessel | Citation discrepancy below |

### R25 — Ünel, Özkalaycı & Çığla 2019 `[full]`
Ünel, F.Ö., Özkalaycı, B.O., Çığla, C. (2019). The Power of Tiling for Small Object
Detection. *CVPR Workshops 2019* (UAVision). openaccess.thecvf.com, read 2026-10-23.

| Used | Where applied |
|---|---|
| §4.2: tiles used in **training** too, ground truth "arranged accordingly without change of object size"; overlap "to preserve the objects along the tile boundaries" (25 %); "the risk of loosing larger object annotations also increases" as tiles shrink | `tools/make_tile_dataset.py` (native-size tiles, rule B + (i)) |
| §4.3: tile detections merged: when the intersection of duplicates exceeds 25 % the higher score is kept | DESIGN §4.10 (an intersection-based rule, like `greedy_nmm`'s IoS, between NMS and merging) |
| Scope: MAV optical imagery, Pelee / SSD | caveat |

### R26 — Xu, Ling, Xie, Zhang & Folkesson 2023 `[full; arXiv v1]`
Xu, W., Ling, L., Xie, Y., Zhang, J., Folkesson, J. (2023). Evaluation of a Canonical
Image Representation for Sidescan Sonar. arXiv:2304.09243v1 [cs.CV]. (The prompt's
short title, "canonical sidescan representation", is this paper.)

| Used | Where applied |
|---|---|
| §III.2, Algorithm 1: slant-range correction to equally spaced ground-range bins of an arbitrary resolution, starting at a 30° incidence angle, which removes the nadir zone | TODO (ground-range resampling: a later arm, not now) |
| §IV.C, fig. 1: after the transformation a feature's width is consistent across range (112 → 78 px and 153 → 77 px for the same feature at two ranges) | TODO (what resampling would change: object size independent of range) |
| §IV: evaluated on 60 patch pairs (patch similarity, SIFT / ORB matching), HUGIN AUV, **not on detection** | TODO caveat: an effect on a detector is untested |

### S1 — Ultralytics (software documentation and source), v8.4.184
Read 2026-10-23: docs.ultralytics.com `models/yolo11`, `models/yolo26`, `modes/train`,
`usage/cfg`, `datasets/detect`, `yolov5/tutorials/tips_for_best_training_results`; source
`ultralytics/cfg/default.yaml`, `cfg/datasets/coco8-grayscale.yaml`, `data/augment.py`,
`models/yolo/detect/train.py`, `nn/tasks.py` (`load`), installed 8.4.184. Software
behaviour, cited for what the code does, not as research evidence.

| Used | Where applied |
|---|---|
| `default.yaml`: augmentation defaults (`hsv_s 0.7`, `hsv_v 0.4`, `translate 0.1`, `scale 0.5`, `fliplr 0.5`, `mosaic 1.0`), `conf` 0.25 predict / 0.001 val, `iou 0.7`, `max_det 300`, `seed`, `deterministic` | `tools/train_yolo.py` (every value set explicitly); adapter defaults |
| `augment.py` `RandomPerspective.box_candidates`: after a crop or shift, a box keeping < 10 % of its area (or < 2 px) is dropped and the image kept | rule B + (i) (`make_tile_dataset.py`) |
| Training tips page: "about 0-10 % background images to help reduce FPs"; "All instances of all classes in all images must be labeled. Partial labeling will not work." | `--empty-fraction 0.10`; the tension with (i) noted in DESIGN §7.1 |
| `coco8-grayscale.yaml`: `channels: 1`; `detect/train.py`: the model is built with the dataset's channels; `tasks.py` `load`: a first convolution with another input-channel count is "filled from the source filters, summed for a single-channel model" | `data_gray1.yaml`; the 1- vs 3-channel arm |
| `models/yolo26`: detection models `yolo26n…x.pt` at 640; prediction "default[s] to the one-to-many head", which "requir[es] NMS post-processing"; one-to-one (NMS-free) with `nms=False`; YOLO26s 48.6 mAP50-95 (COCO). `models/yolo11`: YOLO11s 640, 47.0 mAP50-95, 9.4 M parameters | adapter (in-tile NMS for both families); README |

### S2 — SAHI (software), commit b5a153e
`github.com/obss/sahi`, `sahi/postprocess/combine.py` (`GreedyNMMPostprocess`,
`_apply_merge`), `_numpy_backend.py` (`greedy_nmm_from_matrix`), `utils.py`
(`calculate_bbox_ios`, `get_merged_score` = max, `get_merged_bbox` = union). Read
2026-10-23.

| Used | Where applied |
|---|---|
| Greedy NMM: by descending score, each kept box collects the unassigned lower boxes whose metric (IoU or IoS) with it is ≥ the threshold, then absorbs them one by one, re-checking the match against the growing box; merged box = union, score = max | `core/merge.py` `merge_greedy_nmm` (reimplemented in numpy-free Python, no dependency) |

---

## Project documents (cited by section)

| # | Document | Sections used |
|---|---|---|
| P1 | `project_synthesis.md` (superproject) | §1 (platform: 0.5° azimuth beam, 50° vertical, 20 Hz), §3.2, §5.2 (data structures; detector input ⚠ boat-relative waterfall; detector output fields), §7.6 ⚠ (synthetic-trained detector only for E7), §8.2 ⚠ (calibration), §8.5 (reserved settings), §8.6 ⚠ (offline iteration) |
| P2 | root `.claude/CLAUDE.md` | §1.1, §3, §4.1, §4.3, §4.6, CM-1 … CM-16, D7 |
| P3 | `BlueBoat-SSS/.claude/CLAUDE.md` | ROS 2 interface, NC #1–#16, Data produced |
| P4 | `BlueBoat-SSS/blueboat_sss/blueboat_gcs/CLAUDE.md` | ROS 2 interface of the GCS, *One display model*, *Seabed images*, *Row geometry* |
| P5 | `BlueBoat-SSS/blueboat_sss/blueboat_gcs/docs/HANDOVER.md` | §3.1bis (seabed imaging, analysis topic), §3.2 (detections slot), §3.3, §5 ("Should the AI see interpolated images? … no") |
| P6 | `BlueBoat-SSS/blueboat_sss/blueboat_gcs/docs/SCIENTIFIC_BACKGROUND.md` | §1, §3 (no despeckling in the feed), §5 (one mission-wide invertible window for the feed), §7 (AI picture geometry), §8 (measurements: 60–70 dB/decade falloff, residual spread), §9 (display model). **Its literature citations are not inherited.** Sources it cites that were not read here (Lurton, Capus, Cervenka, Chavez 1986, …) are not used through it |
| P7 | `BlueBoat-SSS-Sim/blueboat_sss_sim/docs/sonar_model.md` | §1, §3 (vertical beam pattern; TVG fraction c = 0.30), §4 (attitude coupling), §6 (encoding), §7 (assumptions; A5 no intra-ping motion) |
| P8 | `BlueBoat-SSS-Sim/blueboat_sss_sim/docs/sea_state_validation.md` | "Against the real boat" table (field roll/pitch/yaw-rate statistics, ATTITUDE 25 Hz) |
| P9 | `BlueBoat-SSS-Sim/blueboat_sss_sim/docs/topics.md` | §4 (pwr_results encoding), §5 (ground-truth contacts) |
| P10 | `SSS-Dataset-Aug-Studio/sss_aug_studio/datasets/yolo.py`, `core/meta.py`; superproject skill `file-interfaces` | dataset layout the Aug Studio reads |

---

## Dropped sources (not readable in full here; nothing rests on them)

| # | Source | Status | Why dropped |
|---|---|---|---|
| R2 | Lurton, X. (2010). *An Introduction to Underwater Acoustics*, 2nd ed. Springer. | [not accessed] | No copy available; book behind a paywall. |
| R3 | Cervenka, P., de Moustier, C. (1993). Sidescan sonar image processing techniques. *IEEE JOE* 18(2):108–122. doi:10.1109/48.219531 | [abstract] | IEEE paywall; the UNH record links only the DOI. |
| R4 | Capus, C.G., et al. (2008). Data correction for visualisation and classification of sidescan SONAR imagery. *IET RSN* 2(3):155–169. doi:10.1049/iet-rsn:20070032 (DOI as printed in R1 §10.7) | [not accessed] | IET paywall. |
| R5 | Chavez, P.S. Jr., et al. (2002). Processing, mosaicking and management of the Monterey Bay digital sidescan-sonar images. *Mar. Geol.* 181:305–315. doi:10.1016/S0025-3227(01)00273-0 | [abstract] | Elsevier paywall; the USGS record has no PDF. |
| ~~R13~~ | SAHI — **kept since 2026-10-23** (read in full, above). | | |
| — | Sapkota, R., Cheppally, R.H., Sharda, A., Karkee, M. (2025–26). YOLO26: Key Architectural Enhancements and Performance Benchmarking for Real-Time Object Detection. arXiv:2509.25164v5 | [accessed, not read in full] | A **third-party** review and benchmark (Cornell / Kansas State), not Ultralytics' own paper. Every YOLO26 fact used here comes from the Ultralytics documentation (S1). Not used. |
| — | "Efficient processing of side-scan sonar images and fast detection of sparse targets in large-scale images" (2025), doi:10.1080/17538947.2025.2510568 | [not accessed] | Taylor & Francis answers HTTP 403 here. Asked of the owner 2026-10-23; nothing rests on it. |
| R16 | Chavez, P.S. Jr. (1986). Processing techniques for digital sonar images from GLORIA. *PE&RS* 52(8):1133–1145 | [not accessed] | The USGS record states no digital version. Anamorphosis is covered by R1 §4.5.2. |
| R17 | Sheffer, T., Guterman, H. (2018). Geometrical correction of side-scan sonar images. *IEEE ICSEE*. doi:10.1109/ICSEE.2018.8646188 | [abstract] | IEEE paywall. |
| R18 | Anstee, S. (1994). DSTO MRL-RR-1-94 | [not accessed] | No accessible copy (the sim's own docs record "abstract only"). |

## Citation discrepancies found in project documents

- (2026-10-23) The task prompt calls arXiv:2509.25164 "the YOLO26 paper"; it is a
  third-party review (above), not by Ultralytics.
- (2026-10-23) SS-YOLO (R24) is described as a USV detector: part of its data was
  collected from a USV, but running the model on the vessel is stated as future work (§5).

- `project_synthesis.md` §13 lists "Sethuraman, Sheppard, Bhandari et al. (2024)". The paper
  (R12) has Bagoren, not Bhandari. The Aug Studio's `bibliography.bib` repeats the error.
- The Aug Studio bib entry `alrawi2017` carries a wrong title ("First-bottom-return
  detection for sidescan sonar imagery"); the real title is in R9.
- R8's own reference [9] dates R14 to 2002; R14 is 1992 (JOE 17(3)).
- GCS `SCIENTIFIC_BACKGROUND.md` §2 said the stream is "not TVG-compensated" and the sim
  uses `tvg_compensation: 0`. The sim's `sonar_model.md` §3 / §6 table says 0.30 since
  2026-09-18 ("the device *does* compensate part of its range loss"), while its own §6
  table states that only `spreading · (1 − tvg)` is determined. **Corrected 2026-10-16**
  in `SCIENTIFIC_BACKGROUND.md` §2: the TVG fraction is not identifiable from the
  device's output. See STUDY §1.4.
- Search-engine summaries of R22 give two different page ranges (721–730 and 785–789);
  neither could be checked against the paper.
