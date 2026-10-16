# Requests to other modules — BlueBoat-Perception

Perception is built inside this module, plus the BlueBoat-SSS and BlueBoat-MCS changes you
ordered (DESIGN §2). **BlueBoat-Control, BlueBoat-SSS-Sim and SSS-Dataset-Aug-Studio are not
edited.** What they would need to change is listed here, as requests for their owners to
schedule. The format follows `BlueBoat-Control/docs/REQUESTS.md`.

The IDs are stable: R-1 (Control: full attitude in `/blueboat/odom`, usable mavros stamps)
and R-2 (simulator: ground truth usable for labels, plus stale docs) were withdrawn on
2026-10-16 by your order. You handle the yaw item separately, and the simulation is not
modified in this work.

## R-3 SSS-Dataset-Aug-Studio — an intensity mapping for the GCS display transfer

**Status 2026-10-16: resolved for the default detector input.** Under `fixed_gray`
(decision 6.4) a dataset's training images are a linear dB window, which the Aug Studio's
existing `log` mapping inverts exactly. The sidecars declare it, and the Aug Studio's own
`AcquisitionMeta` validates them (`tests/test_dataset.py`). The request below stands
only for datasets made with `preprocess: identity` (the display PNG).

**Need.** The GCS seabed PNG is the display model's `power` transfer: transmission loss
removed, the per-side empirical curve divided out, then `u = 10^(γ(e−hi)/10)` with a soft
knee (`blueboat_gcs/core/display_model.py`). The Aug Studio's `IntensityMapping`
(`linear | gamma | log`, `sss_aug_studio/core/meta.py`) cannot express it. Without a
declaration the Aug Studio assumes `log` and mis-inverts.

**What the dataset writer provides (built 2026-10-16, `core/dataset.py`).**
- Every dataset image ships its raw float dB (`raw/<stem>_db.npz`) and the full display
  model in the sidecar's `gcs_metadata.display_model`. The JSON is invertible
  (P4 *Invertibility*).
- The sidecar sets only the `AcquisitionMeta` fields the metadata determines, within the
  Aug Studio's own bounds:
  - always `layout`; `altitude_m` when ≥ 0.1 m; `slant_range_m`; `res_along_m`;
    `speed_mps`; compass `heading_deg`; `depth_m`;
  - `prf_hz` in ping geometry only; `timestamp` only for wall-clock times.
  - **`intensity_mapping` is not set**, and neither is `res_across_m`: the columns are
    slant bins, not ground range. The manifest says both.
  - Validated against the Aug Studio's own `AcquisitionMeta` (pydantic, its `.venv`) by
    `tests/test_dataset.py`.
- The Aug Studio's `extra="ignore"` makes the GCS block harmless.

**Suggested behaviour.** Either:
- a new mapping that reads the GCS display model from the sidecar (`gcs_display_power`); or
- support for loading the raw dB from `raw/` with a declared window.

Until then, intensity-domain families must not be run on Perception datasets.

## R-4 Superproject — root `CLAUDE.md` rows (Phase 5)

The interface-table rows, the module table entry, the build-order line and the
`module-gates` entry for BlueBoat-Perception are delivered in Phase 5, for you to apply.

Also found while reading:
- the root `CLAUDE.md` §4.3 "Transducer lateral offset — disagreement" row (processor 0.0
  vs sim 0.20) is stale: the processor has used 0.285 since SSS `994a74b`, and the
  simulator's `config/default_sonar.yaml:36` has `mount_y_abs_m: 0.285`;
- §4.3 calls the `/sss_ai/seabed_analysis` payload "schema 1". It follows the picture
  metadata: 4 until 2026-10-16, 5 since. Its `detections` are now always `[]`; the
  detections are on `/sss_ai/detections` from Perception;
- the §4.3 `/sss_ai/detections` row says "*(none)* → GCS, placeholder": the producer is
  now BlueBoat-Perception;
- the §4.1 `/blueboat/odom` row's yaw-only real attitude: the GCS now takes attitude from
  `/mavros/imu/data` instead (Q8);
- `project_synthesis.md` §13's "Bhandari" (REFERENCES, discrepancies).

## Coordination note — BlueBoat-MCS launch dialog

`BlueBoat-Control/docs/REQUESTS.md` R-1 (2026-10-15, another work stream) asks MCS for a
per-run "Replanner mode" choice in the same launch dialog. Our MCS change (built
2026-10-16) is compatible by construction:
- the **Session** row is its own `QComboBox` row after *Robot file*, on its own field
  (`LaunchParameters.perception_mode`);
- that field is not a launch argument, so `to_cli()`, `launch_target()` and
  `companion_command()` are unchanged (pinned in `smoke_test.py`, `perception ok`);
- a "Path source" row can be added beside it the same way without touching it.

The Replanner mode itself is not implemented here, by your order.
