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

**Status 2026-10-16: applied**, by your order ("just do it yourself"), directly in the
superproject `.claude/CLAUDE.md` (§1 diagram, module table and deployment list, §2 layout
and build order, §3 the MCS order of 2026-10-16, §4.1 / §4.3 / §4.4 / §4.5 rows, §6 D7, §7
table and CLAUDE.md map) and in the skills `module-gates` (a BlueBoat-Perception row; the SSS
and MCS counts) and `file-interfaces` (Perception record logs and datasets; the seabed-image
detector input; the stale 2/98 stretch statement).

Corrected on the way (found while reading):
- the §4.3 "Transducer lateral offset — disagreement" row (processor 0.0 vs sim 0.20): both
  are 0.285 (processor since SSS `994a74b`; simulator `config/default_sonar.yaml:36`). The
  simulator's own `TODO.md` [P1] still describes the old disagreement; it is not edited
  from this work;
- §4.3's "schema 1" for `/sss_ai/seabed_analysis`: the picture metadata's schema, 5 since
  2026-10-16, with `detections` always `[]`;
- the §4.3 `/sss_ai/detections` "*(none)* → GCS, placeholder" row: produced by
  BlueBoat-Perception.

Still open, not a row of this module: `project_synthesis.md` §13's "Bhandari" (REFERENCES,
discrepancies), for you to judge.

## R-5 BlueBoat-SSS — a convention-3 twin of the live-path georeference test (2026-11-15)

**Status 2026-11-15: open**, for the BlueBoat-SSS owner. Nothing in BlueBoat-SSS was edited
(CM-3).

**Need.** `tests/test_perception_golden.py`
`test_live_perception_path_georeferences_with_convention_2` checks, through the live envelope,
that Perception's world point equals the station's own grid at that pixel. Its golden pings
carry no fore/aft arm, so it only exercises convention 2. Since 2026-11-15 every live session
is georef convention 3, and nothing on the SSS side pins Perception's placement of such a
picture. Session 06 could not add the twin, because it failed until CR-PER-2 was applied in
Perception (2026-11-15: `core/geometry.py` `to_world` / `sensor_point`).

**Proposed test** (next to the convention-2 one, same fixtures; `from dataclasses import
replace` at the top):

```python
ARM = -0.345


@pytest.mark.parametrize("variant", _variants())
def test_live_perception_path_georeferences_with_convention_3(qapp_module, pings, variant):
    """Pings carrying the fore/aft arm make convention-3 pictures; through the
    live envelope, Perception's world point is the station's own convention-3
    grid at that pixel."""
    from blueboat_gcs.core import perception_core
    core, why = perception_core.load()
    if core is None:
        pytest.skip(why)
    g, k = variant.split("/")
    armed = [replace(p, x_offset_m=ARM) for p in pings]
    images = cap.run_variant(g, k, armed)
    assert images
    for img, record, dets in perception_core.detect(images, core):
        assert img.georef["convention"] == 3 and img.georef["lever_arm_m"]["x"] == ARM
        assert record["input"]["georef_convention"] == 3
        (d,) = dets
        r, c = d["pixel"]
        assert d["world_xy"] == [float(img.world_x[r, c]), float(img.world_y[r, c])]
```

**Checked from outside BlueBoat-SSS** (a copy of the fixtures and this body,
`~/ros2_ws/sim_runs/sim_acceptance/06_sss_followup_perception/sss_twin/`):
- 4 passed against the Perception source checkout of 2026-11-15;
- 4 failed against the committed `HEAD` (`1e9264f`), world x off by 0.345 m.

`perception_core.load()` takes the installed package before the checkout, so on a shell that
sources `~/ros2_ws/install` the test needs `colcon build --packages-select blueboat_perception`
first.

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
