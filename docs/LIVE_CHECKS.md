# Phase 3 — live checks (2026-10-16)

The whole chain was run as in operation, on this laptop, headless:
- **Simulator world mission:** Gazebo, the simulated sonar and the control stack.
- **MCS:** its real `LaunchManager`, with the Perception companion it starts.
- **GCS:** the real station, offscreen, driven to START and Record by a timer.
- **Perception:** the node.

Every output went to a scratch data root (`BLUEBOAT_DATA_DIR`). Nothing was written to
`~/ros2_ws/data` or `~/worlds`.

## Setup
| Item | Value |
|---|---|
| World | `generate_world --config config/default_world.yaml` (the simulator's default 80 × 60 m basin, 4 m mean depth, 29 objects), built into the scratch root; no GPS anchor (the shim's legacy origin) |
| Mission | a local-frame lawnmower, 3 lines + 2 U-turns, 155.7 m at 0.6 m/s (`blueboat_trajectory/1`), PID |
| Launch | `LaunchManager.start(LaunchParameters(simulation=True, world_dir=…, gps_deployed_target=…, controller_type="PID", perception_mode=…, extra_args={"headless": "true"}))`, i.e. `full_mission_launch.py` + the companion `ros2 launch blueboat_perception perception_launch.py mode:=… world_dir:=…` |
| GCS | `blueboat_gcs` wiring as `main.py`, Sonar: Simulation; START at 60 s, Record ON at 65 s, Record OFF + STOP near the end |
| ROS | `ROS_DOMAIN_ID=77`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (isolated) |
| Acquisition | the defaults: 20 m, 1200 bins; pictures 256 × 2400, square rows |

## Check 1 — Normal session (run 1: 300 s mission, 223 s of recording)

| | Result |
|---|---|
| Seabed images emitted by the GCS | 47 (one stream, ids 0–46), every 4.4–5.0 s |
| Records published by Perception | **47, one per image**; all `ran`, all joins `complete` |
| Detections back in the GCS (`/sss_ai/detections` → `signals.detection`) | **47**, one per image; `detections.csv` of the session holds the same 47 rows |
| Losses | `gaps` 0, `dropped_no_metadata` 0, `partial_no_pixels` 0 |
| Georeferencing | schema 5, georef convention 2 on every image. The first dummy lies 0.285 m starboard of the track at yaw 0: the lever arm at the nadir bin |
| Attitude | `yaw_rate_source: imu` on every record (the simulator's mavros shim); 5864 IMU samples against 5609 pings |

**Found and fixed during the check.** The join counted 2 `duplicates` that were not
duplicates. In normal mode a key is released on pixels + metadata, and 2 of the 47 PNGs
arrived after their metadata: the three topics carry no ordering guarantee. Such a part is
now counted as `late_parts`, and a real repeat as a duplicate
(`tests/test_join.py::test_a_png_after_release_is_late_not_duplicate`).

## Check 1b — Dataset creation (run 2: 210 s mission)
MCS ran the companion as `mode:=dataset dataset_name:=2026_10_16-14_32_41-sim-p3-dataset
world_dir:=…`, and the launch file claimed the folder and started the ground-truth
recorder.

| | Result |
|---|---|
| Images → records → detections back in the GCS | 27 → 27 → 27; `gaps` 0, `late_parts` 0, `duplicates` 0 |
| Dataset folder | `manifest.json` (`image_count` 27, `skipped_no_png` 0, the stream id, `world_dir`, `ground_truth_bag`), `data.yaml` (`names: {}`), 27 × (`images/<stem>.png` + `.json`, `labels/<stem>.txt` empty, `raw/<stem>_db.npz`) |
| PNG vs the session's PNG | **27 / 27 byte-identical** (live: the same OpenCV encoding of the same mono8 array) |
| Raw dB vs the session's `_world.npz` | 27 / 27 identical |
| Aug Studio | all 27 sidecars validate as its `AcquisitionMeta` (its own `.venv`); its `YoloDataset` loads the folder: 27 images |
| Ground truth | `ground_truth_bag/`: 189 s, 1512 `contacts` + 3785 `sensor` messages. Perception read none of them |
| Offline = live | 27 / 27 identical |

## Check 2 — Offline = live
`blueboat-perception-offline <session> --compare <live log>` over run 1's recorded
session: **47 / 47 records byte-identical** (canonical JSON without `source` and
`provenance`), 0 different, 0 live-only, 0 offline-only, in 1.1 s.

## Check 3 — Latency (run 1, wall clock, same host)

| Stage | p50 | p95 | max |
|---|---|---|---|
| GCS publish job (`to_png8` + envelope + three publishes, its own thread) | 19.1 ms | 28.4 ms | 54.7 ms |
| Perception join (first part → last part) | 2.7 ms | 13.6 ms | 3012 ms ¹ |
| Perception pipeline | 22.1 ms | 23.9 ms | 25.0 ms |
| Perception publish | 3.4 ms | 4.1 ms | 4.4 ms |
| Perception total (last part → published) | 25.5 ms | 27.2 ms | 29.0 ms |
| **End to end, image emitted in the GCS → its detection back in the GCS** | **51.2 ms** | **62.3 ms** | 3072 ms ¹ |

¹ **Image 0 only, in all three runs: 3063–3072 ms**. Every later image takes 48–61 ms
end to end, and joins in ≤ 14 ms. The delay is the first large sample on the new RELIABLE
connections arriving one part late, by ~3 s every time. That is consistent with its
repair waiting for a DDS heartbeat; Fast DDS's default heartbeat period is 3 s. This is an
inference, not verified. Nothing is lost. Against the image period of 4.4–5.0 s, the margin
is ~80× at p95. The design target was < 250 ms (DESIGN §10).

**The ping path (timing run, 70 s bag, wire receive times):**
- profile on the wire → its processed row on the wire: **13.8 ms p50, 18.1 ms p95,
  22.8 ms max** (1371 pings; the processor's pairing);
- `/mavros/imu/data` and `compass_hdg` (the simulator's shim): 20.0 Hz, intervals ≤ 53 ms;
- simulation ran at real time (1.000).

## Check 5 — the fixed detector input, live (2026-10-16, after decisions 6.2–6.5)
Run 4: 135 s, dataset mode, the new defaults (`preprocess: fixed_gray`, mapping
`fixed_gray_v1`, turn flags, the GCS speed window at 2 s).

| | Result |
|---|---|
| Images → records → detections | 15 → 15 → 15 |
| Offline = live | **15 / 15 records identical**, `processing.preprocess`, `summary.detector_input` and the turn flags included |
| Live training images (from the wire) vs `fixed_gray` applied offline to the session's raw dB | **15 / 15 pixel-identical** |
| Dataset | 15 training images (the detector input), 15 display PNGs preserved in `display/`; sidecar `intensity_mapping: log`, `log_range_db` 48.31 |
| Turn flags | 1966 rows with `r* < R` (the U-turn), all rows and detections kept |

## Check 4 — the real-boat run (commands for you)

Nothing changes in the field procedure: the station starts Perception itself with every
real-robot launch, in normal mode.

```bash
# 0. once, after pulling: build what changed (from ~/ros2_ws)
cd ~/ros2_ws && colcon build --packages-select blueboat_perception blueboat_sss

# 1. the station (as usual: Launch Mission -> Real robot). The console then shows
#    "$ ros2 launch blueboat_perception perception_launch.py mode:=normal   [perception]"
#    and "[perception] ... perception 0.1.0: mode=normal ...".
cd ~/ros2_ws && source env.sh && cd src/BlueBoat-SideScanSonar/BlueBoat-MCS && python3 run.py

# 2. the GCS (as usual), Sonar: Real sonars, START, Record ON for the survey
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-SSS/blueboat_sss && python3 -m blueboat_gcs.main

# 3. during the run, from any sourced shell: Perception's counters and latency
ros2 topic echo --once /sss_ai/perception/status std_msgs/msg/String \
    --qos-durability transient_local --qos-reliability reliable

# 4. after the run: offline over the recorded session, compared with the live log
python3 -m blueboat_perception.cli.offline \
    ~/ros2_ws/data/SSS_data/sessions/<stamp> \
    --compare ~/ros2_ws/data/Perception_data/live/<stream_id>.jsonl
```

What to look at:
- one dummy marker per seabed image, on the map and in the waterfall;
- `gaps` 0 and `dropped_no_metadata` 0 in the status;
- `compare: N/N identical`;
- the latency in the status. The images go from the GCS to Perception on the laptop
  itself, so the Base Station link carries none of them;
- the real IMU's roll / pitch / yaw rate in the pictures' `rows_data`; `attitude_age_s`
  there is the age of the sample the GCS attached.
