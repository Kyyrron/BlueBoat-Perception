# BlueBoat-Perception

Seabed images in, detections and image records out, for the BlueBoat
aspect-aware side-scan sonar survey.

```
GCS seabed image + metadata  ->  preprocess (fixed_gray)  ->  detector (dummy; YOLO later)
                             ->  calibration (identity)  ->  detections (/sss_ai/detections)
                                                          +  one record per image (/sss_ai/image_records)
```

* `blueboat_perception/` — the ROS 2 package (`ament_python`):
  * `blueboat_perception/core/` — ROS-free, deterministic core (numpy + stdlib);
  * `blueboat_perception/ros/perception_node.py` — the thin node;
  * `blueboat_perception/cli/offline.py` — the same pipeline over a recorded session;
  * `launch/perception_launch.py` — `mode:=normal|dataset`.
* `docs/` — `DESIGN.md`, `topics.md` (the interface contract), `PREPROCESSING_STUDY.md`,
  `REFERENCES.md`, `REQUESTS.md`.

## Commands

```bash
# build (from ~/ros2_ws)
colcon build --packages-select blueboat_perception

# run (MCS starts it for personalized-world Gazebo runs and real-robot sessions)
ros2 launch blueboat_perception perception_launch.py                 # normal
ros2 launch blueboat_perception perception_launch.py mode:=dataset dataset_name:=2026_10_16-10_00_00-test

# offline over a recorded session (no ROS needed)
python3 -m blueboat_perception.cli.offline ~/ros2_ws/data/SSS_data/sessions/<stamp>
ros2 run blueboat_perception blueboat-perception-offline <session>   # same, installed

# tests (from BlueBoat-Perception/blueboat_perception/, system python)
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest
```

Outputs go under `<root>/data/` (normally `~/ros2_ws/data/`): `Perception_data/live/`,
`Perception_data/offline/`, `Dataset_data/`. Licence: Apache-2.0.

## What is in `Perception_data/`

`~/ros2_ws/data/Perception_data/` holds what Perception writes about the seabed images it
has processed. It has two subfolders. Neither is ever overwritten: a name already taken
gets `-2`, `-3`, … appended.

```
Perception_data/
  live/<stream_id>.jsonl          written while the node runs
  offline/<session>/              written by blueboat-perception-offline
      records.jsonl
      detections.json
      summary.json
```

* **`live/<stream_id>.jsonl`** — the live record log, one file per stream (one GCS run,
  e.g. `gcs-20261020T004952Z-f969`). Each line is one JSON **image record**, written as
  soon as that image is processed, so a node that is killed leaves every finished line
  readable. These are the same records published on `/sss_ai/image_records`. A record holds:
  the image key (`stream_id/image_id`), its input and time span, the boat track, the
  per-side summary, the footprint on the seabed, the processing used (preprocessing,
  detector, calibration), the detections, and `provenance`. `detections: []` means the
  image was searched and nothing was found; `null` means there were no pixels.
  `source` is `live`.
* **`offline/<session>/records.jsonl`** — the same records, made by running the pipeline
  over a saved GCS session (`source: offline`). Nothing is read from or written into the
  session itself.
* **`offline/<session>/detections.json`** — every detection of that run in one flat list.
* **`offline/<session>/summary.json`** — counts (images, detections, detector statuses),
  the stream ids, the pipeline configuration, the run time, and, with `--compare`, the
  result of comparing against a live log.

Offline and live are meant to be equal: `--compare live/<stream_id>.jsonl` checks that the
records match byte for byte once `provenance` and `source` are removed.

Dataset folders are not here: they go to `Dataset_data/` (dataset mode only).

Dev tool (not installed): `tools/preprocessing_study.py` reproduces the Phase 4
measurements of `docs/PREPROCESSING_STUDY.md` §5 from the recorded sessions (read-only).
