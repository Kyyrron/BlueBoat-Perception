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

Dev tool (not installed): `tools/preprocessing_study.py` reproduces the Phase 4
measurements of `docs/PREPROCESSING_STUDY.md` §5 from the recorded sessions (read-only).
