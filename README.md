# BlueBoat-Perception

Seabed images in, detections and image records out, for the BlueBoat
aspect-aware side-scan sonar survey.

```
GCS seabed image + metadata  ->  preprocess (fixed_gray)  ->  detector (dummy, or YOLO over 256x256 tiles)
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
`Perception_data/offline/`, `Dataset_data/`, and, from the offline tools, `Tile_data/`,
`Perception_models/`, `Perception_eval/`. Licence: Apache-2.0.

## Switching the detector (dummy ↔ YOLO11s ↔ YOLO26s)

The detector is one parameter, `detector`, in `blueboat_perception/config/perception.yaml`
(node) or `--detector` (offline CLI). Nothing else changes: same topics, same records.

| Detector | Set | Notes |
|---|---|---|
| **Dummy** (default) | `detector: dummy_center` | the station's former dummy: one point at the picture centre, score 0.5, `"dummy": true` everywhere. No model, no Ultralytics needed |
| **YOLO11s** | `detector: yolo`, `detector_model: <root>/data/Perception_models/<a YOLO11s run>/weights/best.pt` | needs `ultralytics` + `torch` importable by the node's Python |
| **YOLO26s** | the same, `detector_model:` a YOLO26s run's `best.pt` | the family is read from the weights; the record says which (`processing.detector.params.model.card.family`) |

YOLO11s and YOLO26s differ only by the weights file: which family a run is depends on the
`--model` it was trained from (`yolo11s.pt` or `yolo26s.pt`, below). Back to the dummy:
`detector: dummy_center` (the `detector_model` line is then ignored).

```yaml
# config/perception.yaml (then colcon build, or pass the same keys with -p on the node)
detector: yolo
detector_model: /home/<you>/ros2_ws/data/Perception_models/yolo26s-tiles128-s0/weights/best.pt
detector_device: cpu          # cpu = reproducible (offline = live); cuda:0 for a GPU
merge: nms                    # or greedy_nmm, once the comparison has chosen
map_min_confidence: 0.10      # what the GCS map draws; the records keep every detection
```
```bash
# the same without editing the file
ros2 launch blueboat_perception perception_launch.py   # uses the yaml
ros2 run blueboat_perception perception_node --ros-args --params-file <perception.yaml> \
    -p detector:=yolo -p detector_model:=<best.pt>
# offline over a session
python3 -m blueboat_perception.cli.offline <session> --detector yolo --detector-model <best.pt>
```

**How YOLO sees a picture.** A picture is 256 rows × 1200–2402 slant bins. It is cut into
256 × 256 tiles at native resolution (never resized), stride 128 across track, the nadir
kept; every tile goes to YOLO at `imgsz` 256 in one batch; the boxes are shifted back to
picture pixels and the overlapping tiles' duplicates merged (`merge`). One record per
picture, as with the dummy. Details: `docs/DESIGN.md` §4.10.

## Training and comparing YOLO11s / YOLO26s (offline tools, not installed)

All results are **on simulated data**: the datasets come from dataset mode in simulator
worlds; split by world, never by image; the test worlds are never used to tune anything.

```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-Perception
# 0. datasets: one dataset-mode run per world (MCS: launch dialog, Session = Dataset creation)
#    -> ~/ros2_ws/data/Dataset_data/<YYYY_MM_DD-HH_MM_SS>-<note>/
# 1. a split file, by world folder name (or dataset folder name):
cat > split.yaml <<'YAML'
train: [world_a, world_b, world_c, world_d, world_e, world_f]
val:   [world_g, world_h]
test:  [world_i, world_j]
YAML
DS=~/ros2_ws/data/Dataset_data
# 2. tile datasets (one per arm), and the full-picture baseline
python3 tools/make_tile_dataset.py $DS/* --split split.yaml --name tiles128
python3 tools/make_tile_dataset.py $DS/* --split split.yaml --name tiles64 --stride 64
python3 tools/make_tile_dataset.py $DS/* --split split.yaml --name full --mode full
# 3. train (seeded; add --device 0 on a GPU); repeat with --model yolo26s.pt
T=~/ros2_ws/data/Tile_data
python3 tools/train_yolo.py --data $T/tiles128/data.yaml --model yolo11s.pt --name y11s-t128-s0
python3 tools/train_yolo.py --data $T/tiles128/data.yaml --model yolo26s.pt --name y26s-t128-s0
python3 tools/train_yolo.py --data $T/tiles64/data.yaml  --model yolo11s.pt --name y11s-t64-s0
python3 tools/train_yolo.py --data $T/full/data.yaml --imgsz 640 --model yolo11s.pt --name y11s-full-s0
python3 tools/train_yolo.py --data $T/tiles128/data_gray1.yaml --model yolo11s.pt --name y11s-t128-g1-s0
# 4. evaluate on the val worlds (choose merge rule / threshold here), then once on test
M=~/ros2_ws/data/Perception_models
python3 tools/evaluate_detector.py $DS/* --split split.yaml --on val --name y11s-t128-nms \
    --detector yolo --detector-model $M/y11s-t128-s0/weights/best.pt --merge nms
python3 tools/evaluate_detector.py $DS/* --split split.yaml --on val --name y11s-t128-gnmm \
    --detector yolo --detector-model $M/y11s-t128-s0/weights/best.pt --merge greedy_nmm
python3 tools/evaluate_detector.py $DS/* --split split.yaml --on val --name y11s-full \
    --detector yolo --detector-model $M/y11s-full-s0/weights/best.pt --detector-tiling full
python3 tools/evaluate_detector.py $DS/* --split split.yaml --on val --name y11s-t64 \
    --detector yolo --detector-model $M/y11s-t64-s0/weights/best.pt --tile-stride 64
# ... the same for the YOLO26s runs; after the choices are fixed, the same commands with --on test
```
Each step writes a JSON beside its output (`manifest.json`, `model.json`,
`results.json`): configuration, dataset and world split, seed, model, library and code
versions. Augmentation is set explicitly in `tools/train_yolo.py` (no hue / saturation,
rotation, shear, perspective, mosaic, scale or translate; whole-tile flips only;
`--no-flips` for the heading-response experiment). `tools/draw_labels.py DATASET --out DIR
--stats` shows the label boxes and each box's visibility class (2 dB rule).

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
