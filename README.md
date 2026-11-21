# BlueBoat-Perception

Finds objects on side-scan sonar seabed images for the BlueBoat survey.
It is the station's **only detection source**: the GCS shows its detections and runs no detector of its own.
By default it runs a **dummy detector** (one point per image).
A YOLO detector over 256 × 256 tiles is built, but **no trained weights exist yet**.

> **Scores are not probabilities.** Calibration is identity today, so every record says
> `calibrated: false` and `confidence` is a raw detector score.

## Where it fits

```
sonar (BlueBoat-SSS, GCS seabed images) → Perception → Belief → Replanner → Control
```

Inside the module:

```
seabed image (raw dB + PNG + metadata) → join → detector input (fixed_gray)
  → detector (dummy, or YOLO over tiles) → calibration (identity) → georeference
  → /sss_ai/detections  +  one record per image on /sss_ai/image_records
```

Everything runs on the operator laptop, like every node of the project.

## Prerequisites

- ROS 2 Jazzy, workspace `~/ros2_ws`.
- ROS packages: `rclpy`, `std_msgs`, `sensor_msgs`, `vision_msgs`, `launch`, `launch_ros`, `ros2bag`.
- Python: `numpy`, `opencv` (`python3-opencv`), `PyYAML` (the superproject's `requirements.txt` lists them).
- **YOLO only:** `ultralytics` and `torch`, importable by the node's Python. They are not needed for the dummy
  and are not in `requirements.txt`; install them yourself.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select blueboat_perception
source install/setup.bash
```

## Run

### Live node

MCS starts the node for real-robot launches and for Gazebo runs in a built world. To start it by hand:

```bash
ros2 launch blueboat_perception perception_launch.py                       # normal mode
ros2 launch blueboat_perception perception_launch.py use_sim_time:=true    # in a simulation graph
```

Launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `mode` | `normal` | `normal` or `dataset` |
| `dataset_name` | time-stamped | dataset folder name (dataset mode) |
| `world_dir` | empty | simulated world folder; turns on labels from simulator ground truth |
| `use_sim_time` | `false` | `true` in simulation: label wait and ground-truth bag on `/clock` |
| `data_dir` | empty | data root (else `$BLUEBOAT_DATA_DIR`, else the workspace) |

Check the node during a run:

```bash
ros2 topic echo --once /sss_ai/perception/status std_msgs/msg/String \
    --qos-durability transient_local --qos-reliability reliable
```

### Dataset mode (training data from the simulator)

```bash
ros2 launch blueboat_perception perception_launch.py mode:=dataset \
    dataset_name:=2026_11_21-10_00_00-test world_dir:=<world folder> use_sim_time:=true
```

- Writes images, YOLO labels and a manifest to `<root>/data/Dataset_data/<name>/`.
- With `world_dir`, it also records the simulator ground truth into `<dataset>/ground_truth_bag/`.
- Labels come from the rendered picture and the world metadata. Ground truth never reaches the detections.
- Simulated datasets must follow the superproject's `SIM_VALIDITY_CONDITIONS.md`
  (production sonar profile, envelope, disjoint seeds for dataset and evaluation worlds).

### Offline over a recorded session

The same pipeline, no ROS needed. Output equals the live output (once `provenance` and `source` are removed).

```bash
python3 -m blueboat_perception.cli.offline ~/ros2_ws/data/SSS_data/sessions/<stamp>
ros2 run blueboat_perception blueboat-perception-offline <session>          # same, installed

# compare with the live log of that run
python3 -m blueboat_perception.cli.offline <session> \
    --compare ~/ros2_ws/data/Perception_data/live/<stream_id>.jsonl

# re-label a dataset-mode run offline
python3 -m blueboat_perception.cli.offline <session> --dataset NAME \
    --world <world folder> --ground-truth-bag <dataset>/ground_truth_bag
```

Useful options: `--detector yolo --detector-model <best.pt>`, `--preprocess fixed_gray|identity`,
`--near-range-mask-m 0.75`, `--out DIR`. Full list: `--help`.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/sss_ai/seabed_image` | `sensor_msgs/Image` 32FC1 (raw dB) | in, from the GCS |
| `/sss_ai/seabed_image/png` | `sensor_msgs/Image` mono8 | in, from the GCS |
| `/sss_ai/seabed_image/metadata` | `std_msgs/String` (JSON) | in, from the GCS |
| `/sss_ai/detections` | `vision_msgs/Detection2DArray`, one per image | out, to the GCS map |
| `/sss_ai/image_records` | `std_msgs/String` (JSON, `image_record/1`) | out, to Belief and analysis |
| `/sss_ai/perception/status` | `std_msgs/String` (JSON), 1 Hz | out |

- `/sss_ai/detections` carries only detections with `confidence >= map_min_confidence` (0.10).
  The records keep every detection.
- One record per image, always. `detections: []` = searched, nothing found; `null` = could not search.

Fields, QoS and parameters: `docs/topics.md` (the interface contract).

## Configuration

Node defaults: `blueboat_perception/config/perception.yaml` (installed; rebuild after editing it,
or override with `-p` on the node). The main keys:

| Key | Default | Meaning |
|---|---|---|
| `detector` | `dummy_center` | `dummy_center` or `yolo` |
| `detector_model` | `""` | YOLO weights, e.g. `<root>/data/Perception_models/<name>/weights/best.pt` |
| `detector_device` | `cpu` | `cpu` (reproducible) or `cuda:0` |
| `detector_tiling` | `tiles` | `tiles` (256 × 256) or `full` |
| `tile_stride` | `128` | px across track |
| `merge` | `nms` | `nms` or `greedy_nmm` (merging boxes of overlapping tiles) |
| `map_min_confidence` | `0.10` | cut for the GCS map only |
| `preprocess` | `fixed_gray` | detector input: `fixed_gray` or `identity` (the display PNG) |
| `near_range_mask_m` | `0.0` | `0.0` (off) or `0.75` |
| `calibration` | `identity` | only identity exists today |
| `record_log` | `true` | write the live record log |

Switch to YOLO without editing the file:

```bash
ros2 run blueboat_perception perception_node --ros-args \
    --params-file ~/ros2_ws/install/blueboat_perception/share/blueboat_perception/config/perception.yaml \
    -p detector:=yolo -p detector_model:=<best.pt>
```

YOLO11s and YOLO26s differ only by the weights file; the record names the family.

## Training and evaluating YOLO (offline tools)

The tools in `tools/` are not installed. Run them from the repository root. All results are **on simulated
data**. Split by world, never by image. Never tune on the test worlds.

```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-Perception
DS=~/ros2_ws/data/Dataset_data
T=~/ros2_ws/data/Tile_data
M=~/ros2_ws/data/Perception_models

# 1. split file, by world (or dataset) folder name
cat > split.yaml <<'YAML'
train: [world_a, world_b, world_c, world_d, world_e, world_f]
val:   [world_g, world_h]
test:  [world_i, world_j]
YAML

# 2. look at the labels (overlays, crops, visibility statistics)
/usr/bin/python3 tools/draw_labels.py $DS/<dataset> --out DIR --crops --stats

# 3. tile dataset (also: --stride 64, or --mode full for the full-picture baseline)
python3 tools/make_tile_dataset.py $DS/* --split split.yaml --name tiles128

# 4. train (--model yolo11s.pt or yolo26s.pt; --device 0 for a GPU)
python3 tools/train_yolo.py --data $T/tiles128/data.yaml --model yolo11s.pt --name y11s-t128-s0

# 5. evaluate on val (choose merge rule and threshold here), then once with --on test
python3 tools/evaluate_detector.py $DS/* --split split.yaml --on val --name y11s-t128-nms \
    --detector yolo --detector-model $M/y11s-t128-s0/weights/best.pt --merge nms
```

- Each step writes a JSON beside its output (configuration, split, seed, versions).
- `train_yolo.py` sets augmentation explicitly; `--no-flips` turns off the flips.
- Not decided yet: the merge rule (`nms` / `greedy_nmm`) and the 2 dB label visibility threshold.

Other tools: `tools/preprocessing_study.py` (reproduces `docs/PREPROCESSING_STUDY.md` §5, read-only) and
`tools/fit_detector_input.py` (fits a new detector-input mapping on real data, under a new name).

## Output files

All under `<root>/data/` (normally `~/ros2_ws/data/`). Nothing is overwritten: a taken name gets `-2`, `-3`, ….

| Folder | Written by | Content |
|---|---|---|
| `Perception_data/live/<stream_id>.jsonl` | live node | one image record per line |
| `Perception_data/offline/<session>/` | offline CLI | `records.jsonl`, `detections.json`, `summary.json` |
| `Dataset_data/<name>/` | dataset mode | `images/`, `display/`, `labels/`, `raw/`, `manifest.json`, `data.yaml`, `ground_truth_bag/` |
| `Tile_data/`, `Perception_models/`, `Perception_eval/` | offline tools | tile datasets, trained models, evaluations |

A recording session is only read, never written.

## Tests

From `BlueBoat-Perception/blueboat_perception/`, with the system Python
(do not source `~/ros2_ws/install/setup.bash` first: its venv Python lacks pytest):

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest
```

## Layout

```
BlueBoat-Perception/
  blueboat_perception/            ROS 2 package (ament_python)
    blueboat_perception/core/     ROS-free pipeline: join, preprocess, detectors, tiling, merge, records, labels
    blueboat_perception/ros/      the node (perception_node.py) and ground-truth subscriber
    blueboat_perception/cli/      offline.py, the offline runner
    config/perception.yaml        node defaults
    launch/perception_launch.py
    tests/
  tools/                          offline tools: datasets, training, evaluation, studies (not installed)
  docs/                           design and contract documents
```

## Further reading

| Document | What |
|---|---|
| `docs/topics.md` | interface contract: topics, record fields, parameters |
| `docs/DESIGN.md` | why the pipeline is built this way (tiling, merge, labels) |
| `docs/PREPROCESSING_STUDY.md` | detector-input study and decisions (§6) |
| `docs/LIVE_CHECKS.md` | live checks and the real-boat command sheet |
| `docs/REFERENCES.md` | literature behind every non-software choice |
| `docs/REQUESTS.md` | requests to neighbouring modules |
| `../BlueBoat-SSS/blueboat_sss/blueboat_gcs/docs/PERCEPTION_INTEGRATION.md` | the GCS side |
| `../SIM_VALIDITY_CONDITIONS.md` | rules for simulated datasets and results |

Licence: Apache-2.0.
