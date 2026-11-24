# BlueBoat-Perception — user guide

What you can do with the detector module, in detail. For installation, see the
[README](../README.md). Every command assumes `cd ~/ros2_ws && source env.sh` was run first.

## 1. The live node

The Mission Control Station starts the node for real-boat launches and for Gazebo runs in a built
world. To start it by hand:

```bash
ros2 launch blueboat_perception perception_launch.py                       # normal mode
ros2 launch blueboat_perception perception_launch.py use_sim_time:=true    # in a simulation
```

| Launch argument | Default | Meaning |
|---|---|---|
| `mode` | `normal` | `normal` or `dataset` |
| `dataset_name` | time-stamped | Dataset folder name (dataset mode) |
| `world_dir` | empty | A simulated world folder; turns on labels from the simulator's ground truth |
| `use_sim_time` | `false` | `true` in simulation: the label wait and the ground-truth recording follow the simulated clock |
| `data_dir` | empty | Data root (else `$BLUEBOAT_DATA_DIR`, else the workspace) |

Check the node during a run:

```bash
ros2 topic echo --once /sss_ai/perception/status std_msgs/msg/String \
    --qos-durability transient_local --qos-reliability reliable
```

## 2. Dataset mode: training data from the simulator

```bash
ros2 launch blueboat_perception perception_launch.py mode:=dataset \
    dataset_name:=2026_11_21-10_00_00-test world_dir:=<world folder> use_sim_time:=true
```

- Writes images, YOLO labels and a manifest to `<root>/data/Dataset_data/<name>/`.
- With `world_dir`, it also records the simulator's ground truth into `<dataset>/ground_truth_bag/`.
- One label box per object covers its bright echo and its shadow. It is computed from the rendered
  picture, the world's object list and the beam geometry. The simulator's ground truth only says which
  objects a ping saw; it never reaches the detections.
- Simulated datasets follow the superproject's `SIM_VALIDITY_CONDITIONS.md`: the production sonar
  profile, the validated conditions, and different seeds for dataset worlds and evaluation worlds.

From the Mission Control Station, choose the **Session** row "dataset" in the launch dialog: it
starts this mode for you.

## 3. Offline over a recorded session

The same pipeline without ROS. The output equals the live output (once the `provenance` and
`source` fields are removed).

```bash
python3 -m blueboat_perception.cli.offline ~/ros2_ws/data/SSS_data/sessions/<stamp>
ros2 run blueboat_perception blueboat-perception-offline <session>          # the same, installed

# compare with the live log of that run
python3 -m blueboat_perception.cli.offline <session> \
    --compare ~/ros2_ws/data/Perception_data/live/<stream_id>.jsonl

# re-label a dataset-mode run offline
python3 -m blueboat_perception.cli.offline <session> --dataset NAME \
    --world <world folder> --ground-truth-bag <dataset>/ground_truth_bag
```

Useful options: `--detector yolo --detector-model <best.pt>`, `--preprocess fixed_gray|identity`,
`--near-range-mask-m 0.75`, `--out DIR`. Full list: `--help`.

The Ground Control Station's replay window has a **Run AI** button that runs this same pipeline on a
recorded log.

## 4. Settings

The node's defaults are in `blueboat_perception/config/perception.yaml` (installed: rebuild after
editing it, or override with `-p` on the node).

| Key | Default | Meaning |
|---|---|---|
| `detector` | `dummy_center` | `dummy_center` or `yolo` |
| `detector_model` | `""` | YOLO weights, e.g. `<root>/data/Perception_models/<name>/weights/best.pt` |
| `detector_device` | `cpu` | `cpu` (reproducible) or `cuda:0` |
| `detector_tiling` | `tiles` | `tiles` (256 × 256) or `full` (the whole picture) |
| `tile_stride` | `128` | Columns between two tiles |
| `merge` | `nms` | How boxes of overlapping tiles are merged: `nms` (non-maximum suppression) or `greedy_nmm` (greedy merging) |
| `map_min_confidence` | `0.10` | Minimum score to appear on the GCS map (the records keep every detection) |
| `preprocess` | `fixed_gray` | Detector input: `fixed_gray` (a fixed mapping of the raw dB) or `identity` (the display picture) |
| `near_range_mask_m` | `0.0` | `0.0` (off) or `0.75`: hides the first 0.75 m next to the boat |
| `calibration` | `identity` | Only identity exists today |
| `record_log` | `true` | Write the live record log |

Switch to YOLO without editing the file:

```bash
ros2 run blueboat_perception perception_node --ros-args \
    --params-file ~/ros2_ws/install/blueboat_perception/share/blueboat_perception/config/perception.yaml \
    -p detector:=yolo -p detector_model:=<best.pt>
```

YOLO11s and YOLO26s differ only by the weights file; the record names the model family.

## 5. Training and evaluating YOLO

The tools in `tools/` are not installed. Run them from the repository root. All results are **on
simulated data**. Split by world, never by image. Never tune on the test worlds.

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

# 4. train (--model yolo11s.pt or yolo26s.pt; --device 0 for a graphics card)
python3 tools/train_yolo.py --data $T/tiles128/data.yaml --model yolo11s.pt --name y11s-t128-s0

# 5. evaluate on val (choose the merge rule and threshold here), then once with --on test
python3 tools/evaluate_detector.py $DS/* --split split.yaml --on val --name y11s-t128-nms \
    --detector yolo --detector-model $M/y11s-t128-s0/weights/best.pt --merge nms
```

- Each step writes a JSON beside its output: settings, split, seed, versions.
- `train_yolo.py` sets the augmentation explicitly; `--no-flips` turns off the flips. Train with
  level and per-ping gain augmentation, because simulated pictures are brighter than real ones.
- Not decided yet: the merge rule (`nms` or `greedy_nmm`) and the 2 dB label visibility threshold.

Other tools: `tools/preprocessing_study.py` reproduces the preprocessing study (read-only), and
`tools/fit_detector_input.py` fits a new detector-input mapping on real data, under a new name.

## 6. Topics

| Topic | Type | Direction |
|---|---|---|
| `/sss_ai/seabed_image` | `sensor_msgs/Image` 32FC1 (raw dB) | in, from the GCS |
| `/sss_ai/seabed_image/png` | `sensor_msgs/Image` mono8 | in, from the GCS |
| `/sss_ai/seabed_image/metadata` | `std_msgs/String` (JSON) | in, from the GCS |
| `/sss_ai/detections` | `vision_msgs/Detection2DArray`, one per image | out, to the GCS map |
| `/sss_ai/image_records` | `std_msgs/String` (JSON, `image_record/1`) | out, to Belief and analysis |
| `/sss_ai/perception/status` | `std_msgs/String` (JSON), once per second | out |

Every field, delivery setting and parameter: [topics.md](topics.md).

## 7. Output files

All under `<root>/data/` (normally `~/ros2_ws/data/`). Nothing is overwritten: a taken name gets
`-2`, `-3`, and so on.

| Folder | Written by | Content |
|---|---|---|
| `Perception_data/live/<stream_id>.jsonl` | live node | One image record per line |
| `Perception_data/offline/<session>/` | offline command | `records.jsonl`, `detections.json`, `summary.json` |
| `Dataset_data/<name>/` | dataset mode | `images/`, `display/`, `labels/`, `raw/`, `manifest.json`, `data.yaml`, `ground_truth_bag/` |
| `Tile_data/`, `Perception_models/`, `Perception_eval/` | offline tools | Tile datasets, trained models, evaluations |

## 8. Folder layout

```
BlueBoat-Perception/
  blueboat_perception/            ROS 2 package
    blueboat_perception/core/     the pipeline without ROS: join, preprocess, detectors, tiling, merge, records, labels
    blueboat_perception/ros/      the node (perception_node.py) and the ground-truth subscriber
    blueboat_perception/cli/      offline.py, the offline runner
    config/perception.yaml        node defaults
    launch/perception_launch.py
    tests/
  tools/                          offline tools: datasets, training, evaluation, studies (not installed)
  docs/                           this guide, the development guide, design and contract documents
```
