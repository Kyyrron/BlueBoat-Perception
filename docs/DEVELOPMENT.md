# BlueBoat-Perception — development guide

How to work on the detector module: the AI coding assistant setup, the tests, the rules and what
to do next. For using the module, see the [README](../README.md) and the [user guide](USER_GUIDE.md).

## 1. Install Claude Code

The project is developed with [Claude Code](https://docs.claude.com/en/docs/claude-code/overview),
an AI coding assistant that runs in the terminal.

```bash
curl -fsSL https://claude.ai/install.sh | bash        # native installer
# or, with Node.js 18 or newer:
npm install -g @anthropic-ai/claude-code
```

Then start it and log in with your Anthropic account:

```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar               # the whole project: cross-module rules included
claude
# or only this module:
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-Perception && claude
```

Starting from the superproject is better: Claude then also reads the project rules, the simulator's
validity conditions (which bind every dataset) and the shared skills (test commands, file formats).

## 2. What Claude reads in this module

| File | Role |
|---|---|
| `.claude/CLAUDE.md` | Loaded at start: the pipeline, the interface, the non-negotiable rules, the detector input and labels, the commands |
| `.claude/gates.md` | The tests and their expected counts; loaded with the tests by `.claude/rules/gates.md` |
| `.claude/TODO.md` | Open work |

No hook or settings file is needed: nothing in `.claude/` is ignored by git, so a fresh clone is
complete.

## 3. What to set up after a fresh clone

- **pytest in the system Python** (`/usr/bin/python3 -m pip install --user --break-system-packages pytest`
  if it is missing). The workspace venv has no pytest.
- **Optional, for the tests that need them**: ROS sourced (node and recording tests), the field
  corpus under `~/ros2_ws/data/SSS_data` (or `$BLUEBOAT_DATA_DIR/data/SSS_data`), Ultralytics (real
  YOLO tests), and SSS-Dataset-Aug-Studio beside this repository (its sidecar tests). Each group
  skips cleanly when missing.

## 4. Tests

From `BlueBoat-Perception/blueboat_perception/`, with the system Python and **without** sourcing the
workspace:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest
```

Also run these when a change could alter the dummy detector's records (they share this module's
contract):

- BlueBoat-SSS: `tests/test_perception_golden.py` and `tests/test_perception_bridge.py`;
- BlueBoat-MCS: `QT_QPA_PLATFORM=offscreen python3 smoke_test.py` (block `perception ok`).

There is no linter or type checker.

## 5. Rules for changes

- **The core stays free of ROS and deterministic**: no rclpy, Qt, OpenCV, PyTorch or Ultralytics
  imported at load time in `core/`, no randomness, no wall clock.
- **One record per image**, with or without detections; every loss is counted.
- **Offline equals live**: a core change must keep the records byte-identical.
- **The simulator's ground truth reaches the dataset labels and nothing else.** The detection path
  never imports the label code.
- **Never write into a recording session or another module's folder.** Needs go to
  `docs/REQUESTS.md`.
- **The detector input never changes the original data**, and a fixed mapping used for training is
  never refitted in place: a refit is a new file with a new name, fitted on real data.
- **No unvalidated image processing** (despeckling, histogram equalisation...) without a controlled
  comparison.
- **One tiling function for training and inference.**
- **Split by world, never by image; never tune on the test worlds.** Every model carries its
  settings, dataset, split, seed and versions in a JSON beside it.
- **Cite nothing from memory**: every non-software choice traces to `docs/REFERENCES.md`.
- **Never rename a topic**; the image delivery settings must equal the GCS publishers'.
- AI sessions never commit, push or tag: the owner does.

## 6. Next steps

From `.claude/TODO.md`, in plain words:

1. **Build the simulated datasets and run the detector comparison**: YOLO11s and YOLO26s, full
   pictures against 256-pixel tiles with two strides, the two merge rules; choose on the validation
   worlds, then report once on the test worlds.
2. **Set the label visibility threshold** (2 dB today, provisional) from the first datasets'
   statistics (`tools/draw_labels.py --stats`).
3. **Find why ray-cast label boxes sit about 0.1 m ahead of the echo** before training on ray-cast
   datasets.
4. **Fit a calibration** once weights exist, on held-out simulated worlds, so that scores become
   probabilities (`calibrated: true`).
5. **Warn when a rebuilt session cannot be labelled**: today a session rebuilt from its `.svlog`
   gets empty labels silently, because its clock is about 6 s off the ground-truth recording.
