# BlueBoat-Perception — gates

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here.*

## 6. Gates
- **Perception pytest** (no lint or type checker configured): **170 pass / 16 skip** with
  system python and no ROS, corpus, torch or sibling repos (2026-11-15; before that date
  `tests/test_corpus.py` failed at collection when no corpus was present). On the laptop with ROS
  on the path and the corpus: **204 pass / 28 skip**. Skips: ROS node
  and bag tests, corpus tests (`$BLUEBOAT_DATA_DIR/data/SSS_data` or
  `~/ros2_ws/data/SSS_data`), real-Ultralytics tests, Aug Studio sidecar tests (need
  pydantic and the Aug Studio beside this module:
  `SSS-Dataset-Aug-Studio/.venv/bin/python -m pytest tests/test_dataset.py`). On the
  laptop the node and corpus tests run too. A new session's pictures (convention 3 since
  2026-11-15) must stay bit-exact in `test_corpus.py`. To test a session without placing it under
  `~/ros2_ws/data/SSS_data`, set `BLUEBOAT_DATA_DIR` to a root holding `SSS_data/<folder>`.
- **SSS** shares this contract: run its `tests/test_perception_golden.py` and
  `tests/test_perception_bridge.py` on the laptop (they need libEGL for PySide6) whenever
  the dummy's records could change.
- **MCS:** `QT_QPA_PLATFORM=offscreen python3 smoke_test.py` (`perception ok`), `ruff check .`.
