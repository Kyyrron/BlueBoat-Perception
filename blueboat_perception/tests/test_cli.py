"""The offline CLI over a session; refusal inside the session; --compare."""

from __future__ import annotations

import json

from blueboat_perception.cli import offline
from blueboat_perception.core.records import canonical_json, read_jsonl


def test_offline_run_writes_records_detections_summary(tmp_path, synthetic_session):
    rc = offline.main([str(synthetic_session), "--data-dir", str(tmp_path / "root")])
    assert rc == 0
    out = tmp_path / "root" / "data" / "Perception_data" / "offline" / synthetic_session.name
    recs = list(read_jsonl(out / "records.jsonl"))
    assert [r["image_id"] for r in recs] == [0, 1, 2, 3]
    assert all(r["source"] == "offline" for r in recs)
    dets = json.loads((out / "detections.json").read_text())
    assert len(dets) == 4
    summ = json.loads((out / "summary.json").read_text())
    assert summ["images"] == 4 and summ["status"] == {"ran": 4}
    # a second run claims a new folder
    assert offline.main([str(synthetic_session), "--data-dir", str(tmp_path / "root")]) == 0
    assert (out.parent / f"{synthetic_session.name}-2" / "records.jsonl").is_file()


def test_refuses_an_output_inside_the_session(tmp_path, synthetic_session):
    assert offline.main([str(synthetic_session), "--out",
                         str(synthetic_session / "perception")]) == 2
    assert not (synthetic_session / "perception").exists()


def test_compare_against_a_live_log(tmp_path, synthetic_session):
    root = tmp_path / "root"
    assert offline.main([str(synthetic_session), "--data-dir", str(root)]) == 0
    out = root / "data" / "Perception_data" / "offline" / synthetic_session.name
    live = tmp_path / "live.jsonl"
    with open(live, "w", encoding="utf-8") as f:
        for r in read_jsonl(out / "records.jsonl"):
            r.update(source="live", provenance={"latency_ms": {"total": 1.0}})
            f.write(canonical_json(r) + "\n")
    assert offline.main([str(synthetic_session), "--data-dir", str(root),
                         "--compare", str(live)]) == 0
    summ = json.loads((out.parent / f"{synthetic_session.name}-2" / "summary.json").read_text())
    assert summ["compare"]["identical"] == summ["compare"]["compared"] == 4
    # a tampered live record is reported and fails the run
    lines = live.read_text().splitlines()
    rec = json.loads(lines[1])
    rec["track"]["x_m"][0] += 1e-9
    lines[1] = canonical_json(rec)
    live.write_text("\n".join(lines) + "\n")
    assert offline.main([str(synthetic_session), "--data-dir", str(root),
                         "--compare", str(live)]) == 1


def test_dataset_from_a_session(tmp_path, synthetic_session):
    root = tmp_path / "root"
    assert offline.main([str(synthetic_session), "--data-dir", str(root),
                         "--dataset", "offline-ds"]) == 0
    d = root / "data" / "Dataset_data" / "offline-ds"
    assert len(list((d / "images").glob("*.png"))) == 4
    assert json.loads((d / "manifest.json").read_text())["mode"] == "offline"
