"""The core needs no ROS, Qt or OpenCV. Ground truth reaches only the dataset
labels: one ROS file names its topics, and the detection path never imports the
labeller (owner's decision, 2026-10-16)."""

from __future__ import annotations

import ast
import importlib
import pkgutil

from conftest import PY_PKG

#: Never named anywhere in the online sources, comments included.
GROUND_TRUTH = ("/side_scan_sonar/ground_truth", "ground_truth/contacts",
                "ground_truth/sensor", "/sim/sea_state")
#: Never named in a string literal of the online code (docstrings may
#: mention them to say why they are not read).
SENSOR_TOPICS = ("/side_scan_sonar/", "/sss_processor/", "/blueboat/", "/mavros/")


def _code_strings(tree):
    doc = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef,
                             ast.AsyncFunctionDef)) and node.body:
            first = node.body[0]
            if isinstance(first, ast.Expr) and isinstance(first.value, ast.Constant):
                doc.add(id(first.value))
    return [n.value for n in ast.walk(tree) if isinstance(n, ast.Constant)
            and isinstance(n.value, str) and id(n) not in doc]


def test_core_imports_without_ros_qt_or_opencv(no_ros_qt_cv):
    import blueboat_perception.core as core
    names = [m.name for m in pkgutil.walk_packages(core.__path__, core.__name__ + ".")]
    assert len(names) >= 14
    for n in names:
        importlib.import_module(n)
    from blueboat_perception.cli import offline  # noqa: F401  (the CLI too)


#: The one online file allowed to name the simulator's ground-truth topics.
GROUND_TRUTH_FILE = PY_PKG / "ros" / "ground_truth.py"
#: The detection path: never imports the labeller.
DETECTION_PATH = ("pipeline.py", "preprocess.py", "calibration.py", "records.py",
                  "uncertainty.py", "geometry.py", "image.py", "join.py")


def test_online_sources_never_name_a_ground_truth_or_sensor_topic():
    for path in list((PY_PKG / "core").rglob("*.py")) + list((PY_PKG / "ros").rglob("*.py")):
        if path == GROUND_TRUTH_FILE:
            continue
        text = path.read_text(encoding="utf-8")
        for t in GROUND_TRUTH:
            assert t not in text, f"{path.name} mentions {t}"
        for lit in _code_strings(ast.parse(text)):
            for t in SENSOR_TOPICS:
                assert t not in lit, f"{path.name} names {t} in {lit!r}"


def test_the_node_subscribes_to_exactly_the_three_image_topics():
    tree = ast.parse((PY_PKG / "ros" / "perception_node.py").read_text(encoding="utf-8"))
    subs = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
            and getattr(n.func, "attr", "") == "create_subscription"]
    topics = sorted(ast.unparse(s.args[1]) for s in subs)
    assert topics == ["p['topic_image']", "p['topic_metadata']", "p['topic_png']"]


def _imports(path):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    names = []
    for n in ast.walk(tree):
        if isinstance(n, ast.ImportFrom):
            names.append(n.module or "")
            names += [a.name for a in n.names]
        elif isinstance(n, ast.Import):
            names += [a.name for a in n.names]
    return names


def test_the_ground_truth_file_names_only_the_two_label_topics():
    tree = ast.parse(GROUND_TRUTH_FILE.read_text(encoding="utf-8"))
    topics = [lit for lit in _code_strings(tree) if lit.startswith("/")]
    assert sorted(topics) == ["/side_scan_sonar/ground_truth/contacts",
                              "/side_scan_sonar/ground_truth/sensor"]
    subs = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
            and getattr(n.func, "attr", "") == "create_subscription"]
    assert len(subs) == 2
    assert "/sim/sea_state" not in GROUND_TRUTH_FILE.read_text(encoding="utf-8")


def test_the_detection_path_never_imports_the_labeller():
    core = PY_PKG / "core"
    paths = [core / n for n in DETECTION_PATH] + list((core / "detectors").rglob("*.py"))
    for path in paths:
        names = _imports(path)
        assert not any("sim_labels" in n or "ground_truth" in n for n in names), path.name


def test_the_node_reads_ground_truth_only_from_its_dataset_set_up():
    tree = ast.parse((PY_PKG / "ros" / "perception_node.py").read_text(encoding="utf-8"))
    calls = {}
    for fn in ast.walk(tree):
        if isinstance(fn, ast.FunctionDef):
            for n in ast.walk(fn):
                if isinstance(n, ast.Call) and ast.unparse(n.func) == "ground_truth.subscribe":
                    calls.setdefault(fn.name, 0)
                    calls[fn.name] += 1
    assert calls == {"_open_labels": 1}
    callers = [fn.name for fn in ast.walk(tree) if isinstance(fn, ast.FunctionDef)
               and any(isinstance(n, ast.Call) and ast.unparse(n.func) == "self._open_labels"
                       for n in ast.walk(fn))]
    assert callers == ["_open_dataset"]
