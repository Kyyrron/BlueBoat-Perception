"""The core needs no ROS, Qt or OpenCV; online code never reads ground truth."""

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


def test_online_sources_never_name_a_ground_truth_or_sensor_topic():
    for path in list((PY_PKG / "core").rglob("*.py")) + list((PY_PKG / "ros").rglob("*.py")):
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
