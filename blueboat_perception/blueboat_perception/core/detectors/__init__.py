"""Detector plug-ins. ``make_detector(name, **kw)`` builds one by name."""

from .base import Detector, RawDetection
from .dummy import DummyCenter
from .yolo import YoloDetector

DETECTORS = {DummyCenter.name: DummyCenter, YoloDetector.name: YoloDetector}


def make_detector(name: str, **kwargs) -> Detector:
    try:
        cls = DETECTORS[name]
    except KeyError:
        raise ValueError(f"unknown detector {name!r}; known: {sorted(DETECTORS)}")
    return cls(**kwargs)


__all__ = ["Detector", "RawDetection", "DummyCenter", "YoloDetector",
           "DETECTORS", "make_detector"]
