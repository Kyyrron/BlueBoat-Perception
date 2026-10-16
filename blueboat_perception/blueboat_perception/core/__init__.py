"""ROS-free, deterministic core of BlueBoat-Perception.

Imports numpy and the standard library only (no rclpy, no Qt, no OpenCV;
the dataset writer loads OpenCV lazily to encode a PNG). A test sweeps the
imports. Everything here is a pure function of its inputs and configuration:
no randomness, no wall clock, no global state.
"""

from .image import Georef, RowArrays, SeabedImageInput
from .pipeline import Pipeline, PipelineConfig

__all__ = ["Georef", "RowArrays", "SeabedImageInput", "Pipeline", "PipelineConfig"]
