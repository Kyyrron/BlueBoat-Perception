"""Confidence calibration hook.

Default ``Identity``: returns the raw score and says ``calibrated: false``.
A future calibrator (temperature scaling, Guo et al. 2017 §4.2 eq. 9;
Platt or isotonic, §4.1) is fitted on a **held-out set from the same
distribution** (§4) -- separately for sim and real deployments -- and
evaluated with reliability diagrams and ECE/MCE (§2 eq. 3, 5). Guo et al.
treat classification; applying it to detector scores is an extension to
validate, not to assume.

``project_synthesis.md`` §8.2 ⚠: raw scores must not feed a Bayesian
belief grid as probabilities -- every detection carries
``confidence_raw``, ``confidence`` and ``calibrated``, and Belief uses
``confidence`` as a probability only when ``calibrated`` is true.
"""

from __future__ import annotations

from typing import Any, Dict, Optional, Protocol


class Calibrator(Protocol):
    name: str
    params: Dict[str, Any]
    calibrated: bool

    def calibrate(self, score_raw: float, logit: Optional[float],
                  class_id: int) -> float: ...


class Identity:
    name = "identity"
    calibrated = False

    def __init__(self) -> None:
        self.params: Dict[str, Any] = {}

    def calibrate(self, score_raw: float, logit: Optional[float],
                  class_id: int) -> float:
        return float(score_raw)


CALIBRATORS = {"identity": Identity}


def make_calibrator(name: str) -> Calibrator:
    try:
        return CALIBRATORS[name]()
    except KeyError:
        raise ValueError(f"unknown calibrator {name!r}; known: {sorted(CALIBRATORS)}")
