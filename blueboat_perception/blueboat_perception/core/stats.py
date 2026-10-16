"""Latency statistics (bounded window, percentiles), ROS-free."""

from __future__ import annotations

from collections import deque
from typing import Deque, Dict, Optional

import numpy as np


class LatencyStats:
    def __init__(self, window: int = 2000) -> None:
        self._v: Deque[float] = deque(maxlen=int(window))
        self.count = 0

    def add(self, ms: float) -> None:
        self._v.append(float(ms))
        self.count += 1

    def summary(self) -> Dict[str, Optional[float]]:
        if not self._v:
            return {"n": 0, "p50": None, "p95": None, "max": None}
        a = np.asarray(self._v, np.float64)
        return {"n": int(a.size), "p50": float(np.percentile(a, 50)),
                "p95": float(np.percentile(a, 95)), "max": float(a.max())}
