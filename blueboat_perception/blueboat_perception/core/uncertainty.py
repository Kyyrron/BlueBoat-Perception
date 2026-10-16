"""Localisation uncertainty of a detection -- model ``geometric_v1``.

The covariance is built in the boat frame (across = the look direction,
along = the heading) and rotated into the world frame:

* ``σ_across² = pitch²/12 + ((h/g)·σ_h)² + σ_nav² + extent_across²/12``:
  bin quantisation; the altitude error carried through the ground range
  ``g = sqrt(s² − h²)`` (Burguera & Oliver 2016 eq. 2: ∂g/∂h = −h/g);
* ``σ_along² = (s·sin θh)²/12 + row_pitch²/12 + (g·σ_ψ)² + σ_nav²
  + extent_along²/12``: the horizontal beam footprint ``s·θh`` (Blondel
  2009 eq. 2.10; Sethuraman et al. 2024 eq. 1, θh = 0.5° for the Omniscan
  450 per ``project_synthesis.md`` §1) taken as uniform; the row
  quantisation; the heading error at ground range ``g``.

``σ_h`` (0.10 m), ``σ_ψ`` (2°) and ``σ_nav`` (1.0 m) are **assumptions,
not measurements** (``assumed: true`` in every record) until they are
measured against USBL / RTK ground truth (``project_synthesis.md`` §8.4).
``g`` is floored at one bin pitch so a detection at the nadir does not
divide by zero. The flat-floor misplacement of tall objects
(``PREPROCESSING_STUDY.md`` §1.2) is a bias, noted there, not added here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List

MODEL = "geometric_v1"


@dataclass(frozen=True)
class UncertaintyParams:
    sigma_h_m: float = 0.10
    sigma_heading_rad: float = math.radians(2.0)
    sigma_nav_m: float = 1.0
    beam_azimuth_rad: float = math.radians(0.5)

    def to_json(self) -> dict:
        return {"model": MODEL, "sigma_h_m": self.sigma_h_m,
                "sigma_heading_rad": self.sigma_heading_rad,
                "sigma_nav_m": self.sigma_nav_m,
                "beam_azimuth_rad": self.beam_azimuth_rad, "assumed": True}


def covariance_xy(p: UncertaintyParams, *, slant: float, h: float, pitch: float,
                  row_pitch: float, look_rad: float, heading_rad: float,
                  extent_across_m: float = 0.0,
                  extent_along_m: float = 0.0) -> List[List[float]]:
    """2x2 world covariance [[σxx, σxy], [σxy, σyy]] [m²]."""
    h = max(float(h), 0.0)
    g = math.sqrt(max(slant * slant - h * h, 0.0))
    g_eff = max(g, pitch, 1e-6)
    var_across = (pitch * pitch / 12.0 + ((h / g_eff) * p.sigma_h_m) ** 2
                  + p.sigma_nav_m ** 2 + extent_across_m ** 2 / 12.0)
    var_along = ((slant * math.sin(p.beam_azimuth_rad)) ** 2 / 12.0
                 + row_pitch * row_pitch / 12.0
                 + (g * p.sigma_heading_rad) ** 2 + p.sigma_nav_m ** 2
                 + extent_along_m ** 2 / 12.0)
    ca, sa = math.cos(look_rad), math.sin(look_rad)
    cl, sl = math.cos(heading_rad), math.sin(heading_rad)
    sxx = var_across * ca * ca + var_along * cl * cl
    syy = var_across * sa * sa + var_along * sl * sl
    sxy = var_across * ca * sa + var_along * cl * sl
    return [[sxx, sxy], [sxy, syy]]
