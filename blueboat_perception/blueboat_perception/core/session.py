"""Offline reader: the seabed images of a recorded session, in id order.

Accepts any folder the GCS writes seabed images into:
* a recording session ``<data>/SSS_data/sessions/<stamp>/`` (its
  ``seabed_images/``), or a merged session;
* a replay window "Save pictures from the log" folder
  (``seabed_images_<log>/``);
* the ``seabed_images/`` folder itself.

Each image is built from ``metadata/seabed_XXXXX.json`` + its
``_world.npz`` (the raw float dB) + the PNG path, read-only (CM-7, SSS
NC #6). ``stream_id`` comes from the metadata (schema >= 5), else
``session:<folder name>``.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, List, Optional, Union

from .image import SeabedImageInput

_STEM = re.compile(r"^seabed_(\d+)\.json$")


@dataclass(frozen=True)
class ImageFiles:
    image_id: int
    json: Path
    npz: Path
    png: Path


def images_dir(path: Union[str, Path]) -> Path:
    """The folder holding ``metadata/seabed_*.json`` under ``path``."""
    path = Path(path).expanduser().resolve()
    for cand in (path, path / "seabed_images"):
        if (cand / "metadata").is_dir() and any((cand / "metadata").glob("seabed_*.json")):
            return cand
    raise FileNotFoundError(f"no seabed images under {path} "
                            f"(expected metadata/seabed_*.json or seabed_images/metadata/)")


def list_images(path: Union[str, Path]) -> List[ImageFiles]:
    d = images_dir(path)
    out = []
    for j in (d / "metadata").iterdir():
        m = _STEM.match(j.name)
        if not m:
            continue
        stem = j.stem
        out.append(ImageFiles(int(m.group(1)), j,
                              d / "metadata" / f"{stem}_world.npz",
                              d / f"{stem}.png"))
    return sorted(out, key=lambda f: f.image_id)


def session_name(path: Union[str, Path]) -> str:
    """The session's name: the folder above ``seabed_images/`` when there is one."""
    d = images_dir(path)
    return d.parent.name if d.name == "seabed_images" else d.name


def iter_session(path: Union[str, Path], load_png: bool = False,
                 stream_id: Optional[str] = None) -> Iterator[SeabedImageInput]:
    fallback = stream_id or f"session:{session_name(path)}"
    for f in list_images(path):
        yield SeabedImageInput.from_session_files(
            f.json, f.npz, f.png, stream_id=fallback, load_png=load_png)
