"""Where Perception writes: the project's run-data root rule.

Same precedence as BlueBoat-Control's ``custom_functions.data_root``
(skill ``file-interfaces``), first match wins:

1. ``explicit`` -- the node's / CLI's ``data_dir``, when non-empty;
2. ``$BLUEBOAT_DATA_DIR``;
3. the sourced ROS workspace -- parent of the first ``$COLCON_PREFIX_PATH`` entry;
4. the process working directory.

Artifacts go under ``<root>/data/`` -- normally ``~/ros2_ws/data/``:
``Perception_data/live/``, ``Perception_data/offline/``, ``Dataset_data/``.
Names are claimed exclusively (``-2``, ``-3``, ... on a clash), never
overwritten (CM-7).
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional


def data_root(explicit: str = "") -> Path:
    if explicit:
        return Path(os.path.abspath(os.path.expanduser(explicit)))
    env = os.environ.get("BLUEBOAT_DATA_DIR", "")
    if env:
        return Path(os.path.abspath(os.path.expanduser(env)))
    prefix = os.environ.get("COLCON_PREFIX_PATH", "")
    first = prefix.split(os.pathsep)[0] if prefix else ""
    if first:
        return Path(os.path.dirname(os.path.abspath(first)))
    return Path(os.getcwd())


def perception_dir(root: Path, *parts: str) -> Path:
    return Path(root, "data", "Perception_data", *parts)


def dataset_root(root: Path) -> Path:
    return Path(root) / "data" / "Dataset_data"


def sss_data_dir(root: Path) -> Path:
    return Path(root) / "data" / "SSS_data"


def claim_dir(parent: Path, name: str) -> Path:
    """Create ``parent/name`` exclusively; ``name-2``, ``name-3``, ... on a clash."""
    parent = Path(parent)
    parent.mkdir(parents=True, exist_ok=True)
    k = 1
    while True:
        cand = parent / (name if k == 1 else f"{name}-{k}")
        try:
            cand.mkdir()
            return cand
        except FileExistsError:
            k += 1


def claim_file(parent: Path, stem: str, suffix: str) -> Path:
    """Create an empty ``parent/stem<suffix>`` exclusively (O_EXCL)."""
    parent = Path(parent)
    parent.mkdir(parents=True, exist_ok=True)
    k = 1
    while True:
        cand = parent / (f"{stem}{suffix}" if k == 1 else f"{stem}-{k}{suffix}")
        try:
            fd = os.open(cand, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
            os.close(fd)
            return cand
        except FileExistsError:
            k += 1


def is_inside(path: Path, parent: Path) -> bool:
    try:
        Path(path).resolve().relative_to(Path(parent).resolve())
        return True
    except ValueError:
        return False


def safe_name(text: Optional[str]) -> str:
    """A file-name-safe version of a stream id or note."""
    out = "".join(c if (c.isalnum() or c in "._-") else "_" for c in (text or ""))
    return out.strip("._") or "unnamed"
