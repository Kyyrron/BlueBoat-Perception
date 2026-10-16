"""The record log: one canonical record per line, one file per stream.

``<root>/data/Perception_data/live/<stream_id>.jsonl`` (claimed
exclusively; a second run of the same stream id gets ``-2``). Each line is
written and flushed whole, so a killed node leaves every finished record
readable. The offline CLI writes the same format, which is what the
live-vs-offline comparison reads.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional, TextIO

from .dataroot import claim_file, safe_name
from .records import canonical_json


class RecordLog:
    def __init__(self, directory: Path) -> None:
        self.dir = Path(directory)
        self._files: Dict[str, TextIO] = {}
        self.paths: Dict[str, Path] = {}

    def path_for(self, stream_id: str) -> Optional[Path]:
        return self.paths.get(stream_id)

    def write(self, record: dict) -> Path:
        sid = record["stream_id"]
        f = self._files.get(sid)
        if f is None:
            path = claim_file(self.dir, safe_name(sid), ".jsonl")
            f = self._files[sid] = open(path, "a", encoding="utf-8")
            self.paths[sid] = path
        f.write(canonical_json(record) + "\n")
        f.flush()
        return self.paths[sid]

    def close(self) -> None:
        for f in self._files.values():
            try:
                f.close()
            except OSError:
                pass
        self._files.clear()
