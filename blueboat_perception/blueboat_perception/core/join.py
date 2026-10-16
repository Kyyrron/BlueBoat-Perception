"""Join the three parts of one seabed image by key, and count every loss.

The GCS publishes an image as three samples with the same key
``"<stream_id>/<image_id>"``: the 32FC1 pixels, the mono8 PNG and the
metadata JSON. ROS gives no atomicity across topics, so the node joins:

* a key is **complete** when pixels and metadata are present (and the PNG,
  when ``need_png`` -- dataset mode);
* a key still incomplete ``timeout_s`` after its first part is released
  partially: metadata without pixels -> status ``no_pixels`` (a record
  with ``detections: null``, i.e. *unknown*); pixels without metadata
  cannot be georeferenced -> counted ``dropped_no_metadata``;
* a part received twice for one key is a ``duplicate``; a part that arrives
  after its key was released without it (in normal mode the PNG is not
  awaited, and the three topics carry no ordering guarantee) is a
  ``late_part`` -- not a loss and not a duplicate;
* per stream, a released ``image_id`` that skips ids counts the skipped
  ones as ``gaps``.

Time is passed in by the caller (``now``, seconds, any monotonic clock),
so the buffer is deterministic and testable.
"""

from __future__ import annotations

from collections import OrderedDict, deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional, Set, Tuple

PARTS = ("image", "png", "metadata")


def split_key(key: str) -> Tuple[str, int]:
    """``"<stream_id>/<image_id>"`` -> (stream_id, image_id)."""
    sid, _, iid = str(key).rpartition("/")
    if not sid:
        raise ValueError(f"not an image key: {key!r}")
    return sid, int(iid)


@dataclass
class StreamStats:
    completed: int = 0
    partial_no_pixels: int = 0
    partial_no_png: int = 0
    dropped_no_metadata: int = 0
    duplicates: int = 0
    late_parts: int = 0
    gaps: int = 0
    last_image_id: Optional[int] = None

    def to_json(self) -> dict:
        return dict(self.__dict__)


@dataclass
class Released:
    key: str
    stream_id: str
    image_id: int
    status: str                      # complete | no_pixels | no_png
    parts: Dict[str, Any]
    first_seen: float
    last_seen: float                 # arrival of the last part received
    released_at: float

    @property
    def metadata(self):
        return self.parts.get("metadata")


@dataclass
class _Pending:
    first_seen: float
    last_seen: float = 0.0
    parts: Dict[str, Any] = field(default_factory=dict)


class JoinBuffer:
    def __init__(self, timeout_s: float = 5.0, need_png: bool = False,
                 remember: int = 4096) -> None:
        self.timeout_s = float(timeout_s)
        self.need_png = bool(need_png)
        self._pending: "OrderedDict[str, _Pending]" = OrderedDict()
        #: released key -> the parts it had (or received since)
        self._released: Dict[str, Set[str]] = {}
        self._released_order: Deque[str] = deque()
        self._remember = int(remember)
        self.streams: Dict[str, StreamStats] = {}

    # ---- input -------------------------------------------------------------------
    def add(self, part: str, key: str, payload: Any, now: float) -> List[Released]:
        if part not in PARTS:
            raise ValueError(f"unknown part {part!r}")
        sid, _ = split_key(key)
        st = self.streams.setdefault(sid, StreamStats())
        if key in self._released:
            seen = self._released[key]
            if part in seen:
                st.duplicates += 1
            else:
                st.late_parts += 1
                seen.add(part)
            return self.poll(now)
        p = self._pending.get(key)
        if p is None:
            p = self._pending[key] = _Pending(first_seen=now)
        if part in p.parts:
            st.duplicates += 1
        p.parts[part] = payload
        p.last_seen = now
        out = []
        if self._complete(p):
            out.append(self._release(key, "complete", now))
        out.extend(self.poll(now))
        return out

    def poll(self, now: float) -> List[Released]:
        """Release every key older than the timeout, oldest first."""
        out = []
        for key in list(self._pending):
            p = self._pending[key]
            if now - p.first_seen < self.timeout_s:
                continue
            has_meta = "metadata" in p.parts
            has_pix = "image" in p.parts
            if not has_meta:
                sid, _ = split_key(key)
                self.streams[sid].dropped_no_metadata += 1
                self._remember_key(key, set(p.parts))
                del self._pending[key]
            elif not has_pix:
                out.append(self._release(key, "no_pixels", now))
            else:                                   # pixels + metadata, no PNG
                out.append(self._release(key, "no_png", now))
        return out

    def flush(self, now: float) -> List[Released]:
        """Release everything pending (shutdown), as if timed out."""
        for p in self._pending.values():
            p.first_seen = min(p.first_seen, now - self.timeout_s)
        return self.poll(now)

    @property
    def pending(self) -> int:
        return len(self._pending)

    # ---- internals ----------------------------------------------------------------
    def _complete(self, p: _Pending) -> bool:
        need = ("image", "metadata") + (("png",) if self.need_png else ())
        return all(k in p.parts for k in need)

    def _remember_key(self, key: str, parts: Set[str]) -> None:
        self._released[key] = parts
        self._released_order.append(key)
        while len(self._released_order) > self._remember:
            self._released.pop(self._released_order.popleft(), None)

    def _release(self, key: str, status: str, now: float) -> Released:
        p = self._pending.pop(key)
        sid, iid = split_key(key)
        st = self.streams[sid]
        if status == "complete":
            st.completed += 1
        elif status == "no_pixels":
            st.partial_no_pixels += 1
        elif status == "no_png":
            st.partial_no_png += 1
        if st.last_image_id is not None and iid > st.last_image_id + 1:
            st.gaps += iid - st.last_image_id - 1
        if st.last_image_id is None or iid > st.last_image_id:
            st.last_image_id = iid
        self._remember_key(key, set(p.parts))
        return Released(key=key, stream_id=sid, image_id=iid, status=status,
                        parts=p.parts, first_seen=p.first_seen,
                        last_seen=p.last_seen, released_at=now)

    def stats_json(self) -> Dict[str, dict]:
        return {sid: st.to_json() for sid, st in sorted(self.streams.items())}
