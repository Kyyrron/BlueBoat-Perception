"""JoinBuffer: completion, timeout release, gaps, duplicates."""

from __future__ import annotations

import pytest

from blueboat_perception.core.join import JoinBuffer, split_key

K = "gcs-x/"


def test_complete_when_image_and_metadata_arrive_in_any_order():
    j = JoinBuffer(timeout_s=5.0)
    assert j.add("metadata", K + "0", {"m": 1}, 0.0) == []
    out = j.add("image", K + "0", "pix", 0.1)
    assert [r.status for r in out] == ["complete"] and out[0].last_seen == 0.1
    out = j.add("image", K + "1", "pix", 0.2) + j.add("metadata", K + "1", {}, 0.3)
    assert [r.key for r in out] == [K + "1"]
    st = j.streams["gcs-x"]
    assert (st.completed, st.gaps, st.last_image_id) == (2, 0, 1)


def test_dataset_mode_waits_for_the_png_then_releases_without_it():
    j = JoinBuffer(timeout_s=1.0, need_png=True)
    j.add("image", K + "0", "pix", 0.0)
    assert j.add("metadata", K + "0", {}, 0.0) == []
    assert [r.status for r in j.poll(1.5)] == ["no_png"]
    j.add("image", K + "1", "pix", 2.0)
    j.add("metadata", K + "1", {}, 2.0)
    assert [r.status for r in j.add("png", K + "1", "png", 2.1)] == ["complete"]


def test_timeouts_release_partials_and_drop_unplaceable_pixels():
    j = JoinBuffer(timeout_s=1.0)
    j.add("metadata", K + "0", {}, 0.0)            # pixels lost
    j.add("image", K + "1", "pix", 0.0)            # metadata lost
    assert j.poll(0.5) == []
    out = j.poll(1.0)
    assert [(r.key, r.status) for r in out] == [(K + "0", "no_pixels")]
    st = j.streams["gcs-x"]
    assert st.partial_no_pixels == 1 and st.dropped_no_metadata == 1
    assert j.pending == 0


def test_gaps_and_duplicates_are_counted():
    j = JoinBuffer()
    for i in (0, 1, 4):
        j.add("image", K + str(i), "p", 0.0)
        j.add("metadata", K + str(i), {}, 0.0)
    j.add("image", K + "4", "p", 0.0)              # again, after release
    j.add("image", K + "5", "p", 0.0)
    j.add("image", K + "5", "p", 0.0)              # twice while pending
    st = j.streams["gcs-x"]
    assert st.gaps == 2 and st.duplicates == 2 and st.completed == 3
    assert st.late_parts == 0


def test_a_png_after_release_is_late_not_duplicate():
    """Normal mode does not await the PNG; the three topics are unordered.
    Measured in the 2026-10-16 live run: 2 of 47 PNGs came after the metadata."""
    j = JoinBuffer(need_png=False)
    j.add("image", K + "0", "p", 0.0)
    assert [r.status for r in j.add("metadata", K + "0", {}, 0.0)] == ["complete"]
    j.add("png", K + "0", "png", 0.01)
    st = j.streams["gcs-x"]
    assert (st.late_parts, st.duplicates) == (1, 0)
    j.add("png", K + "0", "png", 0.02)               # now it IS a repeat
    assert (st.late_parts, st.duplicates) == (1, 1)


def test_streams_are_counted_separately_and_flush_releases_everything():
    j = JoinBuffer(timeout_s=10.0)
    j.add("metadata", "a/0", {}, 0.0)
    j.add("metadata", "b/0", {}, 0.0)
    out = j.flush(0.1)
    assert sorted(r.key for r in out) == ["a/0", "b/0"]
    assert set(j.stats_json()) == {"a", "b"}


def test_keys():
    assert split_key("gcs-20261016T000000Z-abcd/12") == ("gcs-20261016T000000Z-abcd", 12)
    assert split_key("session:x/y/3") == ("session:x/y", 3)
    with pytest.raises(ValueError):
        split_key("12")
