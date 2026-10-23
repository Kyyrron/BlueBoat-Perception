"""Across-track tiling (core/tiling.py, decided 2026-10-23)."""

from __future__ import annotations

import numpy as np
import pytest

from blueboat_perception.core.image import SeabedImageInput
from blueboat_perception.core.preprocess import FixedGray
from blueboat_perception.core.tiling import (TilingConfig, cut_tiles, tile_box_to_picture,
                                             tile_starts, to_inclusive_px)
from conftest import corpus_image_dirs


def test_the_prompt_example_gives_18_tiles():
    s = tile_starts(2400, 256, 128)
    assert len(s) == 18 and s[:2] == (0, 128) and s[-2:] == (2048, 2144)
    assert tile_starts(2402, 256, 128)[-1] == 2146


@pytest.mark.parametrize("W", [256, 257, 383, 384, 1200, 1202, 1600, 2400, 2401, 2402])
@pytest.mark.parametrize("S", [128, 64])
def test_every_column_is_covered_and_the_last_tile_is_flush(W, S):
    s = tile_starts(W, 256, S)
    covered = np.zeros(W, int)
    for c0 in s:
        assert 0 <= c0 and c0 + 256 <= W
        covered[c0:c0 + 256] += 1
    assert covered.min() >= 1
    assert s[-1] + 256 == W
    assert list(s) == sorted(set(s))
    # regular stride except possibly the flush tile
    d = np.diff(s)
    assert (d[:-1] == S).all() and 0 < d[-1] <= S if len(d) else True


@pytest.mark.parametrize("W", [1200, 1202, 2400, 2402])
def test_any_object_up_to_129_px_lies_whole_in_some_tile(W):
    s = tile_starts(W, 256, 128)
    for a in range(0, W - 129 + 1, 7):
        b = a + 128                          # inclusive, 129 px
        assert any(c0 <= a and b <= c0 + 255 for c0 in s)


def test_narrow_and_exact_widths():
    assert tile_starts(256, 256, 128) == (0,)
    assert tile_starts(100, 256, 128) == (0,)
    assert tile_starts(0, 256, 128) == ()


def test_config_validates():
    with pytest.raises(ValueError):
        TilingConfig(stride=0)
    with pytest.raises(ValueError):
        TilingConfig(stride=300)
    with pytest.raises(ValueError):
        TilingConfig(min_valid_fraction=1.0)
    assert TilingConfig().to_json() == {"tile": 256, "stride": 128,
                                        "min_valid_fraction": 0.001}


def test_tiles_copy_the_picture_and_pad_with_invalid_zeros():
    rng = np.random.default_rng(1)
    gray = rng.integers(1, 256, (200, 300), dtype=np.uint8)     # short and narrow-ish
    tiles = cut_tiles(gray, None)
    assert [t.c0 for t in tiles] == [0, 44]
    for t in tiles:
        assert t.gray.shape == (256, 256) and t.valid.shape == (256, 256)
        assert (t.rows, t.cols) == (200, 256)
        np.testing.assert_array_equal(t.gray[:200], gray[:, t.c0:t.c0 + 256])
        assert not t.gray[200:].any() and not t.valid[200:].any()
        assert t.valid[:200].all()
    narrow = cut_tiles(gray[:, :100], None)
    assert len(narrow) == 1 and narrow[0].cols == 100
    assert not narrow[0].valid[:, 100:].any() and not narrow[0].gray[:, 100:].any()


def test_the_input_is_never_modified():
    gray = np.full((256, 600), 7, np.uint8)
    valid = np.ones_like(gray, bool)
    g0, v0 = gray.copy(), valid.copy()
    for t in cut_tiles(gray, valid):
        t.gray[:] = 0
        t.valid[:] = False
    np.testing.assert_array_equal(gray, g0)
    np.testing.assert_array_equal(valid, v0)


def test_skip_only_essentially_empty_tiles():
    gray = np.full((256, 768), 100, np.uint8)
    valid = np.zeros_like(gray, bool)
    valid[:, :256] = True                      # first tile full
    valid[:10, 300:310] = True                 # 100 px in tile 2: kept (> 65.5)
    tiles = cut_tiles(gray, valid)
    assert [t.skipped for t in tiles] == [False, False, False, True, True]
    assert tiles[1].valid_fraction > 0.001
    # the nadir band (valid, dark) never makes a tile skip
    dark = np.zeros((256, 512), np.uint8)
    assert not any(t.skipped for t in cut_tiles(dark, np.ones_like(dark, bool)))
    # 0.0 skips nothing that has a valid pixel, and an all-invalid tile at 0.0 is kept
    assert not any(t.skipped for t in cut_tiles(gray, valid, TilingConfig(min_valid_fraction=0)))


def test_box_shift_and_clip():
    tiles = cut_tiles(np.zeros((200, 300), np.uint8), None)
    t = tiles[1]                                           # c0 = 44
    assert tile_box_to_picture((10, 5, 20, 15), t) == (54.0, 5.0, 64.0, 15.0)
    # clipped at the real rows (200) and at the tile edge
    assert tile_box_to_picture((250, 190, 300, 230), t) == (294.0, 190.0, 300.0, 200.0)
    # entirely in padding
    assert tile_box_to_picture((10, 210, 20, 250), t) is None
    narrow = cut_tiles(np.zeros((256, 100), np.uint8), None)[0]
    assert tile_box_to_picture((120, 0, 140, 10), narrow) is None


def test_inclusive_pixel_boxes():
    assert to_inclusive_px((10.0, 5.0, 20.0, 15.0), 256, 2400) == (5, 10, 14, 19)
    assert to_inclusive_px((10.49, 5.5, 10.51, 5.6), 256, 2400) == (6, 10, 6, 10)
    assert to_inclusive_px((-3, -2, 2500, 300), 256, 2400) == (0, 0, 255, 2399)


def test_tiling_is_deterministic():
    rng = np.random.default_rng(5)
    gray = rng.integers(0, 256, (256, 1202), dtype=np.uint8)
    a = [(t.c0, t.gray.tobytes(), t.valid.tobytes()) for t in cut_tiles(gray, gray > 3)]
    b = [(t.c0, t.gray.tobytes(), t.valid.tobytes()) for t in cut_tiles(gray, gray > 3)]
    assert a == b


@pytest.mark.skipif(not corpus_image_dirs(), reason="no GCS sessions on this machine")
def test_corpus_pictures_tile_without_losing_a_valid_pixel():
    pre = FixedGray()
    n = 0
    for d in corpus_image_dirs():
        for js in sorted((d / "metadata").glob("seabed_*.json"))[:5]:
            npz = js.with_name(js.stem + "_world.npz")
            if not npz.is_file():
                continue
            img = SeabedImageInput.from_session_files(js, npz, stream_id="session:x")
            inp = pre.run(img)
            H, W = img.shape
            seen = np.zeros((H, W), bool)
            for t in cut_tiles(inp.gray, inp.valid):
                rows = min(H, 256)
                np.testing.assert_array_equal(t.gray[:rows, :t.cols],
                                              inp.gray[:rows, t.c0:t.c0 + t.cols])
                if not t.skipped:
                    seen[:rows, t.c0:t.c0 + t.cols] = True
            assert (seen | ~inp.valid).all()           # every valid pixel is in a kept tile
            n += 1
    assert n > 0
