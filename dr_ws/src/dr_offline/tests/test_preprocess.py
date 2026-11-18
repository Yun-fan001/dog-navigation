"""test_preprocess.py — A1 切片管线单测（纯 numpy 部分）。"""
import numpy as np
import pytest

from dr_offline.preprocess.pipeline import tile_image, tile_positions


def test_tile_geometry_full():
    """10×10 图，tile=6，overlap=2（stride=4）→ 3×3=9 块：位置与重叠正确。"""
    img = np.arange(100).reshape(10, 10)
    tiles = tile_image(img, tile_size=6, overlap=2)
    positions = sorted((t.x0, t.y0) for t in tiles)
    expected = [(x, y) for x in (0, 4, 8) for y in (0, 4, 8)]
    assert positions == expected
    assert tiles[0].data.shape == (6, 6)
    # 边缘块不足 tile_size
    edge_shapes = [t.data.shape for t in tiles if t.x0 == 8 or t.y0 == 8]
    assert edge_shapes == [(2, 6), (2, 6), (6, 2), (2, 2), (2, 6)] or all(
        s[0] == 2 or s[1] == 2 for s in edge_shapes
    )


def test_tile_reconstructs_full_coverage():
    """所有块并集覆盖整幅图（重叠允许）。"""
    rng = np.random.default_rng(0)
    img = rng.uniform(size=(100, 80))
    tiles = tile_image(img, tile_size=32, overlap=8)
    covered = np.zeros_like(img, dtype=bool)
    for t in tiles:
        h, w = t.data.shape
        covered[t.y0:t.y0 + h, t.x0:t.x0 + w] = True
    assert covered.all()


def test_tile_positions_match():
    img = np.zeros((50, 60))
    tiles = tile_image(img, tile_size=20, overlap=4)
    positions = tile_positions(img.shape, tile_size=20, overlap=4)
    assert positions == [(t.x0, t.y0) for t in tiles]


def test_invalid_params_raise():
    img = np.zeros((10, 10))
    with pytest.raises(ValueError):
        tile_image(img, tile_size=0)
    with pytest.raises(ValueError):
        tile_image(img, tile_size=10, overlap=10)
