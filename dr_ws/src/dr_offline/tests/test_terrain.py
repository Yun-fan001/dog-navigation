"""test_terrain.py — A4 验收：已知 DEM 上复现标注地形点，位置误差 ≤1 像元。"""
import numpy as np
import pytest

from dr_offline.terrain.features import (
    KIND_RIDGE,
    KIND_SADDLE,
    detect_ridge_saddle,
    elevation_signature,
)


def make_two_hill_dem(shape=(120, 120), c1=(40, 60), c2=(80, 60), h=30.0, sigma=12.0):
    """两座等高高斯山 → 鞍部位于两山顶连线中点 (60, 60)。"""
    xs = np.arange(shape[1])
    ys = np.arange(shape[0])
    X, Y = np.meshgrid(xs, ys)
    d1 = (X - c1[0]) ** 2 + (Y - c1[1]) ** 2
    d2 = (X - c2[0]) ** 2 + (Y - c2[1]) ** 2
    return h * np.exp(-d1 / (2 * sigma ** 2)) + h * np.exp(-d2 / (2 * sigma ** 2))


def test_saddle_detection_within_1_pixel():
    """A4 验收：鞍部检出位置误差 ≤1 像元。"""
    dem = make_two_hill_dem()
    feats = detect_ridge_saddle(dem, gsd=1.0)
    saddles = [(x, y) for x, y, k in feats if k == KIND_SADDLE]
    assert len(saddles) > 0, "no saddle detected"
    gt = np.array([60.0, 60.0])
    err = min(np.linalg.norm(np.array(s) - gt) for s in saddles)
    print(f"\nsaddle position error: {err:.2f} px")
    assert err <= 1.0, f"saddle error {err:.2f} px > 1 px"


def test_ridge_detection_present():
    """山脊特征应在两山附近检出。"""
    dem = make_two_hill_dem()
    feats = detect_ridge_saddle(dem, gsd=1.0)
    ridges = [(x, y) for x, y, k in feats if k == KIND_RIDGE]
    assert len(ridges) > 0, "no ridge detected"
    # 山脊应位于山顶邻域（距任一山顶 ≤ 3σ）
    for x, y in ridges:
        d = min(np.hypot(x - 40, y - 60), np.hypot(x - 80, y - 60))
        assert d <= 3 * 12.0 + 5.0


def test_elevation_signature_flat():
    """平面 DEM：粗糙度 ≈0，直方图集中于一个 bin。"""
    dem = np.ones((100, 100)) * 5.0
    hist, rough = elevation_signature(dem, 50.0, 50.0, radius_m=20.0, gsd=1.0)
    assert rough == pytest.approx(0.0, abs=1e-9)
    assert hist.sum() == pytest.approx(1.0)
    assert (hist > 0.9).sum() == 1  # 单一 bin 占满


def test_elevation_signature_discriminative():
    """山顶 vs 山外平原的签名可区分（余弦相似度低）。"""
    dem = make_two_hill_dem()
    h1, _ = elevation_signature(dem, 40.0, 60.0, radius_m=15.0)  # 山顶
    h2, _ = elevation_signature(dem, 10.0, 10.0, radius_m=15.0)  # 山外平原
    n1 = h1 / np.linalg.norm(h1)
    n2 = h2 / np.linalg.norm(h2)
    cos = float(n1 @ n2)
    print(f"\nsignature cosine: {cos:.3f}")
    assert cos < 0.6, f"signatures too similar: cos={cos:.3f}"
