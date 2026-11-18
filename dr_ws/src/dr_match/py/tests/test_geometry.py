"""test_geometry.py — SE(2) 几何验证单测（RANSAC/Umeyama）。"""
import math

import numpy as np
import pytest

from dr_match.geometry import (
    apply_se2,
    estimate_se2,
    ransac_se2,
    se2_from_matrix,
    wrap_pi,
)


def _rot2(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def test_estimate_se2_exact():
    rng = np.random.default_rng(0)
    r_true = _rot2(math.radians(37.0))
    t_true = np.array([5.0, -3.0])
    src = rng.uniform(-50, 50, size=(10, 2))
    dst = apply_se2(src, r_true, t_true)
    r, t = estimate_se2(src, dst)
    assert np.allclose(r, r_true, atol=1e-9)
    assert np.allclose(t, t_true, atol=1e-9)


def test_estimate_se2_with_noise():
    rng = np.random.default_rng(1)
    r_true = _rot2(math.radians(-70.0))
    t_true = np.array([-8.0, 12.0])
    src = rng.uniform(-50, 50, size=(20, 2))
    dst = apply_se2(src, r_true, t_true) + rng.normal(0, 0.05, size=(20, 2))
    r, t = estimate_se2(src, dst)
    assert np.allclose(r, r_true, atol=0.01)
    assert np.allclose(t, t_true, atol=0.2)


def test_ransac_se2_with_outliers():
    rng = np.random.default_rng(2)
    r_true = _rot2(math.radians(90.0))
    t_true = np.array([10.0, 10.0])
    src = rng.uniform(-50, 50, size=(10, 2))
    dst = apply_se2(src, r_true, t_true)
    # 4 对换成外点（乱对应）
    dst[0], dst[1] = dst[4].copy(), dst[5].copy()
    dst[2], dst[3] = dst[6].copy(), dst[7].copy()
    out = ransac_se2(src, dst, thr=0.5, min_inliers=3, rng=rng)
    assert out is not None
    r, t, mask, n_in, rmse = out
    assert n_in == 6
    assert np.allclose(r, r_true, atol=1e-6)
    assert np.allclose(t, t_true, atol=1e-6)
    assert rmse < 1e-6


def test_ransac_se2_too_few_inliers_returns_none():
    rng = np.random.default_rng(3)
    src = rng.uniform(0, 10, size=(4, 2))
    dst = rng.uniform(0, 10, size=(4, 2))  # 完全乱对应
    out = ransac_se2(src, dst, thr=0.01, min_inliers=3, rng=rng)
    assert out is None


def test_wrap_pi():
    assert wrap_pi(0.0) == pytest.approx(0.0)
    assert abs(wrap_pi(math.pi)) == pytest.approx(math.pi)   # π 与 −π 等价
    assert abs(wrap_pi(-math.pi)) == pytest.approx(math.pi)
    assert abs(wrap_pi(3 * math.pi)) == pytest.approx(math.pi)
    assert wrap_pi(math.radians(200)) == pytest.approx(math.radians(-160))


def test_se2_from_matrix():
    r = _rot2(math.radians(45.0))
    t = np.array([1.0, 2.0])
    x, y, yaw = se2_from_matrix(r, t)
    assert x == pytest.approx(1.0)
    assert y == pytest.approx(2.0)
    assert yaw == pytest.approx(math.radians(45.0))
