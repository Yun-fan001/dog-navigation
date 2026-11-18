"""test_metrics.py — E2/G1/G3 指标计算单测。"""
import math

import numpy as np
import pytest

from dr_eval.metrics import ate_rmse, repeatability, success_rate_curve


def test_ate_zero_for_exact():
    traj = np.column_stack([np.linspace(0, 10, 50), np.sin(np.linspace(0, 10, 50))])
    assert ate_rmse(traj, traj) == pytest.approx(0.0, abs=1e-9)


def test_ate_invariant_to_rigid_transform():
    """ATE 对轨迹整体刚性变换不变（对齐消除全局位姿差）。"""
    rng = np.random.default_rng(0)
    gt = np.column_stack([np.linspace(0, 20, 100), np.linspace(0, 5, 100)])
    theta = 1.2
    R = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    t = np.array([30.0, -15.0])
    transformed = gt @ R.T + t
    assert ate_rmse(transformed, gt) == pytest.approx(0.0, abs=1e-9)


def test_ate_scales_with_noise():
    """噪声 0.1 m → ATE ≈ 0.1 量级（G2 对比基准的合理性检查）。"""
    rng = np.random.default_rng(1)
    gt = np.column_stack([np.linspace(0, 30, 200), np.zeros(200)])
    est = gt + rng.normal(0, 0.1, size=gt.shape)
    ate = ate_rmse(est, gt)
    assert 0.03 < ate < 0.3


def test_success_rate_curve_monotonic():
    """成功率曲线随节点数单调不减（G1）。"""
    results = [(True, 3), (False, 5), (True, 5), (True, 1)]
    curve = success_rate_curve(results, max_nodes=5)
    rates = [r for _, r in curve]
    assert rates == sorted(rates)
    assert curve[0] == (1, 0.25)   # 仅 1 节点成功
    assert curve[2] == (3, 0.5)    # ≤3 节点成功 2/4
    assert curve[4] == (5, 0.75)   # ≤5 节点成功 3/4


def test_success_rate_curve_empty():
    curve = success_rate_curve([])
    assert all(r == 0.0 for _, r in curve)


def test_repeatability_full_match():
    """相同节点（小噪声）→ 重复检出率 100%。"""
    rng = np.random.default_rng(2)
    pts = rng.uniform(0, 50, size=(10, 2))
    run2 = pts + rng.normal(0, 0.5, size=pts.shape)
    assert repeatability(pts, run2, radius_m=3.0) == pytest.approx(1.0)


def test_repeatability_partial_and_none():
    rng = np.random.default_rng(3)
    pts = rng.uniform(0, 50, size=(10, 2))
    # 一半节点 + 噪声
    assert repeatability(pts, pts[:5] + rng.normal(0, 0.5, size=(5, 2)), radius_m=3.0) == pytest.approx(0.5)
    # 完全不重合
    far = pts + 100.0
    assert repeatability(pts, far, radius_m=3.0) == pytest.approx(0.0)


def test_repeatability_empty():
    assert repeatability([], []) == 0.0


def test_ate_rejects_bad_shape():
    with pytest.raises(ValueError):
        ate_rmse(np.zeros((5, 2)), np.zeros((6, 2)))
