"""test_confidence.py — score → 协方差映射（B4）单测。"""
import math

import numpy as np
import pytest

from dr_match.confidence import (
    AlignConfidenceParams,
    compute_score,
    score_to_covariance,
)


def test_score_formula():
    # score = w1·内点率 + w2·属性相似度 − w3·RMSE
    s = compute_score(inlier_ratio=1.0, attr_sim_mean=1.0, rmse=0.0)
    assert s == 0.9  # 0.5*1 + 0.4*1 - 0.1*0


def test_covariance_monotonic_decreasing():
    params = AlignConfidenceParams()
    scores = np.linspace(params.min_score + 0.01, 1.2, 8)
    traces = []
    for s in scores:
        sigma = score_to_covariance(s, params)
        assert sigma is not None
        assert sigma.shape == (3, 3)
        traces.append(np.trace(sigma))
    # score 越高 → 协方差越小（单调递减）
    assert all(a >= b for a, b in zip(traces, traces[1:]))


def test_covariance_bounds():
    params = AlignConfidenceParams()
    s_max = score_to_covariance(5.0, params)   # 远高于 score_ref
    s_low = score_to_covariance(params.min_score + 1e-6, params)
    assert abs(np.sqrt(s_max[0, 0]) - params.sigma_xy_min) < 1e-3
    assert np.sqrt(s_low[0, 0]) >= params.sigma_xy_max * 0.95


def test_below_min_score_returns_none():
    params = AlignConfidenceParams()
    assert score_to_covariance(params.min_score - 0.01, params) is None
    assert score_to_covariance(-1.0, params) is None


def test_covariance_only_xy_yaw():
    # 全局因子只约束 (x, y, yaw)——对角结构，无耦合项
    sigma = score_to_covariance(1.0)
    assert np.allclose(sigma - np.diag(np.diag(sigma)), 0.0)
