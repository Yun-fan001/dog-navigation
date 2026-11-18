"""指标计算（计划书 §7.3 验证矩阵，纯 numpy 无 ROS 依赖）。"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import numpy as np

from dr_match.geometry import estimate_se2, apply_se2


def ate_rmse(traj_est: np.ndarray, traj_gt: np.ndarray) -> float:
    """绝对轨迹误差 ATE（m）：Umeyama 对齐后 RMSE（计划书 G2/E2）。

    traj：N×2 平面轨迹。纯 LIO 漂移对比即对同一真值轨迹分别计算 ATE。
    """
    t_e = np.asarray(traj_est, dtype=np.float64)
    t_g = np.asarray(traj_gt, dtype=np.float64)
    if t_e.shape != t_g.shape or t_e.ndim != 2 or t_e.shape[1] != 2:
        raise ValueError(f"expected (N,2) arrays, got {t_e.shape} / {t_g.shape}")
    r, t = estimate_se2(t_e, t_g)
    aligned = apply_se2(t_e, r, t)
    return float(np.sqrt(np.mean(np.sum((aligned - t_g) ** 2, axis=1))))


def success_rate_curve(
    results: Sequence[Tuple[bool, int]],
    max_nodes: int = 5,
) -> List[Tuple[int, float]]:
    """G1/E1：走 N 个节点后的定位成功率曲线。

    results：[(是否成功, 成功时已走过节点数), ...]。
    返回 [(节点数, 累计成功率), ...]（≤N 节点时成功占比，单调不减）。
    """
    if not results:
        return [(n, 0.0) for n in range(1, max_nodes + 1)]
    total = len(results)
    curve = []
    for n in range(1, max_nodes + 1):
        ok = sum(1 for success, nodes in results if success and nodes <= n)
        curve.append((n, ok / total))
    return curve


def repeatability(
    run1: Sequence[np.ndarray],
    run2: Sequence[np.ndarray],
    radius_m: float = 3.0,
) -> float:
    """G3/Gate Review 1：节点重复检出率。

    两次遍历同区域的节点位置列表；run1 中在 run2 某节点 radius 内的比例。
    计划书通过线 ≥60%。
    """
    a = [np.asarray(p, dtype=np.float64) for p in run1]
    b = [np.asarray(p, dtype=np.float64) for p in run2]
    if not a:
        return 0.0
    b_arr = np.vstack(b) if b else np.zeros((0, 2))
    hit = 0
    for p in a:
        if b_arr.size and np.linalg.norm(b_arr - p, axis=1).min() <= radius_m:
            hit += 1
    return hit / len(a)
