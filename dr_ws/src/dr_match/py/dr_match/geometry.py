"""SE(2) 几何验证：Umeyama 最小二乘 + RANSAC（计划书 §3.4 步骤 4）。"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np

_SE2_MIN_PAIRS = 2  # SE(2) 最小模型：2 对对应点


def wrap_pi(a: float) -> float:
    """wrap 到 [-pi, pi]。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def estimate_se2(src: np.ndarray, dst: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Umeyama 最小二乘（无缩放）估计 SE(2)：dst ≈ R @ src + t。

    尺度固定为 1——局部边长（LIO 米制）与全局边长（地图米制）同单位。
    """
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 2:
        raise ValueError(f"expected (n,2) arrays, got {src.shape} / {dst.shape}")
    n = src.shape[0]
    if n < _SE2_MIN_PAIRS:
        raise ValueError("need at least 2 point pairs")
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    h = (src - mu_s).T @ (dst - mu_d)  # 2x2
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T
    t = mu_d - r @ mu_s
    return r, t


def apply_se2(pts: np.ndarray, r: np.ndarray, t: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts, dtype=np.float64)
    return (pts @ r.T) + t


def _fit_error(src: np.ndarray, dst: np.ndarray, r: np.ndarray, t: np.ndarray) -> np.ndarray:
    return np.linalg.norm(apply_se2(src, r, t) - dst, axis=1)


def ransac_se2(
    src: np.ndarray,
    dst: np.ndarray,
    thr: float = 2.0,
    min_inliers: int = 3,
    max_iter: int = 500,
    rng: Optional[np.random.Generator] = None,
) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, int, float]]:
    """RANSAC SE(2)。返回 (R, t, inlier_mask, n_inliers, rmse)；找不到合格解返回 None。

    小图（n≤8）时采样组合数 ≤C(8,2)=28 < max_iter，自然退化为全枚举——对应计划书
    备案「VF2 小图精确解」的简化等价物。
    """
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    n = src.shape[0]
    if n < min_inliers:
        return None
    rng = rng or np.random.default_rng(0)

    pair_idxs = [(i, j) for i in range(n) for j in range(i + 1, n)]
    if len(pair_idxs) > max_iter:
        pair_idxs = [pair_idxs[k] for k in rng.choice(len(pair_idxs), size=max_iter, replace=False)]

    best: Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, int, float]] = None
    best_inliers = -1
    for i, j in pair_idxs:
        try:
            r, t = estimate_se2(src[[i, j]], dst[[i, j]])
        except (np.linalg.LinAlgError, ValueError):
            continue
        err = _fit_error(src, dst, r, t)
        mask = err < thr
        n_in = int(mask.sum())
        if n_in < min_inliers or n_in < best_inliers:
            continue
        # 内点重拟合
        r_ref, t_ref = estimate_se2(src[mask], dst[mask])
        err_ref = _fit_error(src, dst, r_ref, t_ref)
        mask_ref = err_ref < thr
        n_ref = int(mask_ref.sum())
        rmse = float(np.sqrt(np.mean(err_ref[mask_ref] ** 2)))
        if n_ref > best_inliers:
            best_inliers = n_ref
            best = (r_ref, t_ref, mask_ref, n_ref, rmse)
    return best


def se2_from_matrix(r: np.ndarray, t: np.ndarray) -> Tuple[float, float, float]:
    """(R,t) → (x, y, yaw)。"""
    return float(t[0]), float(t[1]), math.atan2(r[1, 0], r[0, 0])
