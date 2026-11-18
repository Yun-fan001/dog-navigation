"""A4 地形节点提取（DEM → 山脊/鞍部 + 2.5D 高程签名）。

计划书 A4：DEM（SRTM/AW3D30/UAV DSM）→ 高程梯度 + 脊线/鞍部检测。
验收：已知 DEM 上复现标注地形点，位置误差 ≤1 像元。
"""
from __future__ import annotations

from typing import List, Tuple

import numpy as np
from scipy.spatial import cKDTree

# 特征类型
KIND_RIDGE = 0
KIND_SADDLE = 1


def _cluster_points(
    pts: List[Tuple[int, int]],
    eps_px: float,
) -> List[List[int]]:
    """贪心半径聚类：返回 [[点索引, ...], ...]。"""
    if not pts:
        return []
    arr = np.asarray(pts, dtype=float)
    tree = cKDTree(arr)
    visited: set = set()
    clusters: List[List[int]] = []
    for k in range(len(arr)):
        if k in visited:
            continue
        nb = tree.query_ball_point(arr[k], eps_px)
        for m in nb:
            visited.add(m)
        clusters.append(list(nb))
    return clusters


def detect_ridge_saddle(
    dem: np.ndarray,
    gsd: float = 1.0,
    saddle_grad_thr: float = 0.05,
    ridge_lap_thr: float = -0.01,
    cluster_eps_m: float = 5.0,
) -> List[Tuple[float, float, int]]:
    """DEM → 地形特征点列表 [(x_m, y_m, kind), ...]。

    - 鞍部：|∇h| < thr 且 det(H) < 0（Hessian 特征值异号——山脊与沟谷的鞍点）
    - 山脊：拉普拉斯（曲率和）< 负阈值 且 坡度高于全图中值
    """
    dem = np.asarray(dem, dtype=np.float64)
    gy, gx = np.gradient(dem)
    grad_mag = np.hypot(gx, gy)
    gyy, gyx = np.gradient(gy)
    gxy, gxx = np.gradient(gx)
    lap = gxx + gyy
    det_h = gxx * gyy - gxy * gyx

    saddle_mask = (grad_mag < saddle_grad_thr) & (det_h < 0)
    ridge_mask = (lap < ridge_lap_thr) & (grad_mag > float(np.median(grad_mag)))

    eps_px = cluster_eps_m / gsd
    out: List[Tuple[float, float, int]] = []
    for mask, kind in ((ridge_mask, KIND_RIDGE), (saddle_mask, KIND_SADDLE)):
        ys, xs = np.where(mask)
        if len(xs) == 0:
            continue
        clusters = _cluster_points(list(zip(xs, ys)), eps_px)
        for cl in clusters:
            cx = float(np.mean([xs[m] for m in cl])) * gsd
            cy = float(np.mean([ys[m] for m in cl])) * gsd
            out.append((cx, cy, kind))
    return out


def elevation_signature(
    dem: np.ndarray,
    cx_px: float,
    cy_px: float,
    radius_m: float = 20.0,
    gsd: float = 1.0,
    n_bins: int = 16,
) -> Tuple[np.ndarray, float]:
    """以 (cx_px, cy_px) 为中心 20 m 邻域的高程直方图 + 粗糙度（计划书 §3.1 签名）。"""
    dem = np.asarray(dem, dtype=np.float64)
    r_px = radius_m / gsd
    h, w = dem.shape
    y0, y1 = max(0, int(cy_px - r_px)), min(h, int(cy_px + r_px) + 1)
    x0, x1 = max(0, int(cx_px - r_px)), min(w, int(cx_px + r_px) + 1)
    yy, xx = np.ogrid[y0:y1, x0:x1]
    mask = (xx - cx_px) ** 2 + (yy - cy_px) ** 2 <= r_px ** 2
    vals = dem[y0:y1, x0:x1][mask]
    if vals.size == 0:
        return np.zeros(n_bins, dtype=np.float32), 0.0
    hist, _ = np.histogram(vals, bins=n_bins, density=True)
    hist = hist / hist.sum() if hist.sum() > 0 else hist  # L1 归一化（sum=1）
    return hist.astype(np.float32), float(vals.std())
