"""置信度 → 协方差映射（计划书 §3.4 步骤 5、B4）。

score = w1·内点率 + w2·属性相似度均值 − w3·RMSE（RMSE 单位 m）。
协方差随 score 单调递减；score 低于阈值 → 不注入因子（NGPS 式平滑退化）。
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np


@dataclass
class AlignConfidenceParams:
    # score 权重（计划书 §3.4 步骤 5）
    w_inlier: float = 0.5
    w_attr: float = 0.4
    w_rmse: float = 0.1          # 1/m，与 rmse 相乘后无量纲
    # 协方差映射：sigma = max_sigma * exp(-slope * (score - score_ref))
    sigma_xy_max: float = 25.0   # 低置信度时平面标准差上限（m）——约为初始不确定度 30 m 量级
    sigma_yaw_max: float = math.pi / 3.0
    sigma_xy_min: float = 0.5    # 高置信度时平面标准差下限（m）
    sigma_yaw_min: float = math.radians(3.0)
    slope: float = 3.0
    score_ref: float = 0.8       # score=score_ref 时 sigma 取 max（开始收缩的阈值）
    min_score: float = 0.4       # 低于该 score 不注入因子（匹配失效 → 系统纯 LIO）


def compute_score(
    inlier_ratio: float,
    attr_sim_mean: float,
    rmse: float,
    params: Optional[AlignConfidenceParams] = None,
) -> float:
    p = params or AlignConfidenceParams()
    return float(p.w_inlier * inlier_ratio + p.w_attr * attr_sim_mean - p.w_rmse * rmse)


def score_to_covariance(
    score: float,
    params: Optional[AlignConfidenceParams] = None,
) -> Optional[np.ndarray]:
    """score → 3×3 协方差（x, y, yaw 对角）；score 过低返回 None（不注入）。"""
    p = params or AlignConfidenceParams()
    if score < p.min_score:
        return None
    gain = math.exp(-p.slope * max(0.0, score - p.score_ref))
    s_xy = p.sigma_xy_min + (p.sigma_xy_max - p.sigma_xy_min) * gain
    s_yaw = p.sigma_yaw_min + (p.sigma_yaw_max - p.sigma_yaw_min) * gain
    return np.diag([s_xy**2, s_xy**2, s_yaw**2])
