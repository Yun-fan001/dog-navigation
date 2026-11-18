"""AlignResult —— A12 对齐输出（接口冻结见 docs/interfaces.md §2）。"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np


@dataclass
class AlignResult:
    """一次匹配假设：全局 (x, y, yaw) + 协方差 + 置信度。"""

    hypothesis_id: int
    xy: np.ndarray = field(default_factory=lambda: np.zeros(2))   # 全局 (x, y)
    yaw: float = 0.0                                              # 全局 yaw（rad）
    sigma: np.ndarray = field(default_factory=lambda: np.full((3, 3), np.inf))  # 协方差
    score: float = 0.0
    inlier_ratio: float = 0.0
    rmse: float = np.inf
    correspondences: List[Tuple[int, int]] = field(default_factory=list)  # (local_id, global_id)

    def __post_init__(self) -> None:
        self.xy = np.asarray(self.xy, dtype=np.float64)
        self.sigma = np.asarray(self.sigma, dtype=np.float64)
        if self.sigma.shape != (3, 3):
            raise ValueError(f"sigma must be 3x3, got {self.sigma.shape}")
