"""SemanticNode —— 节点复合描述子（计划书 §3.1，接口冻结见 docs/interfaces.md）。"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
import math

import numpy as np

ELEV_HIST_BINS = 16  # 20 m 邻域高程直方图 bin 数


class NodeClass(IntEnum):
    """节点类别，按灾后稳定性排序（★ 越多越稳定）。"""

    UNKNOWN = 0
    ROAD_JUNCTION = 1      # ★★★ 道路网拓扑顶点（交叉口/急弯/断头路端点）
    TERRAIN_FEATURE = 2    # ★★★ 地形特征（山脊/鞍部/冲沟，来自 DEM）
    WATER_EDGE = 3         # ★★☆ 水域/河岸线拐点
    HARD_GROUND = 4        # ★★☆ 大型硬质场地（操场/停车场角点）
    SURVIVING_BUILDING = 5  # ★☆☆ 幸存建筑角点（仅辅助，需损毁评估过滤）


def _normalize_hist(hist: np.ndarray) -> np.ndarray:
    """直方图 L2 归一化；全零/NaN 输入返回全零。"""
    h = np.asarray(hist, dtype=np.float64)
    if h.size == 0 or not np.all(np.isfinite(h)):
        return np.zeros(ELEV_HIST_BINS)
    n = np.linalg.norm(h)
    return h / n if n > 1e-12 else np.zeros(ELEV_HIST_BINS)


@dataclass
class SemanticNode:
    """节点复合描述子 = 语义类别 + 局部 2.5D 高程签名 + 拓扑度数。"""

    id: int
    cls: NodeClass = NodeClass.UNKNOWN
    conf: float = 1.0                       # 提取置信度 [0,1]
    pos_xy: np.ndarray = field(default_factory=lambda: np.zeros(2))  # 全局系（离线）/局部队列系（机载）
    yaw_hint: float = math.nan              # 方向性提示（rad）；无方向为 NaN
    elev_hist: np.ndarray = field(default_factory=lambda: np.zeros(ELEV_HIST_BINS))
    roughness: float = 0.0                  # 邻域高程标准差（m）
    degree: int = 0                         # 图上度（交叉口=3/4）
    edge_angles: list = field(default_factory=list)  # 出边相对方位（排序归一化，旋转不变）

    def __post_init__(self) -> None:
        self.pos_xy = np.asarray(self.pos_xy, dtype=np.float64)
        self.elev_hist = np.asarray(self.elev_hist, dtype=np.float64)
        if self.elev_hist.size != ELEV_HIST_BINS:
            raise ValueError(f"elev_hist must have {ELEV_HIST_BINS} bins, got {self.elev_hist.size}")
        self.edge_angles = sorted(float(a) for a in self.edge_angles if math.isfinite(a))

    # ---------- 属性相似度（候选生成与 score 的组成部分） ----------

    def signature_similarity(self, other: "SemanticNode") -> tuple[bool, float]:
        """返回 (cls 是否一致, 高程签名余弦相似度 ∈[0,1])。计划书 §3.4 步骤 1 的直接依据。"""
        cls_match = (self.cls == other.cls) or self.cls == NodeClass.UNKNOWN or other.cls == NodeClass.UNKNOWN
        ha, hb = _normalize_hist(self.elev_hist), _normalize_hist(other.elev_hist)
        na, nb = np.linalg.norm(ha), np.linalg.norm(hb)
        cos = float(ha @ hb) if (na > 1e-12 and nb > 1e-12) else 0.0
        return cls_match, float(np.clip(cos, 0.0, 1.0))

    def attribute_similarity(self, other: "SemanticNode") -> float:
        """复合属性相似度 ∈[0,1]：类别 + 高程签名 + 粗糙度 + 度数。

        用于匹配 score 的 w2 项（计划书 §3.4 步骤 5）。
        """
        cls_match, cos = self.signature_similarity(other)
        w_cls = 1.0 if cls_match else 0.0
        if w_cls == 0.0:
            return 0.0
        w_hist = 0.5 * cos
        # 粗糙度差（σ_r=0.5 m 量级衰减）
        w_rough = 0.2 * math.exp(-abs(self.roughness - other.roughness) / 0.5)
        # 度数相似
        w_deg = 0.3 * (1.0 if self.degree == other.degree else max(0.0, 1.0 - 0.4 * abs(self.degree - other.degree)))
        return float(np.clip(w_hist + w_rough + w_deg, 0.0, 1.0))

    def uniqueness(self, candidate_sims: list[float]) -> float:
        """属性独特度 = 1 − 次佳候选相似度（种子选取依据，§3.4 步骤 2）。"""
        if not candidate_sims:
            return 1.0
        ordered = sorted(candidate_sims, reverse=True)
        second_best = ordered[1] if len(ordered) > 1 else 0.0
        return 1.0 - second_best
