"""dr_match —— 图匹配与全局对齐（计划书 A10–A12）。

无 ROS 依赖的纯 Python 参考实现，作为 C++ 版（dr_match/core，Ubuntu 构建）
的 golden reference：C++ 落地时用同一组测试数据交叉验证。

算法主线（计划书 §3.4）：
  候选生成 → 种子选取（属性独特边对 + 边长/夹角过滤）→ 种子生长（成对一致性）
  → RANSAC SE(2) 几何验证 → 置信度与协方差映射
"""

from .node import NodeClass, SemanticNode
from .graph import Edge, NodeGraph
from .geometry import wrap_pi, estimate_se2, apply_se2, ransac_se2
from .candidates import generate_candidates
from .result import AlignResult
from .confidence import score_to_covariance, AlignConfidenceParams
from .matcher import MatchConfig, GraphMatcher

__all__ = [
    "NodeClass",
    "SemanticNode",
    "Edge",
    "NodeGraph",
    "wrap_pi",
    "estimate_se2",
    "apply_se2",
    "ransac_se2",
    "generate_candidates",
    "AlignResult",
    "score_to_covariance",
    "AlignConfidenceParams",
    "MatchConfig",
    "GraphMatcher",
]

__version__ = "0.1.0"
