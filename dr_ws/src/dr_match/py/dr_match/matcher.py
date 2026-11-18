"""种子生长式子图匹配器（计划书 §3.4 算法主线，代码任务 A10–A12）。

流程：
1. 候选生成：cls + 高程签名余弦 → top-K（candidates.py）
2. 种子选取：属性最独特的局部边对，枚举全局候选边对，边长（比例一致）+ 边夹角过滤
3. 生长扩展：BFS 逐节点扩展，成对一致性 |Δd| < τ_d 且 |Δθ| < τ_θ
4. 几何验证：RANSAC SE(2)，内点数 ≥ min_corr 且 RMSE < max_rmse 才接受
5. 置信度：score → 协方差（confidence.py）；保留 top-M 假设（A11，供 GTSAM MaxMixture）
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

from .candidates import generate_candidates
from .confidence import AlignConfidenceParams, compute_score, score_to_covariance
from .geometry import ransac_se2, se2_from_matrix, wrap_pi
from .graph import NodeGraph
from .result import AlignResult


@dataclass
class MatchConfig:
    """匹配参数（计划书 §3.4 伪代码中的常量）。"""

    top_k: int = 20              # 候选生成 K
    tau_d: float = 2.0           # 成对一致性边长容差（m）——容纳 LIO 短程漂移
    tau_theta: float = math.radians(15.0)   # 成对一致性转角容差（rad）
    ransac_thr: float = 2.0      # 几何验证内点阈值（m）
    min_corr: int = 3            # 最少对应数（<3 不匹配，只累积窗口）
    max_rmse: float = 2.0        # 接受上限（m）
    max_hypotheses: int = 5      # A11：top-M 假设（M≤5）
    max_seeds: int = 200         # 种子枚举上限（防组合爆炸）
    min_hist_cos: float = 0.0    # 候选生成直方图余弦下限
    confidence: AlignConfidenceParams = field(default_factory=AlignConfidenceParams)


class GraphMatcher:
    """G_l ⊆? G_g → SE(2) 假设集 + 置信度。"""

    def __init__(self, config: Optional[MatchConfig] = None) -> None:
        self.config = config or MatchConfig()

    # ------------------------------------------------------------------

    def match(self, local: NodeGraph, global_: NodeGraph) -> List[AlignResult]:
        """匹配主入口。返回按 score 降序的假设列表（可为空）。

        局部节点数 < min_corr 时不匹配（计划书备案：只累积窗口，系统靠 LIO 继续跑）。
        """
        cfg = self.config
        if len(local.nodes) < cfg.min_corr:
            return []

        cands = generate_candidates(local, global_, top_k=cfg.top_k, min_hist_cos=cfg.min_hist_cos)

        seeds = self._select_seeds(local, global_, cands)
        if not seeds:
            return []

        hypotheses: List[AlignResult] = []
        seen_corr_sets: Set[frozenset] = set()
        for corr_seed in seeds:
            corr = self._grow(local, global_, cands, corr_seed)
            if len(corr) < cfg.min_corr:
                continue
            hyp = self._verify_and_score(local, global_, corr)
            if hyp is None:
                continue
            key = frozenset((l, g) for l, g in hyp.correspondences)
            if key in seen_corr_sets:
                continue
            seen_corr_sets.add(key)
            hypotheses.append(hyp)

        hypotheses.sort(key=lambda h: h.score, reverse=True)
        for i, h in enumerate(hypotheses[: cfg.max_hypotheses]):
            h.hypothesis_id = i
        return hypotheses[: cfg.max_hypotheses]

    # ------------------------------------------------------------------

    def _select_seeds(
        self,
        local: NodeGraph,
        global_: NodeGraph,
        cands: Dict[int, List[Tuple[int, float]]],
    ) -> List[List[Tuple[int, int]]]:
        """步骤 2：种子对应集。

        遍历局部边（按两端节点属性独特度降序），枚举候选对 (c_u, c_v)：
        - (c_u, c_v) 必须是全局边
        - 边长一致 |d_l − d_g| < τ_d（米制单位相同，比例一致即 1:1）

        注：种子阶段只有一条边，没有旋转参考，不能比较边的绝对方向角
        （局部系与全局系差一个未知 yaw）；夹角检查在生长阶段以「两条边的
        夹角差」形式执行（旋转不变量）。
        """
        cfg = self.config
        seeds: List[List[Tuple[int, int]]] = []

        def node_uniqueness(nid: int) -> float:
            sims = [s for _, s in cands.get(nid, [])]
            if not sims:
                return 0.0
            ordered = sorted(sims, reverse=True)
            second_best = ordered[1] if len(ordered) > 1 else 0.0
            return 1.0 - second_best

        local_edges = sorted(
            local.edges,
            key=lambda e: (node_uniqueness(e.src) + node_uniqueness(e.dst),
                           max(local.degree_of(e.src), local.degree_of(e.dst))),
            reverse=True,
        )

        for e in local_edges:
            if len(seeds) >= cfg.max_seeds:
                break
            d_l = e.length
            for c_u, _ in cands.get(e.src, []):
                for c_v, _ in cands.get(e.dst, []):
                    if c_u == c_v:
                        continue
                    g_edge = global_.edge_between(c_u, c_v)
                    if g_edge is None:
                        continue
                    if abs(g_edge.length - d_l) > cfg.tau_d:
                        continue
                    seeds.append([(e.src, c_u), (e.dst, c_v)])
                    if len(seeds) >= cfg.max_seeds:
                        break
                if len(seeds) >= cfg.max_seeds:
                    break
        return seeds

    def _grow(
        self,
        local: NodeGraph,
        global_: NodeGraph,
        cands: Dict[int, List[Tuple[int, float]]],
        seed: List[Tuple[int, int]],
    ) -> List[Tuple[int, int]]:
        """步骤 3：BFS 种子生长，成对一致性检查。

        候选 b' 需同时满足：
        - 与锚点 a 的边长一致 |d(a,b) − d(a',b')| < τ_d；
        - 与 b 的其他已匹配邻居 x 的边长一致（多锚点，旋转不变）；
        - 边夹角一致：局部 ∠(a→b, x→b) 与全局 ∠(a'→b', x'→b') 之差 < τ_θ
          ——只比较「两条边的夹角」，因此对未知的全局 yaw 不敏感。
        贪心取边长误差最小的候选；单锚点（无夹角检查）由 RANSAC 几何验证兜底。
        """
        cfg = self.config
        corr: Dict[int, int] = dict(seed)
        used_global: Set[int] = {g for _, g in seed}
        queue: List[int] = [l for l, _ in seed]

        while queue:
            a = queue.pop(0)
            a_g = corr[a]
            for b in local.neighbors_of(a):
                if b in corr:
                    continue
                e_ab = local.edge_between(a, b)
                if e_ab is None:
                    continue
                best: Optional[Tuple[int, float]] = None  # (b_g, 边长误差)
                for b_g, _ in cands.get(b, []):
                    if b_g in used_global or b_g == a_g:
                        continue
                    g_ab = global_.edge_between(a_g, b_g)
                    if g_ab is None:
                        continue
                    if abs(g_ab.length - e_ab.length) > cfg.tau_d:
                        continue
                    # 多锚点一致性：b 的其他已匹配邻居 x
                    x_pairs = [(x, x_g) for (x, x_g) in corr.items()
                               if x != a and local.has_edge(x, b)]
                    consistent = True
                    for x, x_g in x_pairs:
                        e_xb = local.edge_between(x, b)
                        g_xb = global_.edge_between(x_g, b_g)
                        if g_xb is None or abs(g_xb.length - e_xb.length) > cfg.tau_d:
                            consistent = False
                            break
                    if not consistent:
                        continue
                    # 边夹角一致性（旋转不变量）：∠(x→b, a→b) 局部 vs 全局
                    if x_pairs:
                        x, x_g = x_pairs[0]
                        e_xb = local.edge_between(x, b)
                        g_xb = global_.edge_between(x_g, b_g)
                        ang_l = wrap_pi(e_xb.dtheta - e_ab.dtheta)
                        ang_g = wrap_pi(g_xb.dtheta - g_ab.dtheta)
                        if abs(wrap_pi(ang_g - ang_l)) > cfg.tau_theta:
                            continue
                    err_d = abs(g_ab.length - e_ab.length)
                    if best is None or err_d < best[1]:
                        best = (b_g, err_d)
                if best is not None:
                    corr[b] = best[0]
                    used_global.add(best[0])
                    queue.append(b)
        return [(l, g) for l, g in corr.items()]

    def _verify_and_score(
        self,
        local: NodeGraph,
        global_: NodeGraph,
        corr: List[Tuple[int, int]],
    ) -> Optional[AlignResult]:
        """步骤 4 + 5：RANSAC SE(2) 几何验证 → score → 协方差。"""
        cfg = self.config
        src = np.array([local.nodes[l].pos_xy for l, _ in corr])
        dst = np.array([global_.nodes[g].pos_xy for _, g in corr])
        out = ransac_se2(src, dst, thr=cfg.ransac_thr, min_inliers=cfg.min_corr)
        if out is None:
            return None
        r, t, mask, n_in, rmse = out
        if n_in < cfg.min_corr or rmse > cfg.max_rmse:
            return None

        inlier_corr = [corr[i] for i in range(len(corr)) if mask[i]]
        attr_sims = [
            local.nodes[l].attribute_similarity(global_.nodes[g]) for l, g in inlier_corr
        ]
        attr_mean = float(np.mean(attr_sims)) if attr_sims else 0.0
        inlier_ratio = n_in / len(corr)
        score = compute_score(inlier_ratio, attr_mean, rmse, cfg.confidence)
        sigma = score_to_covariance(score, cfg.confidence)

        x, y, yaw = se2_from_matrix(r, t)
        return AlignResult(
            hypothesis_id=-1,  # 排序后统一编号
            xy=np.array([x, y]),
            yaw=yaw,
            sigma=sigma if sigma is not None else np.full((3, 3), np.inf),
            score=score,
            inlier_ratio=inlier_ratio,
            rmse=rmse,
            correspondences=inlier_corr,
        )