"""候选生成（计划书 §3.4 步骤 1）：cls + 高程签名余弦相似度取 top-K。

向量化实现：全局节点签名堆成矩阵，每个局部节点一次 matmul 完成全库打分，
保证 N=5000 时仍满足 A10 的 <200 ms 预算。
"""
from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np

from .graph import NodeGraph
from .node import NodeClass, _normalize_hist


def generate_candidates(
    local: NodeGraph,
    global_: NodeGraph,
    top_k: int = 20,
    min_hist_cos: float = 0.0,
) -> Dict[int, List[Tuple[int, float]]]:
    """对 G_l 每个节点，在 G_g 中按「类别一致 + elev_hist 余弦相似度」取 top-K 候选。

    返回 {local_id: [(global_id, eff_sim), ...]}（相似度降序）。
    eff_sim：同类为余弦值；跨类（含 UNKNOWN 放行）打 0.5 折扣。
    """
    gids = list(global_.nodes.keys())
    if gids:
        g_hist = np.stack([_normalize_hist(global_.nodes[g].elev_hist) for g in gids])
        g_cls = np.array([int(global_.nodes[g].cls) for g in gids])
    else:
        g_hist = np.zeros((0, 16))
        g_cls = np.zeros(0, dtype=int)

    cands: Dict[int, List[Tuple[int, float]]] = {}
    for lid, ln in local.nodes.items():
        h = _normalize_hist(ln.elev_hist)
        cos = g_hist @ h  # (N,)
        lc = int(ln.cls)
        cls_ok = (g_cls == lc) | (g_cls == int(NodeClass.UNKNOWN)) | (lc == int(NodeClass.UNKNOWN))
        eff = np.where(g_cls == lc, cos, 0.5 * cos)
        valid = cls_ok & (cos >= min_hist_cos)
        order = np.argsort(-np.where(valid, eff, -1.0), kind="stable")
        picked = []
        for i in order:
            if len(picked) >= top_k:
                break
            if valid[i]:
                picked.append((gids[i], float(eff[i])))
        cands[lid] = picked
    return cands

