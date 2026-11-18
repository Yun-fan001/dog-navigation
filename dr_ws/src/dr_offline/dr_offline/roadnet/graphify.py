"""A3 路网提取与节点化：骨架 → 图 → 度≠2 聚类节点。

计划书 A3：道路掩膜 → 形态学骨架化 → 图化（NetworkX 思想）→ 度≠2 的像素
聚类为节点。验收：检出率 ≥85%、虚警 ≤15%、位置误差 ≤2 px。

节点类别按灾后稳定性：道路网拓扑顶点 ★★★（交叉口/急弯/断头路端点）。
"""
from __future__ import annotations

from typing import Dict, List, Set, Tuple

import numpy as np
from scipy.spatial import cKDTree

from dr_match.graph import Edge, NodeGraph
from dr_match.node import NodeClass, SemanticNode

from .skeleton import skeletonize

_NEIGH = [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]


def skeleton_to_graph(
    skel: np.ndarray,
    gsd: float = 1.0,
    cluster_eps_m: float = 3.0,
) -> NodeGraph:
    """骨架二值图 → NodeGraph（节点 = 度≠2 像素聚类质心，边 = 度 2 链路径长）。

    gsd：地面采样距离（m/px），用于像素 ↔ 米换算。
    cluster_eps_m：关键像素聚类半径（m），对应计划书 A8 的 DBSCAN eps=3 m。
    """
    skel = np.asarray(skel, dtype=bool)
    ys, xs = np.where(skel)
    n = len(xs)
    g = NodeGraph()
    if n == 0:
        return g

    pix_to_id: Dict[Tuple[int, int], int] = {
        (int(x), int(y)): i for i, (x, y) in enumerate(zip(xs, ys))
    }
    adj: List[Set[int]] = [set() for _ in range(n)]
    for i in range(n):
        x, y = xs[i], ys[i]
        for dx, dy in _NEIGH:
            j = pix_to_id.get((int(x + dx), int(y + dy)))
            if j is not None and i < j:
                adj[i].add(j)
                adj[j].add(i)
    degree = [len(a) for a in adj]

    # 关键点（度≠2）聚类 → 节点
    key_ids = [i for i in range(n) if degree[i] != 2]
    if not key_ids:
        return g  # 纯环结构（无路口/端点），暂无节点——环交点由其他检测器覆盖
    key_pts = np.column_stack([xs[key_ids], ys[key_ids]]).astype(float)
    eps_px = cluster_eps_m / gsd
    tree = cKDTree(key_pts)
    visited: Set[int] = set()
    node_of_key: Dict[int, int] = {}
    for k, i in enumerate(key_ids):
        if k in visited:
            continue
        nb = tree.query_ball_point(key_pts[k], eps_px)
        for m in nb:
            visited.add(m)
            node_of_key[key_ids[m]] = len(g.nodes)
        cx = float(np.mean(key_pts[nb, 0])) * gsd
        cy = float(np.mean(key_pts[nb, 1])) * gsd
        g.add_node(SemanticNode(
            id=len(g.nodes),
            cls=NodeClass.ROAD_JUNCTION,
            conf=1.0,
            pos_xy=np.array([cx, cy]),
        ))

    # 度 2 链收缩 → 边（长度 = 链像素数 × gsd）
    key_set: Set[int] = set(key_ids)
    seen_pairs: Set[Tuple[int, int]] = set()
    for k0 in key_ids:
        for nb in adj[k0]:
            prev, cur = k0, nb
            length = 1.0
            traversed: Set[int] = {k0}
            while degree[cur] == 2 and cur not in traversed:
                traversed.add(cur)
                nxt = next(i for i in adj[cur] if i != prev)
                prev, cur = cur, nxt
                length += 1.0
            if cur not in key_set:
                continue  # 闭环回到起点
            a, b = node_of_key[k0], node_of_key[cur]
            if a == b:
                continue
            pair = (min(a, b), max(a, b))
            if pair in seen_pairs:
                continue
            seen_pairs.add(pair)
            g.add_edge(Edge(a, b, length * gsd))

    g.recompute_edges()
    g.update_topology_attributes()
    return g


def extract_road_nodes(
    mask: np.ndarray,
    gsd: float = 0.3,
    cluster_eps_m: float = 3.0,
) -> NodeGraph:
    """道路掩膜 → 节点图（A3 完整管线：骨架化 + 图化）。"""
    return skeleton_to_graph(skeletonize(mask), gsd=gsd, cluster_eps_m=cluster_eps_m)
