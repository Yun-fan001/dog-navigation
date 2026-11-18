"""NodeGraph —— 属性图（全局 G_g / 局部 G_l 通用）。

节点间边 = 相对位姿（边长 + 转角），机载侧来自 LIO（A9：滑动窗口 3–8 节点）。
"""
from __future__ import annotations

from dataclasses import dataclass
import copy
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from .geometry import wrap_pi
from .node import SemanticNode


@dataclass
class Edge:
    src: int
    dst: int
    length: float              # 边长（m）
    dtheta: float = 0.0        # 转角（rad，wrap 到 [-pi, pi]）


class NodeGraph:
    """无向属性图。邻接查询用 hash 结构，保证 N=数千节点时匹配器可用。"""

    def __init__(self) -> None:
        self.nodes: Dict[int, SemanticNode] = {}
        self.edges: List[Edge] = []
        self._adj: Dict[int, List[Edge]] = {}
        self._edge_set: set = set()  # (min_id, max_id)

    # ---------- 构建 ----------

    def add_node(self, node: SemanticNode) -> None:
        if node.id in self.nodes:
            raise ValueError(f"duplicate node id {node.id}")
        self.nodes[node.id] = node
        self._adj[node.id] = []

    def add_edge(self, e: Edge) -> None:
        if e.src not in self.nodes or e.dst not in self.nodes:
            raise ValueError(f"edge references unknown node: {e.src}-{e.dst}")
        key = (min(e.src, e.dst), max(e.src, e.dst))
        if key in self._edge_set:
            return
        self.edges.append(e)
        self._edge_set.add(key)
        self._adj[e.src].append(e)
        # 反向副本：dtheta 语义为「src→dst 方向角」，反向 = +π（不是取负！）
        self._adj[e.dst].append(Edge(e.dst, e.src, e.length, wrap_pi(e.dtheta + math.pi)))

    # ---------- 查询 ----------

    def has_edge(self, a: int, b: int) -> bool:
        return (min(a, b), max(a, b)) in self._edge_set

    def edge_between(self, a: int, b: int) -> Optional[Edge]:
        for e in self._adj.get(a, []):
            if e.dst == b:
                return e
        return None

    def neighbors_of(self, nid: int) -> List[int]:
        return [e.dst for e in self._adj.get(nid, [])]

    def degree_of(self, nid: int) -> int:
        return len(self._adj.get(nid, []))

    def edge_angles_of(self, nid: int) -> List[float]:
        """出边相对方位（rad），排序归一化 → 旋转不变（计划书 §3.1）。"""
        node = self.nodes[nid]
        ang = []
        for e in self._adj.get(nid, []):
            dx = self.nodes[e.dst].pos_xy[0] - node.pos_xy[0]
            dy = self.nodes[e.dst].pos_xy[1] - node.pos_xy[1]
            ang.append(math.atan2(dy, dx))
        ang.sort()
        if len(ang) <= 1:
            return ang
        return [a - ang[0] for a in ang[1:]]

    # ---------- 结构 ----------

    def local_ids_sorted_by(self, key) -> List[int]:
        return sorted(self.nodes.keys(), key=key)

    def update_topology_attributes(self) -> None:
        """按几何重算 degree / edge_angles（建图后调用一次）。"""
        for nid, node in self.nodes.items():
            node.degree = self.degree_of(nid)
            node.edge_angles = self.edge_angles_of(nid)

    def recompute_edges(self) -> None:
        """按当前节点坐标重算所有边的 length / dtheta（坐标被修改后调用）。

        dtheta 是几何衍生属性，必须与 pos_xy 保持一致；否则
        edge_between 返回的反向边副本会持有过期几何。
        """
        pairs = [(e.src, e.dst) for e in self.edges]
        self.edges.clear()
        self._adj = {nid: [] for nid in self.nodes}
        self._edge_set.clear()
        for a, b in pairs:
            pa, pb = self.nodes[a].pos_xy, self.nodes[b].pos_xy
            dx, dy = pb[0] - pa[0], pb[1] - pa[1]
            d = float(math.hypot(dx, dy))
            self.add_edge(Edge(a, b, d, math.atan2(dy, dx)))

    def pairwise(self, a: int, b: int) -> Optional[Tuple[float, float]]:
        """返回 (边长, 转角 wrap 到 [-pi,pi])，无路径返回 None。"""
        e = self.edge_between(a, b)
        if e is not None:
            return e.length, e.dtheta
        return None

    def node_ids(self) -> List[int]:
        return list(self.nodes.keys())

    def subgraph(self, ids: List[int]) -> "NodeGraph":
        """抽取节点子集 + 保留内部边（滑动窗口 G_l 构造用）。

        节点深拷贝：局部图的坐标/签名修改不得污染原图（局部图与全局图
        是同一批节点的两次独立观测）。
        """
        g = NodeGraph()
        for i in ids:
            g.add_node(copy.deepcopy(self.nodes[i]))
        for e in self.edges:
            if e.src in g.nodes and e.dst in g.nodes:
                g.add_edge(Edge(e.src, e.dst, e.length, e.dtheta))
        return g

    def __len__(self) -> int:
        return len(self.nodes)
