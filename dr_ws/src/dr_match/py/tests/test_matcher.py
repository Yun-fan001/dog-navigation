"""test_matcher.py — 图匹配器验收单测（计划书 A10–A12）。

验收标准即断言：
- A10：仿真已知真值下 top-1 正确率 ≥90%（无损毁场景）；单次 <200 ms（N=5000，n=5）
  （Python 参考实现性能基线宽松为 <5 s；C++ 版硬线 200 ms）
- A11：多假设数 ≤M；A12：输出 (x,y,yaw) + Σ
"""
import math
import time

import numpy as np
from scipy.spatial import cKDTree

from dr_match.geometry import wrap_pi
from dr_match.graph import Edge, NodeGraph
from dr_match.matcher import GraphMatcher, MatchConfig
from dr_match.node import NodeClass, SemanticNode

CLASSES = [
    NodeClass.ROAD_JUNCTION,
    NodeClass.TERRAIN_FEATURE,
    NodeClass.WATER_EDGE,
    NodeClass.HARD_GROUND,
]


def _rot2(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def _hist_for(id_: int, rng: np.random.Generator, noise: float = 0.0) -> np.ndarray:
    """与节点 id 绑定的可区分高程签名（主峰位置 = id % 16）。"""
    h = rng.uniform(0.3, 1.0, size=16)
    h[int(id_) % 16] += 2.0
    h = h / np.linalg.norm(h)
    if noise > 0:
        h = h + rng.normal(0, noise, size=16)
        h = h / np.linalg.norm(h)
    return h


def make_global_graph(rng: np.random.Generator, n: int = 60, area: float = 500.0, k: int = 3, seed: int = 0) -> NodeGraph:
    """随机属性图：均匀分布节点 + k 近邻边（含边长/转角）。"""
    rng = np.random.default_rng(seed)
    g = NodeGraph()
    for i in range(n):
        g.add_node(SemanticNode(
            id=i,
            cls=CLASSES[int(rng.integers(0, len(CLASSES)))],
            pos_xy=rng.uniform(0, area, size=2),
            elev_hist=_hist_for(i, rng),
            roughness=float(rng.uniform(0.0, 0.5)),
        ))
    pts = np.array([g.nodes[i].pos_xy for i in range(n)])
    tree = cKDTree(pts)
    _, idx = tree.query(pts, k=min(k + 1, n))  # 含自身
    for i in range(n):
        for j in idx[i]:
            j = int(j)
            if j == i or g.has_edge(i, j):
                continue
            d = float(np.linalg.norm(pts[i] - pts[j]))
            dx, dy = pts[j] - pts[i]
            g.add_edge(Edge(i, j, d, wrap_pi(math.atan2(dy, dx))))
    g.update_topology_attributes()
    return g


def make_local_graph(
    global_: NodeGraph,
    rng: np.random.Generator,
    size: int = 5,
    seed: int = 0,
    pos_noise: float = 0.05,
    angle_noise_deg: float = 1.0,
):
    """抽取连通局部子图 → 变换到局部队列系 + 观测噪声（模拟 LIO 短程测量）。

    真值变换：global = R_true @ local + t_true。返回 (local, R_true, t_true, ids)。
    """
    rng = np.random.default_rng(seed)
    start = int(rng.integers(0, len(global_.nodes)))
    ids, q = [start], [start]
    while q and len(ids) < size:
        cur = q.pop(0)
        for nb in global_.neighbors_of(cur):
            if nb not in ids:
                ids.append(nb)
                q.append(nb)
                if len(ids) >= size:
                    break
    local = global_.subgraph(ids)

    theta = float(rng.uniform(-math.pi, math.pi))
    t_true = rng.uniform(0, 300, size=2)
    r_true = _rot2(theta)
    for nid in local.nodes:
        n = local.nodes[nid]
        gpos = global_.nodes[nid].pos_xy
        n.pos_xy = r_true.T @ (gpos - t_true) + rng.normal(0, pos_noise, size=2)
        # 机载侧观测 = 全局签名 + 传感器噪声（保留与全局的对应关系）
        base = np.asarray(global_.nodes[nid].elev_hist, dtype=np.float64)
        noisy = base + rng.normal(0, 0.02, size=16)
        n.elev_hist = noisy / np.linalg.norm(noisy)
    # 边是几何衍生属性：坐标（已含观测噪声）修改后整体重算，
    # 保证 _adj 反向边副本与 edges 一致（recompute_edges 原子重建全部边）
    local.recompute_edges()
    local.update_topology_attributes()
    return local, r_true, t_true, ids


def test_top1_correctness_monte_carlo():
    """A10 验收：已知真值下 top-1 正确率 ≥90%（50 次蒙特卡洛，无损毁场景）。"""
    matcher = GraphMatcher()
    rng = np.random.default_rng(42)
    ok = 0
    for trial in range(50):
        g = make_global_graph(rng, n=60, seed=trial)
        local, r_true, t_true, ids = make_local_graph(g, rng, size=5, seed=trial)
        hyps = matcher.match(local, g)
        assert len(hyps) > 0, f"trial {trial}: no hypothesis found"
        h = hyps[0]
        corr = dict(h.correspondences)
        correct = all(corr.get(i) == i for i in ids)
        pos_err = float(np.linalg.norm(h.xy - t_true))
        yaw_true = math.atan2(r_true[1, 0], r_true[0, 0])
        yaw_err = abs(wrap_pi(h.yaw - yaw_true))
        if correct and pos_err < 0.5 and yaw_err < math.radians(5.0):
            ok += 1
    rate = ok / 50
    print(f"\ntop-1 correct rate: {rate:.2f} ({ok}/50)")
    assert rate >= 0.9, f"top-1 correct rate {rate:.2f} < 0.9"


def test_no_match_below_min_nodes():
    """备案：n < min_corr 时不匹配，只累积窗口（系统靠 LIO 继续跑）。"""
    matcher = GraphMatcher()
    rng = np.random.default_rng(1)
    g = make_global_graph(rng, n=20, seed=1)
    local = g.subgraph([0, 1])
    assert matcher.match(local, g) == []


def test_hypothesis_count_capped():
    """A11：top-M 假设数 ≤ M（M≤5）。"""
    matcher = GraphMatcher(MatchConfig(max_hypotheses=5))
    rng = np.random.default_rng(7)
    g = make_global_graph(rng, n=80, seed=7)
    local, _, _, _ = make_local_graph(g, rng, size=5, seed=7)
    hyps = matcher.match(local, g)
    assert len(hyps) <= 5


def test_sigma_present_and_finite():
    """A12：输出 (x,y,yaw) + 3×3 协方差。"""
    matcher = GraphMatcher()
    rng = np.random.default_rng(9)
    g = make_global_graph(rng, n=60, seed=9)
    local, _, _, _ = make_local_graph(g, rng, size=5, seed=9)
    hyps = matcher.match(local, g)
    assert len(hyps) > 0
    assert hyps[0].sigma.shape == (3, 3)
    assert np.all(np.isfinite(hyps[0].sigma))


def test_performance_n5000():
    """A10 性能基准：N=5000、n=5。C++ 目标 <200 ms；Python 参考实现宽松线 <5 s。"""
    rng = np.random.default_rng(11)
    g = make_global_graph(rng, n=5000, seed=11, k=3)
    local, _, _, _ = make_local_graph(g, rng, size=5, seed=11)
    matcher = GraphMatcher()
    t0 = time.perf_counter()
    hyps = matcher.match(local, g)
    dt = time.perf_counter() - t0
    assert len(hyps) > 0
    print(f"\nmatch N=5000 n=5: {dt * 1000:.1f} ms")
    assert dt < 5.0
