"""test_roadnet.py — A3 验收：路网节点检出率 ≥85%、虚警 ≤15%、位置误差 ≤2 px。"""
import numpy as np
import pytest

from dr_offline.roadnet.graphify import extract_road_nodes


def _draw_line(mask: np.ndarray, p0, p1, width: int = 3) -> None:
    """在掩膜上画线段（含端点）。"""
    x0, y0 = p0
    x1, y1 = p1
    n = max(abs(x1 - x0), abs(y1 - y0)) * 2 + 1
    for t in np.linspace(0, 1, n):
        cx = int(round(x0 + (x1 - x0) * t))
        cy = int(round(y0 + (y1 - y0) * t))
        r = width // 2
        mask[max(0, cy - r):cy + r + 1, max(0, cx - r):cx + r + 1] = 1


def make_test_mask() -> tuple:
    """合成路网：十字路口 + 断头路端点 + T 路口 + L 急弯。

    返回 (mask, ground_truth_nodes)。真值节点（像素坐标）：
    十字中心 (50,50)、主路端点 (5,50)/(95,50)、竖路端点 (50,5)/(50,95)、
    T 路口 (50,70) 及其支路端点 (20,70)、L 急弯 (70,50) 及其端点 (70,90)。
    断头路端点属于合法节点（★★★ 类别之一）。
    """
    mask = np.zeros((100, 100), dtype=bool)
    # 水平主路 + 垂直主路（十字）
    _draw_line(mask, (5, 50), (95, 50), width=3)
    _draw_line(mask, (50, 5), (50, 95), width=3)
    # T 路口支路（连接垂直路 x=50, y=70）
    _draw_line(mask, (20, 70), (50, 70), width=3)
    # L 急弯（右上）
    _draw_line(mask, (70, 50), (70, 90), width=3)
    gt = [(50, 50), (5, 50), (95, 50), (50, 5), (50, 95),
          (50, 70), (20, 70), (70, 50), (70, 90)]
    return mask, gt


def test_road_nodes_detection_rate_and_accuracy():
    """A3 验收：检出率 ≥85%，位置误差 ≤2 px。"""
    mask, gt = make_test_mask()
    g = extract_road_nodes(mask, gsd=0.3, cluster_eps_m=3.0)  # 3 m / 0.3 = 10 px 聚类
    nodes = g.nodes
    assert len(nodes) > 0

    detected = [n.pos_xy / 0.3 for n in nodes.values()]  # 回到像素
    detected = np.asarray(detected)
    hit = 0
    for (gx, gy) in gt:
        d = np.linalg.norm(detected - np.array([gx, gy]), axis=1)
        if d.min() <= 2.0:  # 位置误差 ≤2 px
            hit += 1
    rate = hit / len(gt)
    print(f"\nroad node detection rate: {hit}/{len(gt)}")
    assert rate >= 0.85, f"detection rate {rate:.2f} < 0.85"


def test_road_nodes_false_alarm_rate():
    """A3 验收：虚警 ≤15%（距真值 >3 px 的检出节点占比）。"""
    mask, gt = make_test_mask()
    g = extract_road_nodes(mask, gsd=0.3, cluster_eps_m=3.0)
    detected = np.asarray([n.pos_xy / 0.3 for n in g.nodes.values()])
    gt_arr = np.asarray(gt)
    far = 0
    for p in detected:
        if np.linalg.norm(gt_arr - p, axis=1).min() > 3.0:
            far += 1
    n = len(detected)
    assert far / n <= 0.15, f"false alarm rate {far}/{n} > 15%"


def test_degrees_of_intersections():
    """拓扑签名：十字路口度 ≈4，T 路口度 ≈3。"""
    mask, _ = make_test_mask()
    g = extract_road_nodes(mask, gsd=0.3, cluster_eps_m=3.0)
    degs = {tuple(np.round(n.pos_xy / 0.3, 0)): n.degree for n in g.nodes.values()}
    # 十字中心附近节点的度应为 4（4 条出边）
    assert any(abs(k[0] - 50) <= 2 and abs(k[1] - 50) <= 2 and v == 4 for k, v in degs.items())
    # T 路口附近节点的度应为 3
    assert any(abs(k[0] - 50) <= 2 and abs(k[1] - 70) <= 2 and v == 3 for k, v in degs.items())


def test_empty_mask():
    g = extract_road_nodes(np.zeros((50, 50), dtype=bool))
    assert len(g.nodes) == 0
