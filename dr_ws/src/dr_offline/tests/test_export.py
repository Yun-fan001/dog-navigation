"""test_export.py — A5 验收：node_map.bin round-trip 完整（含全部签名字段）。"""
import math

import numpy as np
import pytest

from dr_match.graph import Edge, NodeGraph
from dr_match.node import NodeClass, SemanticNode
from dr_offline.export.writer import (
    graph_from_node_map,
    node_map_from_graph,
    read_node_map,
    write_node_map,
)


def make_graph() -> NodeGraph:
    g = NodeGraph()
    n0 = SemanticNode(
        id=0, cls=NodeClass.ROAD_JUNCTION, conf=0.95,
        pos_xy=np.array([10.0, 20.0]), yaw_hint=0.5,
        elev_hist=np.linspace(0, 1, 16), roughness=0.3,
    )
    n1 = SemanticNode(
        id=1, cls=NodeClass.TERRAIN_FEATURE, conf=0.8,
        pos_xy=np.array([30.0, 40.0]), yaw_hint=math.nan,  # NaN 方向
        elev_hist=np.zeros(16), roughness=0.0,
    )
    g.add_node(n0)
    g.add_node(n1)
    g.add_edge(Edge(0, 1, 28.3, 0.785))
    g.update_topology_attributes()
    return g


def _assert_graphs_equal(g1: NodeGraph, g2: NodeGraph):
    assert set(g1.nodes) == set(g2.nodes)
    for i in g1.nodes:
        a, b = g1.nodes[i], g2.nodes[i]
        assert a.cls == b.cls
        assert a.conf == pytest.approx(b.conf)
        assert np.allclose(a.pos_xy, b.pos_xy)
        if math.isnan(a.yaw_hint):
            assert math.isnan(b.yaw_hint)
        else:
            assert a.yaw_hint == pytest.approx(b.yaw_hint)
        assert np.allclose(a.elev_hist, b.elev_hist, atol=1e-6)
        assert a.roughness == pytest.approx(b.roughness, abs=1e-6)
        assert a.degree == b.degree
        assert np.allclose(a.edge_angles, b.edge_angles, atol=1e-6)
    assert len(g1.edges) == len(g2.edges)
    for e1, e2 in zip(g1.edges, g2.edges):
        assert (e1.src, e1.dst) == (e2.src, e2.dst)
        assert e1.length == pytest.approx(e2.length)
        assert e1.dtheta == pytest.approx(e2.dtheta)


def test_round_trip(tmp_path):
    """A5 验收：写 → 读 → 字段完整一致。"""
    g = make_graph()
    path = tmp_path / "node_map.bin"
    nm = node_map_from_graph(g, gsd=0.3, projection="EPSG:32650", origin_xy=(500.0, 300.0))
    assert nm.gsd == pytest.approx(0.3)
    assert nm.origin_x == pytest.approx(500.0)
    write_node_map(nm, str(path))
    nm2 = read_node_map(str(path))
    assert nm2.version == nm.version
    assert nm2.gsd == pytest.approx(0.3)
    g2 = graph_from_node_map(nm2)
    _assert_graphs_equal(g, g2)


def test_bad_magic_raises(tmp_path):
    path = tmp_path / "bad.bin"
    path.write_bytes(b"NOTAMAP!!!")
    with pytest.raises(ValueError):
        read_node_map(str(path))


def test_truncated_payload_raises(tmp_path):
    g = make_graph()
    path = tmp_path / "trunc.bin"
    nm = node_map_from_graph(g)
    write_node_map(nm, str(path))
    data = path.read_bytes()
    path.write_bytes(data[:-5])
    with pytest.raises(ValueError):
        read_node_map(str(path))


def test_empty_graph_round_trip(tmp_path):
    g = NodeGraph()
    path = tmp_path / "empty.bin"
    write_node_map(node_map_from_graph(g), str(path))
    nm = read_node_map(str(path))
    assert len(nm.nodes) == 0 and len(nm.edges) == 0
    assert len(graph_from_node_map(nm).nodes) == 0
