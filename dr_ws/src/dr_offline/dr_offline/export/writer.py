"""A5 节点目录导出：NodeGraph → node_map.bin（protobuf）。

文件格式：magic(9B) + version(1B) + payload_len(4B LE) + protobuf payload。
Schema：dr_msgs/proto/node_map.proto（接口冻结，docs/interfaces.md §1）。
"""
from __future__ import annotations

import math
import struct
import time
from typing import Tuple

import numpy as np

from dr_match.graph import Edge, NodeGraph
from dr_match.node import NodeClass, SemanticNode
from dr_msgs import node_map_pb2

MAGIC = b"DRNODEMAP"
FORMAT_VERSION = 1


def write_node_map(nm: node_map_pb2.NodeMap, path: str) -> None:
    blob = nm.SerializeToString()
    with open(path, "wb") as f:
        f.write(MAGIC)
        f.write(struct.pack("<B", FORMAT_VERSION))
        f.write(struct.pack("<I", len(blob)))
        f.write(blob)


def read_node_map(path: str) -> node_map_pb2.NodeMap:
    with open(path, "rb") as f:
        magic = f.read(len(MAGIC))
        if magic != MAGIC:
            raise ValueError(f"bad magic: {magic!r}")
        ver = struct.unpack("<B", f.read(1))[0]
        if ver != FORMAT_VERSION:
            raise ValueError(f"unsupported version {ver}, expected {FORMAT_VERSION}")
        (n,) = struct.unpack("<I", f.read(4))
        blob = f.read(n)
        if len(blob) != n:
            raise ValueError("truncated payload")
    nm = node_map_pb2.NodeMap()
    nm.ParseFromString(blob)
    return nm


def node_map_from_graph(
    graph: NodeGraph,
    gsd: float = 0.3,
    projection: str = "",
    origin_xy: Tuple[float, float] = (0.0, 0.0),
    version: str = "0.1.0",
) -> node_map_pb2.NodeMap:
    """NodeGraph → NodeMap protobuf（含全部签名字段，A5 验收）。"""
    nm = node_map_pb2.NodeMap()
    nm.version = version
    nm.timestamp = int(time.time())
    nm.projection = projection
    nm.origin_x, nm.origin_y = float(origin_xy[0]), float(origin_xy[1])
    nm.gsd = float(gsd)
    for nid, n in graph.nodes.items():
        p = nm.nodes.add()
        p.id = int(nid)
        p.cls = int(n.cls)
        p.conf = float(n.conf)
        p.pos_x = float(n.pos_xy[0])
        p.pos_y = float(n.pos_xy[1])
        p.yaw_hint = float(n.yaw_hint)  # NaN 合法（无方向）
        p.elev_hist.extend(np.asarray(n.elev_hist, dtype=np.float32))
        p.roughness = float(n.roughness)
        p.degree = int(n.degree)
        p.edge_angles.extend(float(a) for a in n.edge_angles)
    for e in graph.edges:
        pe = nm.edges.add()
        pe.src = int(e.src)
        pe.dst = int(e.dst)
        pe.length = float(e.length)
        pe.dtheta = float(e.dtheta)
    return nm


def graph_from_node_map(nm: node_map_pb2.NodeMap) -> NodeGraph:
    """NodeMap protobuf → NodeGraph（机载层加载用）。"""
    g = NodeGraph()
    for p in nm.nodes:
        g.add_node(SemanticNode(
            id=p.id,
            cls=NodeClass(p.cls),
            conf=p.conf,
            pos_xy=np.array([p.pos_x, p.pos_y]),
            yaw_hint=p.yaw_hint if math.isfinite(p.yaw_hint) else math.nan,
            elev_hist=np.asarray(p.elev_hist, dtype=np.float64),
            roughness=p.roughness,
            degree=p.degree,
            edge_angles=list(p.edge_angles),
        ))
    for e in nm.edges:
        g.add_edge(Edge(e.src, e.dst, e.length, e.dtheta))
    return g
