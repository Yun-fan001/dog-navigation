"""dr_offline —— 离线层（计划书 A1–A5，地面站/云端，Python）。

航拍/卫星正射图 → 语义分割 → 全局节点图 G_g + 节点目录 node_map.bin。
与 dr_match 共享核心数据结构（SemanticNode/NodeGraph）。
"""
from .preprocess.pipeline import Tile, tile_image
from .roadnet.graphify import extract_road_nodes, skeleton_to_graph
from .terrain.features import detect_ridge_saddle, elevation_signature
from .export.writer import (
    read_node_map,
    write_node_map,
    node_map_from_graph,
    graph_from_node_map,
)

__all__ = [
    "Tile",
    "tile_image",
    "extract_road_nodes",
    "skeleton_to_graph",
    "detect_ridge_saddle",
    "elevation_signature",
    "read_node_map",
    "write_node_map",
    "node_map_from_graph",
    "graph_from_node_map",
]

__version__ = "0.1.0"
