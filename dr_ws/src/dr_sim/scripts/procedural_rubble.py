"""程序化瓦砾堆生成器（计划书 §7.2）。

随机凸包堆叠 + 碎石贴图 → Gazebo SDF 模型（model.config + model.sdf + STL mesh）。
用于 S2/S3 灾景仿真场景：30–50%（S2）/ >70%（S3）建筑替换为瓦砾堆模型。
"""
from __future__ import annotations

import math
import struct
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
from scipy.spatial import ConvexHull


def _write_binary_stl(path: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    """写二进制 STL：80 B 头 + uint32 三角数 + 每三角 (normal+3v)×f32 + uint16。"""
    with open(path, "wb") as f:
        f.write(b"dr_sim procedural rubble".ljust(80, b"\x00"))
        f.write(struct.pack("<I", len(faces)))
        for tri in faces:
            a, b, c = (vertices[i] for i in tri)
            n = np.cross(b - a, c - a)
            norm = np.linalg.norm(n)
            n = n / norm if norm > 1e-12 else np.array([0.0, 0.0, 1.0])
            for v in (n, a, b, c):
                f.write(struct.pack("<3f", *v))
            f.write(struct.pack("<H", 0))


def _random_rock_vertices(
    center: np.ndarray,
    radii: np.ndarray,
    n_verts: int,
    rng: np.random.Generator,
) -> np.ndarray:
    """椭球状随机凸包顶点：随机方向 + 随机半径（带波动）。"""
    dirs = rng.normal(size=(n_verts, 3))
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    scale = rng.uniform(0.55, 1.0, size=(n_verts, 1))
    return center + dirs * radii * scale


def _rock_transform(
    rock: np.ndarray,
    rng: np.random.Generator,
    max_offset: float,
    sink: float,
) -> np.ndarray:
    """随机偏航旋转 + 水平平移 + 下沉（模拟压入地面，堆叠更稳）。"""
    yaw = rng.uniform(0, 2 * math.pi)
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    t = np.array([rng.uniform(-max_offset, max_offset),
                  rng.uniform(-max_offset, max_offset), 0.0])
    out = rock @ R.T + t
    out[:, 2] -= float(out[:, 2].min()) - sink  # 最低点压到 sink 高度
    return out


def generate_rubble_pile(
    seed: int = 0,
    n_rocks: int = 8,
    out_dir: str = ".",
    model_name: str = "rubble_pile",
    rock_size: Tuple[float, float, float] = (0.4, 0.4, 0.25),
    spread: float = 0.35,
    sink: float = 0.05,
) -> List[Path]:
    """生成一堆瓦砾的 Gazebo 模型。

    参数：
      rock_size：单块碎石平均半径 (m)；spread：堆叠散布半径 (m)；
      sink：碎石压入地面深度 (m)。
    返回 mesh 文件路径列表。
    """
    out = Path(out_dir)
    mesh_dir = out / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed)

    meshes: List[Path] = []
    for i in range(n_rocks):
        n_verts = int(rng.integers(8, 18))
        center = np.array([rng.uniform(-spread, spread),
                           rng.uniform(-spread, spread),
                           rock_size[2] * rng.uniform(0.3, 0.8)])
        verts = _random_rock_vertices(
            center,
            np.array([rock_size[0], rock_size[1], rock_size[2]]),
            n_verts, rng,
        )
        verts = _rock_transform(verts, rng, max_offset=spread, sink=sink)
        hull = ConvexHull(verts)
        stl = mesh_dir / f"rock_{i:02d}.stl"
        _write_binary_stl(stl, verts, hull.simplices)
        meshes.append(stl)

    _write_sdf(out / "model.sdf", model_name, len(meshes))
    _write_model_config(out / "model.config", model_name)
    return meshes


def _write_sdf(path: Path, model_name: str, n_rocks: int) -> None:
    parts = []
    for i in range(n_rocks):
        uri = f"model://{model_name}/meshes/rock_{i:02d}.stl"
        parts.append(f"""      <collision name="rock_{i}">
        <geometry><mesh><uri>{uri}</uri></mesh></geometry>
      </collision>
      <visual name="rock_{i}_visual">
        <geometry><mesh><uri>{uri}</uri></mesh></geometry>
        <material><diffuse>0.45 0.42 0.40 1</diffuse></material>
      </visual>""")
    sdf = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="body">
{chr(10).join(parts)}
    </link>
  </model>
</sdf>
"""
    path.write_text(sdf, encoding="utf-8")


def _write_model_config(path: Path, model_name: str) -> None:
    cfg = f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>dog-navigation dr_sim</name>
  </author>
  <description>Procedural rubble pile (random convex hulls) for disaster scenarios S2/S3.</description>
</model>
"""
    path.write_text(cfg, encoding="utf-8")
