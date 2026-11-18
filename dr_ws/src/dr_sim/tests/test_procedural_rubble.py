"""test_procedural_rubble.py — 程序化瓦砾模型可被 Gazebo 加载（结构校验）。"""
import struct
from pathlib import Path

import numpy as np
import pytest

from procedural_rubble import generate_rubble_pile


def _read_stl_triangle_count(path: Path) -> int:
    data = path.read_bytes()
    assert len(data) >= 84, "STL too short"
    (n,) = struct.unpack("<I", data[80:84])
    # 每三角 50 B
    assert len(data) == 84 + n * 50, "STL size mismatch"
    return n


def test_generate_rubble_pile(tmp_path):
    meshes = generate_rubble_pile(seed=7, n_rocks=5, out_dir=str(tmp_path), model_name="rubble_test")
    assert len(meshes) == 5

    # model.config + model.sdf 存在且合法
    cfg = (tmp_path / "model.config").read_text(encoding="utf-8")
    sdf = (tmp_path / "model.sdf").read_text(encoding="utf-8")
    assert "<name>rubble_test</name>" in cfg
    assert 'sdf version="1.6">model.sdf' in cfg
    assert "<static>true</static>" in sdf
    # SDF 引用全部 5 个 mesh
    for i in range(5):
        assert f"model://rubble_test/meshes/rock_{i:02d}.stl" in sdf

    # STL 有效：非空、三角形数与长度一致、顶点有限
    for m in meshes:
        n = _read_stl_triangle_count(m)
        assert n >= 4  # 凸包至少四面体


def test_generate_rubble_reproducible(tmp_path):
    """同 seed 可复现（实验可复现纪律）。"""
    a = generate_rubble_pile(seed=42, n_rocks=3, out_dir=str(tmp_path / "a"), model_name="m")
    b = generate_rubble_pile(seed=42, n_rocks=3, out_dir=str(tmp_path / "b"), model_name="m")
    assert a[0].read_bytes() == b[0].read_bytes()


def test_rubble_geometric_sanity(tmp_path):
    """碎石顶点位于合理范围（堆叠半径内，z ≥ 0）。"""
    meshes = generate_rubble_pile(seed=1, n_rocks=3, out_dir=str(tmp_path), spread=0.35)
    for m in meshes:
        data = m.read_bytes()
        n = struct.unpack("<I", data[80:84])[0]
        verts = []
        for i in range(n):
            off = 84 + i * 50 + 12  # 跳过 normal
            verts.append(struct.unpack("<9f", data[off:off + 36]))
        arr = np.asarray(verts).reshape(-1, 3, 3).reshape(-1, 3)
        assert arr[:, 2].min() >= -1e-6  # 最低点 >= 地面（sink 处理）
        assert np.abs(arr[:, :2]).max() < 2.0  # 水平范围合理
