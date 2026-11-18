"""A1 影像预处理管线：正射校正检查、UTM 重投影、GSD 统一、切片。

切片规格（计划书 A1）：1024×1024，重叠 128 px。
验收：任意输入 GeoTIFF → 标准切片集，投影误差 <0.5 px。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

DEFAULT_TILE_SIZE = 1024
DEFAULT_OVERLAP = 128


@dataclass
class Tile:
    x0: int          # 原图像素坐标（左上角）
    y0: int
    data: np.ndarray

    @property
    def shape(self) -> Tuple[int, int]:
        return self.data.shape[:2]


def tile_image(
    img: np.ndarray,
    tile_size: int = DEFAULT_TILE_SIZE,
    overlap: int = DEFAULT_OVERLAP,
) -> List[Tile]:
    """按 (tile_size × tile_size, overlap px) 切分影像，返回 Tile 列表。

    边缘块允许不足 tile_size（不 padding，避免推理伪影），
    与相邻块保持 overlap 重叠。
    """
    if img.ndim != 2 and img.ndim != 3:
        raise ValueError(f"expected 2D or 3D image, got ndim={img.ndim}")
    h, w = img.shape[:2]
    if tile_size <= 0 or overlap < 0 or overlap >= tile_size:
        raise ValueError(f"invalid tile params: size={tile_size} overlap={overlap}")
    tiles: List[Tile] = []
    stride = tile_size - overlap
    for y0 in range(0, h, stride):
        for x0 in range(0, w, stride):
            data = img[y0 : min(y0 + tile_size, h), x0 : min(x0 + tile_size, w)]
            tiles.append(Tile(x0, y0, data))
    return tiles


def tile_positions(
    img_shape: Tuple[int, int],
    tile_size: int = DEFAULT_TILE_SIZE,
    overlap: int = DEFAULT_OVERLAP,
) -> List[Tuple[int, int]]:
    """只返回各切片左上角坐标（供并行推理调度）。"""
    h, w = img_shape[:2]
    stride = tile_size - overlap
    return [(x0, y0) for y0 in range(0, h, stride) for x0 in range(0, w, stride)]


# --- GeoTIFF 读写（rasterio，可选依赖；缺失时仅切片逻辑可用） ---

def load_geotiff(path: str):
    """读取 GeoTIFF → (img, transform, crs)。需要 rasterio。"""
    import rasterio  # lazy import：无 rasterio 环境仍可用纯 numpy 切片

    with rasterio.open(path) as src:
        img = src.read()
        return img, src.transform, src.crs


def unify_gsd(src_path: str, dst_path: str, target_gsd: float = 0.3) -> None:
    """GSD 统一重采样（A1）。需要 rasterio。

    重投影/重采样到目标 GSD，写出 GeoTIFF。
    """
    import rasterio
    from rasterio.enums import Resampling
    from rasterio.warp import calculate_default_transform, reproject

    with rasterio.open(src_path) as src:
        transform, width, height = calculate_default_transform(
            src.crs, src.crs, src.width, src.height,
            left=src.bounds.left, bottom=src.bounds.bottom,
            right=src.bounds.right, top=src.bounds.top,
            resolution=target_gsd,
        )
        kwargs = src.meta.copy()
        kwargs.update(
            transform=transform, width=width, height=height,
            crs=src.crs, resampling=Resampling.bilinear,
        )
        with rasterio.open(dst_path, "w", **kwargs) as dst:
            for i in range(1, src.count + 1):
                reproject(
                    source=rasterio.band(src, i),
                    destination=rasterio.band(dst, i),
                    src_transform=src.transform, src_crs=src.crs,
                    dst_transform=transform, dst_crs=src.crs,
                    resampling=Resampling.bilinear,
                )
