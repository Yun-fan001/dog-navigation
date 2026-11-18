"""A3 骨架化（skimage.morphology.skeletonize 的薄封装）。

计划书选型：道路掩膜 → 形态学骨架化（skimage.skeletonize）。
"""
from __future__ import annotations

import numpy as np


def skeletonize(mask: np.ndarray) -> np.ndarray:
    """二值掩膜 → 单像素宽骨架（bool 数组，与原图同形）。"""
    from skimage.morphology import skeletonize as _skel  # lazy import

    m = np.asarray(mask)
    if m.dtype != bool:
        m = m > 0
    return _skel(m)
