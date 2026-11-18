"""Pose3 → SE(2) 投影先验因子（计划书 §4.1 / B3）。

创新点 B 的数学核心：全局匹配输出 (x, y, yaw) 三自由度测量，
因子只约束机体的 x/y/yaw，z/roll/pitch 永远不被全局因子触碰。

残差定义：
    r(T, m) = [x(T) − x_m, y(T) − y_m, wrap(yaw(T) − yaw_m)]
雅可比为对 SE(3) **左扰动** ξ = [v, ω] 的解析导数；单测用
中心差分数值微分校验（B3 验收标准）。
"""
from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def skew(w: np.ndarray) -> np.ndarray:
    """3 向量 → so(3) 反对称矩阵。"""
    w = np.asarray(w, dtype=np.float64)
    return np.array([
        [0.0, -w[2], w[1]],
        [w[2], 0.0, -w[0]],
        [-w[1], w[0], 0.0],
    ])


def exp_se3(xi: np.ndarray) -> np.ndarray:
    """SE(3) 指数映射（罗德里格斯公式）。ξ = [v, ω]，返回 4×4 齐次矩阵。"""
    xi = np.asarray(xi, dtype=np.float64)
    v, w = xi[:3], xi[3:]
    theta = float(np.linalg.norm(w))
    W = skew(w)
    if theta < 1e-12:
        R = np.eye(3) + W + 0.5 * (W @ W)
        V = np.eye(3) + 0.5 * W
    else:
        W2 = W @ W
        R = np.eye(3) + (math.sin(theta) / theta) * W + ((1.0 - math.cos(theta)) / theta**2) * W2
        V = np.eye(3) + ((1.0 - math.cos(theta)) / theta**2) * W + ((theta - math.sin(theta)) / theta**3) * W2
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = V @ v
    return T


def se2_from_pose3(T: np.ndarray) -> Tuple[float, float, float]:
    """4×4 SE(3) → (x, y, yaw)。yaw = atan2(R[1,0], R[0,0])。"""
    T = np.asarray(T, dtype=np.float64)
    x, y = float(T[0, 3]), float(T[1, 3])
    yaw = math.atan2(T[1, 0], T[0, 0])
    return x, y, yaw


def se2_projection_error(T: np.ndarray, meas: np.ndarray) -> np.ndarray:
    """残差 r(T, m) ∈ R³。

    meas = [x_m, y_m, yaw_m]（全局匹配输出，dr_match AlignResult）。
    注意：z/roll/pitch 不参与残差——垂直方向弹性松弛。
    """
    x, y, yaw = se2_from_pose3(T)
    return np.array([
        x - float(meas[0]),
        y - float(meas[1]),
        wrap_pi(yaw - float(meas[2])),
    ])


# ∂R/∂ω_k 的生成元：R' = Exp(ω) R ≈ (I + [ω]×) R
_G1 = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
_G2 = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]])
_G3 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
_GENERATORS = (_G1, _G2, _G3)


def se2_projection_jacobian(T: np.ndarray) -> np.ndarray:
    """∂r/∂ξ ∈ R³ˣ⁶（SE(3) 左扰动）。

    平移部分：t' = t + v + ω×t → ∂[x,y]/∂ξ = [I2, −[t]×] 的前两行。
    旋转部分：yaw = atan2(R10, R00)，∂yaw/∂ω_k =
        (R00·(Gk R)[1,0] − R10·(Gk R)[0,0]) / (R00² + R10²)。
    z/roll/pitch 对应列为零（自由）。
    """
    T = np.asarray(T, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]

    J = np.zeros((3, 6))
    J[0, 0] = 1.0  # ∂x/∂vx
    J[1, 1] = 1.0  # ∂y/∂vy
    # ∂x/∂ω = −[t]× 的第一行 = [0, t_z, −t_y]
    J[0, 3:] = [0.0, t[2], -t[1]]
    J[1, 3:] = [-t[2], 0.0, t[0]]

    denom = R[0, 0] ** 2 + R[1, 0] ** 2
    if denom > 1e-12:
        for k, Gk in enumerate(_GENERATORS):
            dR = Gk @ R
            J[2, 3 + k] = (R[0, 0] * dR[1, 0] - R[1, 0] * dR[0, 0]) / denom
    return J
