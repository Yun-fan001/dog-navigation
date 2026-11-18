"""test_se2_projection.py — B3 验收：Pose3→SE(2) 投影因子残差/雅可比数值微分校验。"""
import math

import numpy as np
import pytest

from dr_fusion_math.se2_projection import (
    exp_se3,
    se2_from_pose3,
    se2_projection_error,
    se2_projection_jacobian,
    wrap_pi,
)


def _random_pose3(rng: np.random.Generator) -> np.ndarray:
    """随机 SE(3)：任意 roll/pitch/yaw/平移（含 z）。"""
    T = np.eye(4)
    # 用欧拉角构造（保证正交性）
    roll = rng.uniform(-0.9, 0.9)
    pitch = rng.uniform(-0.9, 0.9)
    yaw = rng.uniform(-math.pi, math.pi)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = rng.uniform(-20, 20, size=3)
    return T


def _numerical_jacobian(T: np.ndarray, meas: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """中心差分数值雅可比：∂r/∂ξ（左扰动）。"""
    J = np.zeros((3, 6))
    for k in range(6):
        e = np.zeros(6)
        e[k] = eps
        T_plus = exp_se3(e) @ T
        T_minus = exp_se3(-e) @ T
        J[:, k] = (se2_projection_error(T_plus, meas) - se2_projection_error(T_minus, meas)) / (2 * eps)
    return J


def test_jacobian_matches_numerical_diff():
    """B3 验收：解析雅可比与数值微分一致（20 组随机位姿 + 随机测量）。"""
    rng = np.random.default_rng(0)
    for _ in range(20):
        T = _random_pose3(rng)
        meas = np.array([rng.uniform(-30, 30), rng.uniform(-30, 30),
                         rng.uniform(-math.pi, math.pi)])
        J_ana = se2_projection_jacobian(T)
        J_num = _numerical_jacobian(T, meas)
        assert np.allclose(J_ana, J_num, atol=1e-6), f"jacobian mismatch:\n{J_ana}\nvs\n{J_num}"


def test_z_roll_pitch_free():
    """创新点 B 核心性质：z 平移完全自由；水平位姿下 roll/pitch 对残差一阶无影响。"""
    T = _random_pose3(np.random.default_rng(1))
    meas = np.array([1.0, 2.0, 0.3])
    r0 = se2_projection_error(T, meas)
    J0 = se2_projection_jacobian(T)

    # z 平移不改变残差
    T1 = T.copy()
    T1[2, 3] += 5.0
    assert np.allclose(r0, se2_projection_error(T1, meas))

    # ∂r/∂vz = 0（z 平移列恒零）
    assert np.allclose(J0[:, 2], [0.0, 0.0, 0.0], atol=1e-12)

    # 水平位姿（纯 yaw 旋转）：∂yaw/∂ωx、∂yaw/∂ωy = 0 —— roll/pitch 微扰一阶无影响
    c, s = math.cos(0.7), math.sin(0.7)
    T_h = np.eye(4)
    T_h[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    J_h = se2_projection_jacobian(T_h)
    assert np.allclose(J_h[:, 3:5], 0.0, atol=1e-12)

    # 非水平位姿（有 pitch 时）ωx/ωy 列非零：roll 对全局 yaw 有一阶耦合，
    # 这是 SE(3) 的正确几何，解析式的正确性由数值微分测试保证
    assert not np.allclose(J0[:, 3:5], 0.0)


def test_residual_recovers_measurement():
    """残差零点：T 的 (x,y,yaw) 与测量一致时残差为 0（z/r/p 任意）。"""
    rng = np.random.default_rng(2)
    T = _random_pose3(rng)
    x, y, yaw = se2_from_pose3(T)
    meas = np.array([x, y, yaw])
    assert np.allclose(se2_projection_error(T, meas), 0.0, atol=1e-12)


def test_wrap_pi_in_residual():
    """yaw 残差 wrap 到 [−π, π]。"""
    T = _random_pose3(np.random.default_rng(3))
    x, y, yaw = se2_from_pose3(T)
    meas = np.array([x, y, yaw + 2 * math.pi])
    r = se2_projection_error(T, meas)
    assert abs(r[2]) < math.pi + 1e-12


def test_analytic_vs_numerical_near_identity():
    """退化位姿（单位阵，denom≈0 边界）的雅可比仍与数值一致。"""
    T = np.eye(4)
    meas = np.array([0.0, 0.0, 0.0])
    J_ana = se2_projection_jacobian(T)
    J_num = _numerical_jacobian(T, meas)
    assert np.allclose(J_ana, J_num, atol=1e-6)
