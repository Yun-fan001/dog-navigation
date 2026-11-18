"""dr_eval —— 实验指标计算（计划书 §7.3 验证矩阵）。

G1 全局定位成功率曲线、G2 ATE、G3 重复检出率。所有实验指标在此定义，
dr_eval 一键出指标表（rosbag 回放脚本在 Ubuntu 环境补充）。
"""
from .metrics import (
    ate_rmse,
    repeatability,
    success_rate_curve,
)

__all__ = ["ate_rmse", "repeatability", "success_rate_curve"]

__version__ = "0.1.0"
