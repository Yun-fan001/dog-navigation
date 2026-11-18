"""dr_fusion_math —— 因子图数学参考实现（计划书 B3，无 ROS/GTSAM 依赖）。

作为 GTSAM C++ 自定义因子（dr_fusion/include）的 golden reference：
同一组测试数据交叉验证。B3 验收：残差/雅可比数值微分校验通过。
"""
from .se2_projection import (
    se2_from_pose3,
    se2_projection_error,
    se2_projection_jacobian,
    exp_se3,
    skew,
)

__all__ = [
    "se2_from_pose3",
    "se2_projection_error",
    "se2_projection_jacobian",
    "exp_se3",
    "skew",
]

__version__ = "0.1.0"
