"""pytest 全局配置：把仓库内 Python 纯库目录加入 sys.path。

这样测试可直接 `import dr_match / dr_offline / dr_fusion_math / dr_eval`，
无需 pip install -e。
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent

_PY_PATHS = [
    "dr_ws/src/dr_match/py",          # dr_match（图匹配参考实现）
    "dr_ws/src/dr_offline",           # dr_offline（离线层）
    "dr_ws/src/dr_fusion/math",       # dr_fusion_math（因子数学参考）
    "dr_ws/src/dr_eval",              # dr_eval（指标计算）
    "dr_ws/src/dr_msgs/python",       # dr_msgs（protobuf 生成绑定）
    "dr_ws/src/dr_sim/scripts",       # dr_sim（瓦砾生成器等脚本）
]

for rel in _PY_PATHS:
    p = ROOT / rel
    if p.is_dir() and str(p) not in sys.path:
        sys.path.insert(0, str(p))
