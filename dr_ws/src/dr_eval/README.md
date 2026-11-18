# dr_eval —— 实验指标与回放（计划书 §7.3 验证矩阵）

## 已实现（✅ 本机可测，纯 numpy）

| 实验 | 指标 | 函数 |
|---|---|---|
| E2 轨迹精度 | ATE（Umeyama 对齐后 RMSE） | `dr_eval.metrics.ate_rmse` |
| E1/G1 全局定位收敛 | 走 N 节点成功率曲线 | `dr_eval.metrics.success_rate_curve` |
| G3/Gate Review 1 | 节点重复检出率 | `dr_eval.metrics.repeatability` |

```powershell
python -m pytest dr_ws/src/dr_eval/tests -v
```

## 待 Ubuntu 补充

- rosbag 回放脚本（`scripts/`）：回放 `/odom_lio`、`/align_event`、真值 `/ground_truth_pose` → 一键出指标表；
- E1 蒙特卡洛驱动器：100 次随机初始位姿（≤30 m / ±180°）→ 成功率曲线（对接 dr_match）；
- E3 消融开关（去语义/去拓扑/去主动/硬切换）的配置与数据收集；
- E5 对照基线（mcl_3dl 原生 + 检索式粗定位）指标收集。
