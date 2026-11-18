# dr_active —— 主动探头 + 路径微调（计划书 C1–C3，创新点 C）

BehaviorTree.CPP 运行时（Ubuntu 引入，禁止从零写）；**行为树 XML 已冻结**（`bt/*.xml`）。

## 行为原语（§5.1）

| 原语 | 触发条件 | 动作 | 代价 |
|---|---|---|---|
| `LookAhead` | 距预测节点 <10 m | 机身转向/抬头对准预测节点方位 | ~0 s（行进中） |
| `StopAndScan` | 匹配置信度低 | 原地 360° 扫视，多视角聚合分割 | ~6 s |
| `StandUp` | 遮挡严重 | 后腿站立抬升视点 ~40 cm | ~3 s，耗稳定裕度 |
| `Detour` | next-best-node 收益为正 | 路径微调（≤15 m 绕行） | 绕行时间 |

## next-best-node 评分（C2）

```
score(i) = α · ΔI(i) / (1 + cost(i))
ΔI(i)   = 当前未匹配节点数 × P(节点i可确认) × 节点i属性独特度
cost(i) = 绕行距离 / 平均速度 + 地形代价（perception_3d 可通行性图）
```

验收：决策延迟 <50 ms；主动/被动一键切换（rosbag 可复现，C3）。

## 备案（§5.1）

信息增益计算吃紧 → 降级为**脚本化扫视**（每个语义候选区固定 StopAndScan），保留大部分收益。

## 接口

- 输入：`/align_event`（匹配置信度）、G_l/G_g 匹配状态
- 输出：`/behavior_cmd`（waypoint 微调 / 扫视指令）→ dddmr 规划器/控制器
- C++ 节点实现 + BehaviorTree.CPP 集成：Ubuntu 环境（P5 阶段，计划书第 16–19 周）
