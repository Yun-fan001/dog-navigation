# dr_sim —— Gazebo 仿真（计划书 §7.2 场景矩阵）

| 场景 | 内容 | 俯视先验 | 目的 |
|---|---|---|---|
| S1 完好城区 | 规则街区 | 与场景一致 | 管线贯通，方法上限 |
| S2 部分倒塌 | 30–50% 建筑替换为瓦砾堆 | **灾前图（不一致！）** | 核心测试：节点稳定性筛选 |
| S3 严重倒塌 | >70% 损毁 + 开阔瓦砾场 | 灾前图 | 压力测试 + 退化策略 |
| S4 烟尘低光 | 光照/能见度扰动 | 一致 | 传感器退化 + 腿式里程计兜底 |

## 瓦砾资产生成（✅ 本机可跑）

```powershell
python dr_ws/src/dr_sim/scripts/procedural_rubble.py    # 见脚本 argparse/示例
# 生成 Gazebo 模型：models/rubble_pile_<seed>/{model.config, model.sdf, meshes/*.stl}
# 随机凸包堆叠（scipy ConvexHull）+ 碎石 STL，同 seed 可复现
```

单测：`python -m pytest dr_ws/src/dr_sim/tests`。

## 俯视影像模拟器（待 Ubuntu）

- Gazebo 俯视相机渲染场景 → 灾前正射图（GSD 0.3 m 模拟卫星）；
- S2/S3 的"不一致先验"：先渲染完好场景存档，再程序化替换建筑为瓦砾后运行实验；
- dddmr_navigation 的 Go2 Gazebo 模型作为机器人载体（§7.2）。

## worlds/

场景世界文件在 Ubuntu + dddmr 集成后落地（S1–S4 各一个 .world + 生成脚本）。
