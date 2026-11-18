# dr_match —— 图匹配与全局对齐（计划书 A10–A12）

**创新点 A 的核心模块**：把机载层局部节点图 G_l（3–8 节点滑动窗口）与离线层全局节点图 G_g（可达数千节点）做属性子图匹配，输出 (x, y, yaw) + 协方差 + 置信度，供 dr_fusion 以软约束注入因子图。

## 算法主线（计划书 §3.4）

```
1. 候选生成：对 G_l 每个节点，按 cls + elev_hist 余弦相似度取 G_g top-K（K=20）
2. 种子选取：属性最独特的局部边对，枚举全局候选边对；边长（比例一致，米制 1:1）+ 边夹角过滤
3. 生长扩展：BFS 逐节点扩展，成对一致性 |Δd| < τ_d 且 |Δθ| < τ_θ
4. 几何验证：RANSAC SE(2)（n≤8 时自然退化为全枚举 ≈ VF2 小图精确解的简化等价物），
   内点数 ≥3 且 RMSE < 2 m 才接受
5. 置信度：score = w1·内点率 + w2·属性相似度均值 − w3·RMSE → 自适应协方差；
   保留 top-M 假设（A11，M≤5，供 GTSAM MaxMixture 消歧）
```

## 目录

| 路径 | 内容 |
|---|---|
| `py/dr_match/` | **Python 参考实现（本机可测）**，C++ 版的 golden reference |
| `py/tests/` | pytest 单测：A10 蒙特卡洛 top-1 ≥90%、N=5000 性能基准、A11/A12 |
| `include/dr_match/` | C++ 纯库接口头文件（无 ROS 依赖，接口与 Python 对齐） |
| `core/` | C++ 实现目录（Ubuntu + Eigen 构建，待部署阶段落地） |

## 快速验证

```powershell
cd D:\projects\dog-navigation
python -m pytest dr_ws/src/dr_match/py/tests -v
```

## 关键参数（MatchConfig）

| 参数 | 默认 | 说明 |
|---|---|---|
| top_k | 20 | 候选生成 K |
| tau_d | 2.0 m | 成对一致性边长容差（容纳 LIO 短程漂移） |
| tau_theta | 15° | 成对一致性转角容差 |
| ransac_thr / max_rmse | 2.0 m | 几何验证内点阈值 / 接受上限 |
| min_corr | 3 | 最少对应数（**n<3 不匹配**，只累积窗口） |
| max_hypotheses | 5 | A11 top-M（M≤5） |

## 备案路线（计划书 §3.4 备案）

- 种子生长误匹配率高 → 换 VF2 精确子图同构（n≤8 复杂度可接受）求全部候选解再加权；
- 属性区分度不足 → 节点描述子换 DINOv2 patch 特征（384 维），特征相似度替代语义类别；
- n<3 → 不匹配只累积窗口，创新点 B 保证系统靠 LIO 继续跑。

## 验收状态

- [x] A10：蒙特卡洛 top-1 ≥90%（50 次，测试断言）＋ 性能基准（Python <5 s / C++ 目标 <200 ms）
- [x] A11：多假设 top-M 封顶
- [x] A12：输出 (x,y,yaw)+Σ+score+correspondences，接口冻结（docs/interfaces.md）
