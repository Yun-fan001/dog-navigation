# dr_fusion —— 因子图融合（计划书 B3–B5，创新点 B 骨架）

**平面严格对准、垂直弹性松弛**：全局匹配只输出 (x, y, yaw) 三自由度软约束注入 GTSAM iSAM2，z/roll/pitch 完全交给 FAST-LIO2 与腿式运动学。

## 目录

| 路径 | 内容 | 状态 |
|---|---|---|
| `math/dr_fusion_math/` | **SE(2) 投影因子数学参考（纯 Python/numpy）** | ✅ 可测 |
| `math/tests/` | B3 验收：残差/雅可比数值微分校验（中心差分） | ✅ 通过 |
| `include/dr_fusion/` | GTSAM C++ 自定义因子接口（冻结） | ⏳ Ubuntu 实现 |
| （待建）`src/` | GTSAM C++ 实现 + MaxMixture/B4 协方差调度 | ⏳ Ubuntu |

## B3 验收：数值微分校验

```powershell
python -m pytest dr_ws/src/dr_fusion/math/tests -v
```

- 解析雅可比 vs 中心差分数值雅可比：20 组随机位姿（任意 roll/pitch/yaw/z），atol 1e-6；
- 性质测试：z 平移不影响残差（∂r/∂vz = 0）；水平位姿下 roll/pitch 微扰一阶无影响；
- C++ 实现落地后用**同一组测试数据**交叉验证（golden reference 纪律）。

## 数学要点（B3）

```
残差  r(T, m) = [ x(T)−x_m, y(T)−y_m, wrap(yaw(T)−yaw_m) ] ∈ R³
雅可比 ∂r/∂ξ：SE(3) 左扰动 ξ=[v,ω]
  平移：∂[x,y]/∂ξ = [I₂, −[t]×] 的前两行（∂r/∂vz ≡ 0）
  旋转：∂yaw/∂ω_k = (R00·(G_k R)[1,0] − R10·(G_k R)[0,0]) / (R00²+R10²)
```

## B4/B5 落地要点（Ubuntu）

1. 协方差自适应：score → Σ 映射已冻结于 `dr_match/py/dr_match/confidence.py`（B4），
   GTSAM 侧直接消费；score 低于 min_score 不注入因子（NGPS 式平滑退化，系统回纯 LIO 无跳变）；
2. 鲁棒化：Huber / DCS 开关；错误对齐只能降权不能带飞轨迹（**永不硬切换**）；
3. 多假设：top-M（M≤5）以 `gtsam::MaxMixtureFactor` 注入，由后续观测自然消歧（A11）；
4. 持续锚定：对齐成功后每个新匹配节点注入新先验因子（§3.5 持续漂移抑制）。
