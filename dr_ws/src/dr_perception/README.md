# dr_perception —— 机载感知（计划书 A6–A9，部署目标 C++/TensorRT）

| 任务 | 内容 | 验收 |
|---|---|---|
| A6 | 机载轻量分割：SegFormer-B0 → TensorRT FP16，4 类（道路/水域/硬质场地/其他），512×512，关键帧触发（位移 2 m 或转向 15°） | Orin Nano ≥10 Hz，道路 IoU ≥65% |
| A7 | 局部高程图：grid_map，LIO 点云 → 5 cm 分辨率 50×50 m 滑窗，高程直方图/粗糙度签名 | 与离线 DEM 同区域签名余弦 ≥0.8（仿真） |
| A8 | 机载节点检测：①分割掩膜道路分支检测（骨架化找度≠2 点，算法复用 dr_offline/roadnet）②高程图极值/鞍部 ③语义突变点；DBSCAN(eps=3 m) 聚类 | 节点重复检出率 ≥60%（Gate Review 1 关键指标） |
| A9 | 局部队列：滑动窗口保留最近 8 节点，边 = LIO 相对位姿 → G_l(t) | 边长误差 ≤2% |

## 实现复用

- **A8 的骨架化/图化/地形检测**直接复用 `dr_offline` 的纯 Python 实现（roadnet/graphify.py、terrain/features.py）——机载 C++ 版以它们为 golden reference；
- G_l(t) 维护与匹配输入格式：复用 `dr_match` 的 `NodeGraph` 结构（C++ 版 `include/dr_match/matcher.h`）；
- 关键帧触发逻辑与 DBSCAN 聚类参数见 `config/`（待 Ubuntu 环境冻结）。

## 部署

- ROS1 Noetic 包 + grid_map（ANYbotics，fork 配置）+ TensorRT（Orin Nano）；
- 分割模型导出：SegFormer-B0 ONNX → TensorRT（训练脚本在 dr_offline/seg，GPU 环境）。
