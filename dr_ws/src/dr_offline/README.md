# dr_offline —— 离线层（计划书 A1–A5）

地面站/云端运行的全局节点图构建管线。**当前为纯 Python 实现，本机可测。**

## 管线（对应计划书任务）

| 任务 | 模块 | 状态 |
|---|---|---|
| A1 影像预处理 | `preprocess/pipeline.py`：切片（1024×1024 / 128 px 重叠）、GSD 统一、UTM 重投影（rasterio） | ✅ 切片可测；重投影需真实 GeoTIFF 验证 |
| A2 灾后语义分割 | `seg/`：SegFormer-B2 在 RescueNet/xBD 微调 | ⏳ 待 GPU/数据集（Ubuntu/云端） |
| A3 路网提取与节点化 | `roadnet/`：skimage 骨架化 → 度≠2 聚类图化（复用 dr_match 数据结构） | ✅ 检出率/虚警/位置误差断言通过 |
| A4 地形节点提取 | `terrain/features.py`：山脊/鞍部检测 + 2.5D 高程签名 | ✅ 鞍部 ≤1 px 断言通过 |
| A5 节点目录导出 | `export/writer.py`：NodeGraph → node_map.bin（protobuf，含版本/时间戳/GSD/锚点） | ✅ round-trip 断言通过 |

## 运行测试

```powershell
python -m pytest dr_ws/src/dr_offline/tests -v
```

## 与机载层的接口

- `/node_map` 文件：`node_map.bin`（schema：`dr_msgs/proto/node_map.proto`，接口冻结 docs/interfaces.md §1）；
- A3 的骨架化/图化与 A4 的签名计算是 **A7/A8 机载节点检测的 golden reference**（dr_perception C++ 版按此实现）；
- 备案（§3.2）：灾后正射图不可得 → 灾前图 + xBD 损毁评估过滤易碎节点；分割崩溃 → SAM2 半自动；DEM 过粗 → ODM 摄影测量或纯路网节点。
