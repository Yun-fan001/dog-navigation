# 开发指南（dog-navigation）

计划书版本：v1.0（`docs/项目计划书.md`）。本文是落地工程指引。

## 1. 双环境开发策略

| 环境 | 用途 | 工具链 |
|---|---|---|
| Windows 本机（当前） | 核心算法原型与单测：dr_match 图匹配、dr_fusion 因子数学、dr_offline 离线层、dr_eval 指标、dr_sim 资产生成 | Python 3.10+（见 requirements-dev.txt） |
| Ubuntu 20.04 | ROS1 Noetic 全栈：dddmr + Go2 Gazebo 仿真、FAST-LIO2、GTSAM C++ 纯库、BehaviorTree.CPP、TensorRT 机载部署 | ROS Noetic / catkin / CMake / GTSAM / CUDA |

**原则（计划书 §6.3）**：核心算法写成无 ROS 依赖的纯库；Python 版本是 C++ 实现的**参考实现（golden reference）**——C++ 版落地时用同一组单测数据交叉验证。

## 2. 快速开始（Windows）

```powershell
cd D:\projects\dog-navigation
python -m pip install -r requirements-dev.txt
# 生成 protobuf 绑定（修改 node_map.proto 后需重新执行）
python -m grpc_tools.protoc -I dr_ws/src/dr_msgs/proto --python_out dr_ws/src/dr_msgs/python/dr_msgs node_map.proto
# 全量单测
python -m pytest
```

## 3. 模块开工顺序（计划书 §6.4-C.3）

每个模块固定流程：**跑通官方 demo → 嵌入本项目接口 → 替换创新逻辑 → 过验收标准**。

| 顺序 | 模块 | 本轮准备状态 |
|---|---|---|
| 1 | dr_match（A10–A12） | ✅ Python 参考实现 + 单测 |
| 2 | dr_fusion 数学（B3） | ✅ SE(2) 投影因子残差/雅可比 + 数值微分校验 |
| 3 | dr_offline（A1–A5） | ✅ 核心逻辑 + 单测（seg 训练待 RescueNet/xBD 数据与 GPU） |
| 4 | dr_eval / dr_sim / dr_active | ✅ 可测部分就绪 |
| 5 | dr_lio / dr_perception / ROS 集成 | ⏳ Ubuntu 环境（fork 配置，禁止从零写） |

## 4. 测试 ↔ 验收标准映射

计划书每个任务的验收标准直接写成测试断言：

| 计划书验收 | 测试位置 |
|---|---|
| A3 节点检出率 ≥85%、虚警 ≤15%、位置误差 ≤2 px | `dr_offline/tests/test_roadnet.py` |
| A4 地形点在已知 DEM 上位置误差 ≤1 像元 | `dr_offline/tests/test_terrain.py` |
| A5 目录 round-trip 完整 | `dr_offline/tests/test_export.py` |
| A10 匹配 top-1 正确（已知真值）、单次 <200 ms（N=5000 基准） | `dr_match/py/tests/test_matcher.py` |
| A12 输出 (x,y,yaw)+Σ | `dr_match/py/tests/test_matcher.py` |
| B3 因子残差/雅可比数值微分校验通过 | `dr_fusion/math/tests/test_se2_projection.py` |
| E2 ATE 计算 | `dr_eval/tests/test_metrics.py` |
| G3 重复检出率统计 | `dr_eval/tests/test_metrics.py` |
| S2 瓦砾资产可加载 | `dr_sim/tests/test_procedural_rubble.py` |

## 5. Ubuntu 20.04 部署步骤（P0 剩余项，后续执行）

```bash
# 1. ROS1 Noetic + catkin 环境
sudo apt install ros-noetic-desktop-full
# 2. 依赖
sudo apt install libgtsam-dev libeigen3-dev libyaml-cpp-dev protobuf-compiler
# 3. workspace
cd dog-navigation/dr_ws && catkin_make  # 或 catkin build
# 4. 外部栈（fork 配置，禁止从零写）
#    dddmr_navigation   github.com/dfl-rlab/dddmr_navigation   → 导航全栈/Go2 Gazebo/mcl_3dl 基线
#    FAST_LIO           github.com/hku-mars/FAST_LIO           → dr_lio（改 Mid-360 配置与外参）
#    grid_map           github.com/ANYbotics/grid_map           → dr_perception 高程图
#    BehaviorTree.CPP   github.com/BehaviorTree/BehaviorTree.CPP → dr_active
#    SegFormer          github.com/NVlabs/SegFormer             → 分割（微调/导出 TRT）
# 5. 验收：S1 场景纯 LIO 闭环（P0 出口标准）
```

## 6. 分支与 CI 纪律（§6.3）

- 分支：`main`（可复现基线）← `dev` ← feature 分支；
- 每次合并跑仿真冒烟测试（10 min 场景，LIO 漂移与对齐成功率不劣化）；
- Python 单测在 Windows/Ubuntu CI 双跑（`.github/workflows/python-tests.yml`）。

## 7. 关键设计决策速查

1. **不硬切换，只弹性松弛**（B4）：全局匹配以 (x,y,yaw) 先验因子注入，协方差由置信度自适应；错误对齐只能降权不能带飞轨迹。
2. **n<3 不匹配**：图匹配窗口凑不齐 3 个可靠节点时只累积窗口，系统靠 LIO 继续跑。
3. **节点定义禁用"建筑"语义**（创新点 A）：灾后易碎类只作辅助，主力是路网拓扑顶点与地形特征。
4. **Gate Review 1（第 8 周）**：S2 场景节点重复检出率 ≥60%，否则触发 §3.6 备案（地形剖面 TERCOM 式匹配降级）。
