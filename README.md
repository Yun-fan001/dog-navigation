# dog-navigation

> **灾害复杂地形下基于 2D 卫星/航拍影像的地面四足机器人 3D 定位与建图**
>
> 计划书 v1.0 见 [`docs/项目计划书.md`](docs/项目计划书.md)

抗险救灾场景中 GPS 拒止、地形剧变，地面四足机器人面临迷路/跑偏、视角低遮挡重、算力受限三大痛点。本项目利用灾后 24–72 h 内可获得的 2D 俯视影像（无人机正射/卫星，GSD ≈ 0.3 m）作为先验地图，为算力受限的四足机器人（Go2 + Jetson）提供全局定位锚定与建图辅助。

## 三个创新点

| # | 创新点 | 一句话 |
|---|---|---|
| A（核心） | 灾后稳定节点体系 + 图匹配全局对齐 | 路网拓扑顶点/地形特征等灾后稳定节点携带「语义类别 + 2.5D 高程签名 + 拓扑度数」复合描述子，狗走过 3–5 个节点后用属性子图匹配 + SE(2) 几何验证精确对齐 |
| B（骨架） | 平面严格对准、垂直弹性松弛 | 全局匹配只输出 (x, y, yaw) 三自由度软约束注入 GTSAM 因子图，z/roll/pitch 交给 LIO；置信度自适应协方差，匹配失效平滑退化回纯 LIO |
| C（增益） | 主动探头 + 路径微调 | next-best-node 信息增益准则做路径微调，配合机身姿态/云台主动确认节点签名 |

## 仓库结构

```
dog-navigation/
├── docs/                    # 计划书、开发指南、接口冻结表
├── requirements-dev.txt     # Python 开发依赖
├── pytest.ini / conftest.py # 单测配置（验收标准即断言）
└── dr_ws/src/               # ROS workspace 源码（monorepo，见计划书 §6.1）
    ├── dr_msgs/             # ROS msg 定义 + node_map.proto（节点目录 schema）
    ├── dr_offline/          # 离线层 A1–A5（Python）：影像预处理/路网图化/地形特征/节点目录导出
    ├── dr_perception/       # 机载感知 A6–A9（部署目标 C++/TensorRT，当前为接口冻结）
    ├── dr_match/            # 图匹配 A10–A12：C++ 无 ROS 纯库（Linux 构建）+ Python 参考实现（本机可测）
    ├── dr_fusion/           # 因子图 B3–B5：Pose3→SE(2) 投影因子数学（Python 参考 + 数值微分校验）
    ├── dr_lio/              # FAST-LIO2 fork 与配置说明（B1、B2）
    ├── dr_active/           # 行为树 XML 骨架（C1–C3 原语）
    ├── dr_sim/              # Gazebo 仿真：程序化瓦砾堆 SDF 生成器、世界、俯视影像模拟
    └── dr_eval/             # 实验指标计算（ATE/成功率曲线/重复检出率）
```

## 开发环境（双环境策略）

**算法原型开发在 Windows 本机即可完成，ROS 集成需 Ubuntu 20.04**（ROS1 Noetic，见计划书 §2.3 技术选型）。分阶段说明见 [`docs/dev-guide.md`](docs/dev-guide.md)。

### Windows 本机（算法开发，当前阶段）

```powershell
pip install -r requirements-dev.txt
python -m grpc_tools.protoc -I dr_ws/src/dr_msgs/proto --python_out dr_ws/src/dr_msgs/python/dr_msgs node_map.proto
pytest
```

### Ubuntu 20.04（ROS 全栈部署，后续阶段）

```bash
# catkin workspace：dr_ws/src 直接可作为 catkin src；步骤见 docs/dev-guide.md
```

## 当前状态（P0 基础设施 · 计划书第 1–3 周）

- [x] 仓库骨架与包结构（§6.1 冻结）
- [x] 核心数据结构：SemanticNode / NodeGraph / node_map.proto
- [x] dr_match 图匹配器 Python 参考实现（种子生长 + 成对一致性 + RANSAC SE(2)，A10–A12 算法主线）
- [x] dr_fusion Pose3→SE(2) 投影因子数学参考（残差/雅可比过数值微分校验，B3）
- [x] dr_offline 离线层核心（A1 切片、A3 路网图化、A4 地形特征、A5 导出）
- [x] dr_sim 程序化瓦砾堆 SDF 生成器
- [x] dr_active 行为树 XML 骨架
- [x] dr_eval 指标计算（ATE/成功率/重复检出率）
- [ ] C++ 纯库构建（dr_match/core、dr_fusion GTSAM 因子）—— 待 Ubuntu 环境
- [ ] dddmr_navigation + Go2 Gazebo 仿真跑通（S1 场景）—— 待 Ubuntu 环境
- [ ] FAST-LIO2 部署与标定 —— 待 Ubuntu 环境

## 开发纪律（计划书 §6.3，红线）

1. 核心算法（dr_match、dr_fusion 数学）**无 ROS 依赖**，ROS 只在外层 wrapper；
2. 每个任务配 pytest/GoogleTest 用例，**验收标准即测试断言**；
3. 禁止从零实现 FAST-LIO2、GTSAM 求解器、mcl_3dl、分割 backbone、BehaviorTree 运行时——一律 fork 配置；
4. 所有实验用 rosbag + 配置文件可复现，`dr_eval` 一键出指标表。
