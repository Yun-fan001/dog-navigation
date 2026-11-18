# 接口冻结表（计划书 §6.2）

> 开发前冻结，变动需评审。以下为 v1.0 冻结版本。

## 1. 文件接口

| 接口 | 类型 | 内容 | 生产者 → 消费者 | Schema |
|---|---|---|---|---|
| `/node_map` | 文件 | 全局节点目录（含版本、时间戳、GSD、UTM 锚点） | 离线层 → 机载层（出发前加载） | `dr_msgs/proto/node_map.proto` |
| `/local_node` | topic | SemanticNode（局部队列系） | dr_perception → dr_match | `dr_msgs/msg/SemanticNode.msg` |
| `/align_hypothesis` | topic | `(x,y,yaw) + Σ + score + 假设id` | dr_match → dr_fusion | `dr_msgs/msg/AlignEvent.msg` / `FactorMsg.msg` |
| `/odom_lio` | topic | 6-DoF 里程计 | dr_lio → dr_fusion / dr_match | nav_msgs/Odometry |
| `/behavior_cmd` | topic | waypoint 微调 / 扫视指令 | dr_active → dddmr 规划器 / 控制器 | 自定义，随 dddmr 集成冻结 |
| `/align_event` | topic | 对齐成功/失败事件（日志与主动感知触发） | dr_fusion → dr_active / dr_eval | `dr_msgs/msg/AlignEvent.msg` |

## 2. 数据结构（Python 参考 / C++ 对应）

### SemanticNode（计划书 §3.1）

| 字段 | 类型 | 说明 |
|---|---|---|
| id | int32 | 节点 ID |
| cls | NodeClass | 类别枚举（见 proto） |
| conf | float64 | 提取置信度 |
| pos_xy | (2,) float64 | 全局系（离线侧）或局部队列系（机载侧） |
| yaw_hint | float64 | 节点方向性提示；无方向用 NaN |
| elev_hist | (16,) float32 | 以节点为中心 20 m 邻域高程直方图 |
| roughness | float32 | 邻域地形粗糙度 |
| degree | uint8 | 图上度（交叉口=3/4） |
| edge_angles | float32[] | 出边相对方位，排序归一化（旋转不变） |

Python 参考：`dr_match/py/dr_match/node.py`；proto：`dr_msgs/proto/node_map.proto`。

### AlignResult（A12 输出）

| 字段 | 类型 | 说明 |
|---|---|---|
| hypothesis_id | int | 假设编号（多假设时区分） |
| xy | (2,) | 全局 (x, y) 估计 |
| yaw | float | 全局 yaw（rad） |
| sigma | (3,3) | 协方差（由 score 自适应映射，B4） |
| score | float | 置信度（w1·内点率 + w2·属性相似度 − w3·RMSE） |
| inlier_ratio | float | RANSAC 内点率 |
| rmse | float | 对齐 RMSE（m） |
| correspondences | list[(local_id, global_id)] | 节点对应关系（日志/可视化） |

## 3. 数学接口（dr_fusion，B3）

SE(2) 投影先验因子（Pose3 → SE(2)）：

- 输入：机体位姿 T_wb ∈ SE(3)（6-DoF），全局测量 m = (x_m, y_m, yaw_m)；
- 残差：r = [x_b − x_m, y_b − y_m, wrap(yaw_b − yaw_m)] ∈ R³——**只约束 x/y/yaw，z/roll/pitch 永远不被全局因子触碰**；
- 雅可比：∂r/∂ξ（SE(3) 左扰动），解析式必须过数值微分校验（B3 验收）；
- Python 参考：`dr_fusion/math/dr_fusion_math/se2_projection.py`；C++ 对应（GTSAM custom factor）：`dr_fusion/include/dr_fusion/`（Ubuntu 实现）。

## 4. 置信度 → 协方差映射（B4）

接口：`dr_match/py/dr_match/confidence.py`。score ∈ R → Σ ∈ R³ˣ³。单调递减映射（score 高 → Σ 小），校准扫描见 §7.2 E1 实验。匹配失效（score 低于阈值）→ **不注入因子**（NGPS 式平滑退化）。

## 5. 行为层指令（C1–C3）

| 原语 | 触发 | 动作 | 代价 |
|---|---|---|---|
| LookAhead | 距预测节点 <10 m | 机身转向/抬头对准预测节点方位 | ~0 s |
| StopAndScan | 匹配置信度低 | 原地 360° 扫视聚合分割 | ~6 s |
| StandUp | 遮挡严重 | 后腿站立抬升视点 ~40 cm | ~3 s |
| Detour | next-best-node 收益为正 | 路径微调（≤15 m 绕行） | 绕行时间 |

行为树 XML 骨架：`dr_active/bt/*.xml`。
