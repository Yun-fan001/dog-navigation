// dr_match C++ 纯库接口（无 ROS 依赖）—— 接口冻结 v0.1
//
// 与 Python 参考实现 dr_match/py/dr_match/ 一一对应（golden reference），
// 在 Ubuntu 20.04 + Eigen3 下实现与构建。禁止从零另写一套语义；
// 实现时用同一组测试数据与 Python 版交叉验证。
//
// 计划书任务：A10 匹配器（<200 ms @ N=5000,n=5）、A11 多假设、A12 对齐输出。
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace dr_match {

enum class NodeClass : uint8_t {
  UNKNOWN = 0,
  ROAD_JUNCTION = 1,
  TERRAIN_FEATURE = 2,
  WATER_EDGE = 3,
  HARD_GROUND = 4,
  SURVIVING_BUILDING = 5,
};

struct SemanticNode {
  int32_t id{0};
  NodeClass cls{NodeClass::UNKNOWN};
  double conf{1.0};
  Eigen::Vector2d pos_xy{Eigen::Vector2d::Zero()};
  double yaw_hint{std::nan("")};
  Eigen::Matrix<float, 16, 1> elev_hist;  // 20 m 邻域高程直方图（16 bin）
  float roughness{0.0f};
  uint8_t degree{0};
  std::vector<float> edge_angles;  // 排序归一化（旋转不变）
};

struct GraphEdge {
  int32_t src{0};
  int32_t dst{0};
  double length{0.0};
  double dtheta{0.0};
};

struct AlignResult {
  int hypothesis_id{-1};
  Eigen::Vector2d xy{Eigen::Vector2d::Zero()};
  double yaw{0.0};
  Eigen::Matrix3d sigma{Eigen::Matrix3d::Identity() * 1e6};  // 3x3 (x,y,yaw)
  double score{0.0};
  double inlier_ratio{0.0};
  double rmse{1e9};
  std::vector<std::pair<int32_t, int32_t>> correspondences;  // (local_id, global_id)
};

struct MatchConfig {
  int top_k{20};
  double tau_d{2.0};
  double tau_theta{0.2618};  // 15 deg
  double ransac_thr{2.0};
  int min_corr{3};
  double max_rmse{2.0};
  int max_hypotheses{5};
  int max_seeds{200};
  double min_hist_cos{0.0};
  // 置信度→协方差参数（与 confidence.py AlignConfidenceParams 对齐）
  double w_inlier{0.5}, w_attr{0.4}, w_rmse{0.1};
  double sigma_xy_max{25.0}, sigma_yaw_max{1.0472};
  double sigma_xy_min{0.5}, sigma_yaw_min{0.05236};
  double slope{3.0}, score_ref{0.8}, min_score{0.4};
};

// 邻接查询用 hash（N=5000 时的性能前提）
class NodeGraph {
 public:
  void addNode(const SemanticNode& n);
  void addEdge(const GraphEdge& e);
  bool hasEdge(int32_t a, int32_t b) const;
  std::optional<GraphEdge> edgeBetween(int32_t a, int32_t b) const;
  std::vector<int32_t> neighborsOf(int32_t id) const;
  size_t size() const;
  // ...（实现于 core/graph.cpp，Ubuntu）
};

// 种子生长式匹配器：match() 返回按 score 降序的 top-M 假设。
class GraphMatcher {
 public:
  explicit GraphMatcher(const MatchConfig& cfg = {});
  std::vector<AlignResult> match(const NodeGraph& local, const NodeGraph& global_graph);
  // ...（实现于 core/matcher.cpp，Ubuntu）
};

}  // namespace dr_match
