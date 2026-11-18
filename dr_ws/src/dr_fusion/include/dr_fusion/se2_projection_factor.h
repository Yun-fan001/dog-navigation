// dr_fusion C++ 接口 —— Pose3→SE(2) 投影先验因子（计划书 B3）
//
// GTSAM 自定义因子（Ubuntu 20.04 + GTSAM 4.2 实现）。
// 数学参考：dr_fusion/math/dr_fusion_math/se2_projection.py（数值微分校验通过）。
// 残差：r(T, m) = [x−x_m, y−y_m, wrap(yaw−yaw_m)]，只约束 x/y/yaw。
//
// 使用方式（示意）：
//   auto factor = boost::make_shared<SE2ProjectionPriorFactor>(
//       key, meas_x, meas_y, meas_yaw, noise);   // noise 来自匹配置信度(B4)
//   graph->push_back(factor);
// 多假设（A11）用 gtsam::MaxMixtureFactor 包多个组件。
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace dr_fusion {

// 测量：全局匹配输出的 (x, y, yaw)（dr_match::AlignResult）
struct SE2Measurement {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// 3-DoF 投影先验因子。GTSAM 内部用左扰动（默认），与 Python 参考一致。
class SE2ProjectionPriorFactor
    : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  SE2ProjectionPriorFactor(gtsam::Key key, const SE2Measurement& meas,
                           const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key), meas_(meas) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      gtsam::OptionalMatrixType H = nullptr) const override;

 private:
  SE2Measurement meas_;
};

}  // namespace dr_fusion
