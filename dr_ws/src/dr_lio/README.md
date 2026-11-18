# dr_lio —— LIO 前端（计划书 B1）与腿式里程计（B2）

**纪律：FAST-LIO2 一律 fork 配置，禁止从零实现（§6.4-C）。**

## B1 FAST-LIO2 部署（Ubuntu 20.04 + ROS1 Noetic）

1. fork `github.com/hku-mars/FAST_LIO`；
2. Go2 搭载 Livox Mid-360：按官方文档接入 Livox ROS 驱动（livox_ros_driver）；
3. 修改配置：外参标定（lidar_align + 标定板）、IMU 内参、扫描特征参数；
4. 本目录 `config/` 保存标定结果与 launch 参数；
5. 验收：仿真 5 min 闭环漂移 ≤0.5% 航程。

## B2 腿式里程计（参考 VILENS 思路，T-RO 2023）

- 本体关节编码器 + 足端接触估计 → 接触辅助不变 EKF；
- 与 LIO 松耦合，LIO 在烟尘/黑暗退化时兜底；
- 验收：LIO 人为失效 30 s 内漂移 ≤3% 航程。

## 与 dr_fusion 的关系

- FAST-LIO2 输出 6-DoF 里程计 `/odom_lio` → dr_fusion 作为 BetweenFactor(6-DoF)；
- 全局因子只约束 (x,y,yaw)（dr_fusion），z/roll/pitch 全权交给 LIO 与腿式运动学。
