# FactorMsg.msg — 全局 3-DoF 软约束因子注入请求（dr_match → dr_fusion，§6.2）
# 仅约束 (x, y, yaw)；z/roll/pitch 由 LIO 负责，永远不被全局因子触碰（创新点 B）。
Header header
uint8 mode                   # 0=PRIOR(先验因子) 1=MAXMIXTURE(多假设组件)
float64 x
float64 y
float64 yaw
float64[9] sigma             # 3×3 协方差（行主序）
float64 weight               # MaxMixture 组件权重
