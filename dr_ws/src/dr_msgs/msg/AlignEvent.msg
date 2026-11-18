# AlignEvent.msg — 对齐成功/失败事件（dr_fusion → dr_active / dr_eval，§6.2）
Header header
bool success                 # 本次对齐是否注入因子
int32 hypothesis_id          # 假设编号（多假设时区分）
float64 x                    # 全局 (x, y, yaw) 估计
float64 y
float64 yaw
float64[9] sigma             # 3×3 协方差（行主序）
float64 score                # 匹配置信度
float64 inlier_ratio
float64 rmse
int32 num_nodes              # 本次匹配窗口节点数
