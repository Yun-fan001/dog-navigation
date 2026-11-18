# NodeGraph.msg — 局部节点图 G_l(t)（滑动窗口 3–8 节点，A9）
# 节点间边 = LIO 相对位姿（边长 + 转角）
Header header
dr_msgs/SemanticNode[] nodes
uint32[] edge_src            # 与 edge_dst 等长，对应 nodes 下标
uint32[] edge_dst
float64[] edge_length
float64[] edge_dtheta
