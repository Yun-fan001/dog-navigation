# SemanticNode.msg — 节点复合描述子（计划书 §3.1）
# 与 node_map.proto 的 SemanticNode 一一对应；机载层局部队列系。
int32 id
uint8 cls                    # NodeClass：0=UNKNOWN 1=ROAD_JUNCTION 2=TERRAIN_FEATURE 3=WATER_EDGE 4=HARD_GROUND 5=SURVIVING_BUILDING
float64 conf
float64 pos_x
float64 pos_y
float64 yaw_hint             # NaN 表示无方向
float32[16] elev_hist
float32 roughness
uint8 degree
float32[] edge_angles
