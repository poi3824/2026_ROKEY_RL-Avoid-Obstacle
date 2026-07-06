# rl_avoidance_node가 /avoidance_cmd로 publish, robot_action_node(motion_executor)가 구독한다.
# 자세(rx,ry,rz)는 이동 구간에서 고정으로 보고 위치만 보정한다.
float32 dx
float32 dy
float32 dz

# false면 회피 개입 없음 (robot_action_node는 기존 movel 경로를 그대로 사용)
bool active
