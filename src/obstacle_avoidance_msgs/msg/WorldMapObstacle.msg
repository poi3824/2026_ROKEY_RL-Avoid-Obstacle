# world_map_node의 DBSCAN 클러스터링 결과, 장애물 하나.
# 좌표/길이 단위는 모두 meter. 기준 좌표계는 WorldMapUpdate.header.frame_id(기본 base_link).
int32 id

# 장애물 중심점 (cylinder 기준 중심).
geometry_msgs/Point centroid

# 장애물 bounding 정보. height/z_min/z_max는 reference_ground_z가 있으면 그 값을
# 바닥 기준으로 써서 계산한다 (스캔이 대부분 탑다운이라 클러스터 자체 z_min은
# 신뢰할 수 없음 - world_map_algo.compute_cluster_params 참고).
float32 radius
float32 height
float32 z_min
float32 z_max

# RL/motion planning에서 그대로 쓸 안전 여유 포함 값.
float32 safety_radius
float32 safety_height

# cylinder, unknown 등. 지금은 항상 cylinder로 가정.
string shape_type

# 클러스터 신뢰도/품질 판단용.
int32 num_points
float32 confidence
