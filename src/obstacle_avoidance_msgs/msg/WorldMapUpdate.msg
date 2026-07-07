# "World Map Update" 음성 명령으로 트리거된 1회성 전체 스캔 결과.
# world_map_node가 /world_map/obstacles 로 publish.
std_msgs/Header header

# 원본 point cloud/클러스터 기록이 저장된 디렉토리 (DB/UI에서 나중에 참조할 수 있도록).
string scan_dir

WorldMapObstacle[] obstacles
