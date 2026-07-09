# safety_monitor_node가 발행. brain(고수준 goal 제어)과 motion(실시간 폴링 반응)이 구독.
uint8 RUN=0     # 정상 진행
uint8 PAUSE=1   # 소프트 일시정지(손 감지 등). 서보 유지, 원인 해제 시 재개 가능
uint8 ESTOP=2   # 하드 정지(음성 정지/버튼). motion은 goal abort

uint8 state
string reason   # 로그/HMI용 사유 문자열
