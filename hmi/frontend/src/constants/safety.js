// safety_status.schema.json의 state 값(RUN/PAUSE/ESTOP)에 대한 공통 라벨/색상 매핑.
// TaskProgress 카드와 전역 안전 배너가 같은 의미 체계를 쓰도록 여기서만 정의한다.
export const SAFETY_META = {
  RUN: { label: "정상 진행", cls: "good" },
  PAUSE: { label: "일시정지", cls: "warn" },
  ESTOP: { label: "비상정지", cls: "critical" },
};
