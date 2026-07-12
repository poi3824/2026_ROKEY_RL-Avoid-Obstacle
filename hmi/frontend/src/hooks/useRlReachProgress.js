import { useEffect, useRef, useState } from "react";
import { socket } from "../api/socket";

// motion_executor.move_via_rl() 한 에피소드(=move_via_rl() 호출 1번)의 스텝을
// 배열로 누적한다. episode_id가 바뀌면 새 에피소드로 보고 배열을 리셋한다 -
// step==0 여부로 판단하면 이벤트 유실 시 리셋 시점을 놓칠 수 있어 id 비교로 한다
// (motion_executor.py move_via_rl()의 uuid 발급 주석 참고).
export function useRlReachProgress() {
  const [steps, setSteps] = useState([]);
  const [goalThresholdMm, setGoalThresholdMm] = useState(null);
  const [done, setDone] = useState(false);
  // tiltX/tiltY: ZAxisAlignGauge 레이더용 - 트렌드가 아니라 "가장 최근 스텝" 값만
  // 필요해서 배열로 누적하지 않는다(motion_executor.move_via_rl()의 tilt_x/tilt_y 계산
  // 주석 참고 - 목표 접근축의 로컬 X/Y축에 현재 TCP Z축을 내적한 값).
  const [tiltX, setTiltX] = useState(null);
  const [tiltY, setTiltY] = useState(null);
  const episodeIdRef = useRef(null);

  useEffect(() => {
    function onStep(data) {
      const isNewEpisode = data.episode_id !== episodeIdRef.current;
      episodeIdRef.current = data.episode_id;

      setGoalThresholdMm(data.goal_threshold_mm);
      setDone(!!data.done);
      setSteps((prev) => {
        const base = isNewEpisode ? [] : prev;
        // joint_limit_abort 등 pos_err_mm을 못 구한 종료 이벤트는 그래프에 안 찍는다.
        return data.pos_err_mm == null ? base : [...base, data.pos_err_mm];
      });
      if (data.tilt_x != null && data.tilt_y != null) {
        setTiltX(data.tilt_x);
        setTiltY(data.tilt_y);
      }
    }
    socket.on("rl_reach_progress", onStep);
    return () => socket.off("rl_reach_progress", onStep);
  }, []);

  return { steps, goalThresholdMm, done, tiltX, tiltY };
}
