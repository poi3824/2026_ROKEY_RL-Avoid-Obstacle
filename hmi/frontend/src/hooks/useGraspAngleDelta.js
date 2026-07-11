import { useEffect, useState } from "react";
import { socket } from "../api/socket";

// grasp_angle_delta.schema 없이 그냥 {angle_delta_deg, timestamp} - useSafetyStatus와
// 동일한 패턴. motion_executor.pick()이 attempt마다 즉시 발행하므로 useDbData()의
// 5초 폴링보다 먼저/자주 갱신된다(PerformancePage.jsx가 이 값을 우선 쓰고, 아직 한
// 번도 안 왔으면 기존 DB 기반 값으로 폴백).
export function useGraspAngleDelta() {
  const [deltaDeg, setDeltaDeg] = useState(null);

  useEffect(() => {
    function onGraspDelta(data) {
      setDeltaDeg(data?.angle_delta_deg ?? null);
    }
    socket.on("grasp_angle_delta", onGraspDelta);
    return () => socket.off("grasp_angle_delta", onGraspDelta);
  }, []);

  return deltaDeg;
}
