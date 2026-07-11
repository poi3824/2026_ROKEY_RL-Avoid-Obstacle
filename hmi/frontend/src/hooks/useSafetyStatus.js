import { useEffect, useState } from "react";
import { socket } from "../api/socket";

// safety_status.schema.json 그대로. Task 상태와 절대 같은 훅/모델로 섞지 않는다.
export function useSafetyStatus() {
  const [status, setStatus] = useState(null);

  useEffect(() => {
    function onSafetyStatus(data) {
      setStatus(data);
    }
    socket.on("safety_status", onSafetyStatus);
    return () => socket.off("safety_status", onSafetyStatus);
  }, []);

  return status;
}
