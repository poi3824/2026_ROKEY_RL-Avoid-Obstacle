import { useEffect, useState } from "react";
import { socket } from "../api/socket";

export function useBridgeStatus() {
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    function onBridgeStatus(data) {
      setConnected(!!data?.connected);
    }
    socket.on("bridge_status", onBridgeStatus);
    return () => socket.off("bridge_status", onBridgeStatus);
  }, []);

  return connected;
}
