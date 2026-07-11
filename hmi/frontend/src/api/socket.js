import { io } from "socket.io-client";

// 브라우저는 default 네임스페이스("/")로만 붙는다 - "/ros"는 hmi_ros_bridge
// 전용(Bridge Token 인증)이라 프론트엔드는 접근 권한/필요가 없다.
const SOCKET_URL = import.meta.env.VITE_SOCKET_URL;

export const socket = io(SOCKET_URL, { autoConnect: true, reconnection: true });

const COMMAND_ACK_TIMEOUT_MS = 5000;

// command.schema.json 형태로 emit하고, 같은 command_id의 command_ack를 기다린다.
// hmi/backend가 dedup을 하므로 여기서 command_id를 매번 새로 생성해서 보낸다.
export function sendCommand(action, payload = {}) {
  const commandId = crypto.randomUUID();

  return new Promise((resolve, reject) => {
    const timeoutId = setTimeout(() => {
      socket.off("command_ack", onAck);
      reject(new Error(`command_ack 타임아웃 (${action})`));
    }, COMMAND_ACK_TIMEOUT_MS);

    function onAck(ack) {
      if (ack.command_id !== commandId) return;
      clearTimeout(timeoutId);
      socket.off("command_ack", onAck);
      resolve(ack);
    }

    socket.on("command_ack", onAck);
    socket.emit("command", { command_id: commandId, action, payload });
  });
}
