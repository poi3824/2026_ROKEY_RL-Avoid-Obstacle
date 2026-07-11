import { useEffect, useState } from "react";
import { socket } from "../api/socket";

// 하나의 'task_status' Socket.IO 이벤트가 manipulation/world_map 두 source를
// 전부 실어나른다(task_status_event.schema.json: {source, status}) - 여기서
// source별로 분리된 키에 저장해 한쪽 갱신이 다른쪽을 덮어쓰지 않게 한다.
export function useTaskStatus() {
  const [tasks, setTasks] = useState({ manipulation: null, world_map: null });

  useEffect(() => {
    function onTaskStatus(event) {
      const source = event?.source;
      if (source !== "manipulation" && source !== "world_map") return;
      setTasks((prev) => ({ ...prev, [source]: event.status }));
    }
    socket.on("task_status", onTaskStatus);
    return () => socket.off("task_status", onTaskStatus);
  }, []);

  return tasks;
}
