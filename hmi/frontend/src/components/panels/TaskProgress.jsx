// task_status_event.schema.json을 source별로 분리 렌더링 - {manipulation, world_map}.
// 하나의 Socket.IO 'task_status' 이벤트가 두 source를 다 실어나르므로, 상태를
// 합치지 않고 트랙 2개를 항상 나란히 보여준다(한쪽 업데이트가 다른쪽을 덮어쓰면 안 됨).
const STATUS_BADGE = {
  IDLE: "muted",
  WAITING: "warn",
  RUNNING: "good",
  COMPLETED: "good",
  FAILED: "critical",
};

function TaskTrack({ label, task }) {
  const badgeCls = STATUS_BADGE[task?.status] || "muted";
  const pct = task?.progress != null ? Math.round(task.progress * 100) : null;
  return (
    <div className="task-track">
      <div className="task-track-head">
        <span className="task-track-label">{label}</span>
        <span className={"badge " + badgeCls}>{task?.status ?? "IDLE"}</span>
      </div>
      <div className="task-track-title">{task?.title || "진행 중인 작업 없음"}</div>
      {task?.detail ? <div className="task-track-detail">{task.detail}</div> : null}
      {pct != null ? (
        <div className="task-track-bar">
          <div className="task-track-bar-fill" style={{ width: pct + "%" }} />
        </div>
      ) : null}
      {task?.step_total ? (
        <div className="task-track-steps">{task.step_index}/{task.step_total} 단계</div>
      ) : null}
    </div>
  );
}

export default function TaskProgress({ tasks }) {
  return (
    <div className="card">
      <h3>Task 진행 상태</h3>
      <div className="task-track-grid">
        <TaskTrack label="조작 (manipulation)" task={tasks?.manipulation} />
        <TaskTrack label="월드맵 (world_map)" task={tasks?.world_map} />
      </div>
    </div>
  );
}
