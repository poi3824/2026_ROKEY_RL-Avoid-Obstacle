const NAV_GROUPS = [
  {
    label: "모니터링",
    items: [
      { id: "dashboard", icon: "▣", label: "메인 대시보드" },
      { id: "voice", icon: "\u{1F3A4}", label: "STT-TTS" },
      { id: "vision", icon: "\u{1F4F7}", label: "Vision" },
      { id: "viewer3d", icon: "⚙", label: "World / Robot Viewer" },
    ],
  },
  {
    label: "데이터",
    items: [
      { id: "database", icon: "\u{1F5C4}", label: "DB (SQLite)" },
      { id: "performance", icon: "\u{1F4C8}", label: "Performance" },
    ],
  },
  {
    label: "설정",
    items: [{ id: "settings", icon: "⚙️", label: "설정" }],
  },
];

export default function Sidebar({ activeId, onSelect }) {
  return (
    <aside className="sidebar">
      <div className="brand">
        <div className="mark">R</div>
        <div>
          <div className="name">robot_ws HMI</div>
          <div className="sub">m0609 pick &amp; place cell</div>
        </div>
      </div>

      {NAV_GROUPS.map((group) => (
        <div key={group.label}>
          <div className="side-label">{group.label}</div>
          {group.items.map((item) => (
            <button
              key={item.id}
              type="button"
              className={"side-link" + (activeId === item.id ? " active" : "")}
              onClick={() => onSelect(item.id)}
            >
              <span className="ic">{item.icon}</span>
              {item.label}
            </button>
          ))}
        </div>
      ))}
    </aside>
  );
}

export { NAV_GROUPS };
